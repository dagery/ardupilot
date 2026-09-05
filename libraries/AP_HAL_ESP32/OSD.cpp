
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_WITH_INT_OSD

#include <AP_OSD/AP_OSD_INT.h>

#include "font0.cpp"
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "esp_intr_alloc.h"

#include "esp_rom_gpio.h"

#include "soc/periph_defs.h"
#include "soc/i2s_periph.h"
#include "soc/rtc.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"

//#include <driver/periph_ctrl.h>
#include "esp_timer.h"

#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_periph.h"

#define TAG "OSD.cpp"
extern const AP_HAL::HAL &hal;

#ifndef OSD_INT_HSYNC_PIN
    #define OSD_INT_HSYNC_PIN   GPIO_NUM_36
#endif

#ifndef OSD_INT_FILL_PIN
    #define OSD_INT_FILL_PIN    GPIO_NUM_5
#endif

#ifndef OSD_INT_MASK_PIN
    #define OSD_INT_MASK_PIN    GPIO_NUM_32
#endif

#define CHARS 30
#define ROWS 16

#define FONT_WIDTH 12
#define FONT_HIGHT 18

#define FRAME_WIDTH CHARS*FONT_WIDTH
#define FRAME_HIGHT ROWS*FONT_HIGHT

#define LINE_S 22
#define LINE_F (LINE_S + (ROWS*FONT_HIGHT))

//WIDTH of framebuffer in 32bit sized chunks
#define FRAME_BUFF_WIDTH 12
//HIGHT of framebuffer based on lines count
#define FRAME_BUFF_HIGHT (LINE_F - LINE_S)

// Display memory
// chars row lengh is rounded to 32 symbols to completly fill framebuffer
// but 2 last chars should be never used (always empty)
uint8_t frame[ROWS][CHARS + 2];
//Shadow buffer, used to store last frame 
uint8_t shadow_frame[ROWS][CHARS + 2];

//Processed fonts a stored here
uint8_t char_mem[256][54];

// Pixel level framebuffer, every bit represents 1px
// if bit is 0 - pixel black, if bit set to 1 - white
uint32_t osd_frame_buff_levl[FRAME_BUFF_WIDTH * FRAME_BUFF_HIGHT]; // 288 lines, 384 px per line grouped by 32bit

// Pixel mask framebuffer, every bit represents 1px
// if bit is 0 - pixel transperent, if bit set to 1 - overrided by osd px level
uint32_t osd_frame_buff_mask[FRAME_BUFF_WIDTH * FRAME_BUFF_HIGHT]; // 288 lines, 384 px per line grouped by 32bit

uint32_t *osd_buffer_mask;
uint32_t *osd_buffer_levl;
uint32_t line = 0;

// Assembler code insert for high level interrupt, based on original OSD.cpp
// If you feel like you can optimize it futher, do it!)
// TODO in future:
// 1) HSYNC interrupt configs dma to load in I2S regs, and starts timer, that then
// starts I2S transmition.
// 2) Move to level 7 interupt (NMI)
// 3) add shadow buffer for partial updates

__asm__(R"(
    .data
_l5_intr_stack:
    .section .iram1,"ax", @progbits
    .global     xt_highint5
    .type       xt_highint5,@function
    .align      4
    .literal_position
    .literal .INT_RAW_PWM_REG,  0x3FF5E114
    .literal .line,             line
    .literal .dataval,          0x3ffb0000
    .literal .osd_buffer_mask,  osd_buffer_mask
    .literal .osd_buffer_levl,  osd_buffer_levl
    .literal .I2S0_CONF_REG,    0x3FF4F008
    .literal .I2S0_FIFO_WR_REG, 0x3FF4F000
    .literal .I2S1_CONF_REG,    0x3FF6D008
    .literal .I2S1_FIFO_WR_REG, 0x3FF6D000
    .literal .PWM_CAP_CH0_REG,  0x3FF5E0FC
    .literal .PWM_CAP,          1073078556
    .literal .line_start,       22
    .literal .line_end,         310
xt_highint5:

    #save regs that will be used in stack
    movi    a0, _l5_intr_stack
    s32i    a8, a0, 0
    s32i    a9, a0, 4
    s32i    a10, a0, 8
    s32i    a11, a0, 12
    s32i    a12, a0, 16
    s32i    a13, a0, 20
    s32i    a14, a0, 24
    s32i    a15, a0, 28
    s32i    a2, a0, 32

    #Check interrupt source 
    l32r    a8, .INT_RAW_PWM_REG
    l32i.n  a8, a8, 0
    bbci    a8, 27, end                 #if not CAP0_INT -> end

    #read cap_reg last capture time 
    l32r    a9, .PWM_CAP_CH0_REG
    l32i.n  a10, a9, 0
    blti    a10, 256, end               #if sync < 3.2us, -> end
    movi    a9, 2000 
    bge     a10, a9, long_sync          #if sync > 25us, -> long_sync -> end
    
    #here we only left with horizontal sync ~4.8us, idealy...
    l32r    a8, .line
    l32i.n  a9, a8, 0
    addi.n  a9, a9, 1                   #line++
    s32i.n  a9, a8, 0     
    l32r    a10, .line_start            #store line start in register a10
    bltu    a9, a10, end                #if line < line_start, -> end
    l32r    a11, .line_end               
    bgeu    a9, a11, end                #if line >= line_end, -> end

    l32r    a13, .osd_buffer_levl       #load osd_buffer_levl pointer address
    l32r    a14, .osd_buffer_mask       #load osd_buffer_mask pointer address
    l32i.n  a12, a13, 0                 #load address of osd_frame_buff_levl to a12
    l32i.n  a15, a14, 0                 #load address of osd_frame_buff_mask to a15
    addi    a9, a9, -22                 #line - line_s
    addx2   a9, a9, a9                  # a9 <- a9+a9*2 = 3*a9
    slli    a9, a9, 4 
    add     a13, a9, a12                #keep adress of line start of levl in a13
    add     a14, a9, a15                #keep adress of line start of mask in a14

    #reset fifo and conf_reg of I2S0
    l32r    a12, .I2S0_CONF_REG         #load perf_reg addr in register 
    l32r    a15, .I2S1_CONF_REG 
    movi.n  a8, 0
    s32i.n  a8, a12, 0                  #reset I2S0_CONF_REG to 0 
    s32i.n  a8, a15, 0                  #reset I2S1_CONF_REG to 0 
    movi.n  a8, 5                       #load 1 in register
    s32i.n  a8, a12, 0                  #reset I2S0_CONF_REG to (1 << I2S_TX_RESET_M) 
    s32i.n  a8, a15, 0                  #reset I2S1_CONF_REG to (1 << I2S_TX_RESET_M) 
    movi.n  a8, 0                       #load 0 in register
    s32i.n  a8, a12, 0                  #reset I2S0_CONF_REG to 0 
    s32i.n  a8, a15, 0                  #reset I2S1_CONF_REG to 0 

    #load initial data in .I2S0_FIFO_WR_REG
    l32i.n  a10, a13, 44                #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 44                #load value stored at osd_buffer_mask  
    l32r    a9,  .I2S0_FIFO_WR_REG      #load fifo_write_reg addr in register
    l32r    a8,  .I2S1_FIFO_WR_REG      #load fifo_write_reg addr in register                                     
    s32i.n  a11, a8, 0                  #write in fifo
    s32i.n  a10, a9, 0                  #write in fifo
    #enable transmition
    movi.n  a10, 0x10
    s32i.n  a10, a12, 0                 #enble transmition I2S0
    s32i.n  a10, a15, 0                 #enble transmition I2S1

    

    #load rest of data in .I2S0_FIFO_WR_REG
    l32i.n  a10, a13, 40                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 40                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 36                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 36                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 32                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 32                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo
    
    l32i.n  a10, a13, 28                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 28                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 24                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 24                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 20                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 20                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 16                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 16                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 12                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 12                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 8                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 8                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 4                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 4                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    l32i.n  a10, a13, 0                  #load value stored at osd_buffer_levl
    l32i.n  a11, a14, 0                  #load value stored at osd_buffer_mask                                       
    s32i.n  a11, a8, 0                   #write in fifo                                 
    s32i.n  a10, a9, 0                   #write in fifo

    movi.n  a10, 0                      #clear fifo reg
    s32i    a10, a8, 0                  # for I2S0
    s32i    a10, a9, 0                  # for I2S1
    
    j       end                         #jump to end

    long_sync:
    l32r    a8, .line                   #set line to zero = 0
    movi.n  a10, 0
    s32i.n  a10, a8, 0 

    end:
    l32r    a8, .PWM_CAP                  #clear pwm_cap intr 
    movi.n  a9, -1
    s32i.n  a9, a8, 0

    #restore used regs fromm stack
    movi    a0, _l5_intr_stack
    l32i    a8, a0, 0
    l32i    a9, a0, 4
    l32i    a10, a0, 8
    l32i    a11, a0, 12
    l32i    a12, a0, 16
    l32i    a13, a0, 20
    l32i    a14, a0, 24
    l32i    a15, a0, 28
    l32i    a2, a0, 32
    
    rsync
    memw
    rsr     a0, 213
    rfi     5
)");


void i2s_init()
{
    // I2S0 init
    periph_module_enable(PERIPH_I2S0_MODULE);
    I2S0.conf2.val = 0;
    I2S0.pdm_conf.val = 0;
    I2S0.conf_chan.tx_chan_mod = 0;
    I2S0.fifo_conf.tx_fifo_mod = 0;             // 32-bit single channel data
    I2S0.fifo_conf.dscr_en = 0;                 // no dma
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.conf.val = 0;
    I2S0.clkm_conf.clka_en = 0;
    I2S0.clkm_conf.clkm_div_a = 63;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    I2S0.sample_rate_conf.tx_bck_div_num = 11;
    I2S0.sample_rate_conf.tx_bits_mod = 16;

    // I2S1 init
    periph_module_enable(PERIPH_I2S1_MODULE);
    I2S1.conf2.val = 0;
    I2S1.pdm_conf.val = 0;
    I2S1.conf_chan.tx_chan_mod = 0;
    I2S1.fifo_conf.tx_fifo_mod = 0; // 32-bit single channel data
    I2S1.fifo_conf.dscr_en = 0;     // no dma
    I2S1.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S1.conf.val = 0;
    I2S1.clkm_conf.clka_en = 0;
    I2S1.clkm_conf.clkm_div_a = 63;
    I2S1.clkm_conf.clkm_div_b = 0;
    I2S1.clkm_conf.clkm_div_num = 2;
    I2S1.sample_rate_conf.tx_bck_div_num = 11;
    I2S1.sample_rate_conf.tx_bits_mod = 16;

    I2S1.timing.tx_sd_out_delay = 0;
    I2S0.timing.tx_sd_out_delay = 2;
}

void apll_clock_init()
{
    // Use APLL clock for driving I2S. More flexible frequency 
    // configuration and less clock jiter
    //rtc_clk_apll_enable(1);
    //rtc_clk_apll_coeff_set(0, 0, 100, 11);
}

void gpio_init()
{
    //Init HSYNC input pin
    gpio_reset_pin(OSD_INT_HSYNC_PIN);
    gpio_set_direction(OSD_INT_HSYNC_PIN, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal(OSD_INT_HSYNC_PIN, PWM0_SYNC0_IN_IDX, true);
    esp_rom_gpio_connect_in_signal(OSD_INT_HSYNC_PIN, PWM0_CAP0_IN_IDX, true);
    //Init FILL output pin
    gpio_reset_pin(OSD_INT_FILL_PIN);
    gpio_set_direction(OSD_INT_FILL_PIN, GPIO_MODE_OUTPUT);
    //Init MASK output pin
    gpio_reset_pin(OSD_INT_MASK_PIN);
    gpio_set_direction(OSD_INT_MASK_PIN, GPIO_MODE_OUTPUT);
    //Configure I2S periph outputys to GPIOs throught iomux
    esp_rom_gpio_connect_out_signal(OSD_INT_FILL_PIN, I2S0O_DATA_OUT23_IDX, false, false);
    esp_rom_gpio_connect_out_signal(OSD_INT_MASK_PIN, I2S1O_DATA_OUT23_IDX, false, false);
}

void config_mcpwm()
{
    periph_module_enable(PERIPH_PWM0_MODULE);
    MCPWM0.cap_timer_cfg.cap_timer_en = 1;
    MCPWM0.cap_timer_cfg.cap_synci_en = 1;
    MCPWM0.cap_timer_cfg.cap_synci_sel = 4; // SYNC0
    MCPWM0.cap_timer_phase.val = 0;
    MCPWM0.int_ena.cap0_int_ena = 1;
    MCPWM0.cap_chn_cfg[0].capn_en = 1;
    MCPWM0.cap_chn_cfg[0].capn_mode = (1 << 0); // When bit0 is set to 1: enable capture on the negative edge
    MCPWM0.cap_chn_cfg[0].capn_prescale = 0;
}

void config_isr()
{
    printf("osd setup isr\n");
    esp_err_t err = esp_intr_alloc(ETS_PWM0_INTR_SOURCE, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL5, NULL, NULL, NULL);
    if (err)
        printf("alloc intr error code %d\n", err);
    else
        printf("alloc intr OK!\n");
}

//Print text line on OSD (x, y, text)
void AP_OSD_INT::write(uint8_t x, uint8_t y, const char *text)
{
    if (y >= ROWS || text == NULL)
    {
        return;
    }
    while ((x < CHARS) && (*text != 0))
    {
        frame[y][x] = (uint8_t)*text;
        ++text;
        ++x;
    }
}

void AP_OSD_INT::flush()
{
    uint8_t levl_data[3];
    uint8_t mask_data[3];
    uint8_t *p_fb_levl = (uint8_t *)&osd_frame_buff_levl[0];
    uint8_t *p_fb_mask = (uint8_t *)&osd_frame_buff_mask[0];

    for (uint8_t row = 0; row < ROWS; row++)
    {
        // 3bytes nedded to draw 2 chars, so we compute by 2 chars
        // write chars pixel backwards, starting from last byte of framebuffer
        // (thnks little endian!), then it will be readed in backwards
        // so that pixel order in line is correct
        for (uint8_t colmn = 0, j = 0; colmn < CHARS + 1; colmn += 2, j += 3)
        {
            uint8_t *chr_r = &char_mem[(uint8_t)frame[row][colmn + 1]][0];
            uint8_t *chr_l = &char_mem[(uint8_t)frame[row][colmn]][0];
            //calculate offset pointers
            uint8_t *p_fb_levl_line = p_fb_levl + ((FRAME_BUFF_WIDTH * 4) -1 - j) + ((row) * 12 * 4 * FONT_HIGHT);
            uint8_t *p_fb_mask_line = p_fb_mask + ((FRAME_BUFF_WIDTH * 4) -1 - j) + ((row) * 12 * 4 * FONT_HIGHT);

            for (uint8_t l = 0; l < FONT_HIGHT; l++)
            {
                // Draw left char
                levl_data[0] = (*chr_l & 0xF0);
                mask_data[0] = (*chr_l & 0x0F) << 4;
                chr_l++;
                levl_data[0] |= (*chr_l & 0xF0) >> 4;
                mask_data[0] |= (*chr_l & 0x0F);
                chr_l++;
                levl_data[1] = (*chr_l & 0xF0);
                mask_data[1] = (*chr_l & 0x0F) << 4;
                chr_l++;
                levl_data[1] |= (*chr_r & 0xF0) >> 4;
                mask_data[1] |= (*chr_r & 0x0F);
                chr_r++;
                levl_data[2] = (*chr_r & 0xF0);
                mask_data[2] = (*chr_r & 0x0F) << 4;
                chr_r++;
                levl_data[2] |= (*chr_r & 0xF0) >> 4;
                mask_data[2] |= (*chr_r & 0x0F);
                chr_r++;

                *(p_fb_levl_line)       = levl_data[0];
                *(p_fb_levl_line - 1)   = levl_data[1];
                *(p_fb_levl_line - 2)   = levl_data[2];
                p_fb_levl_line += (FONT_WIDTH * 4);

                *(p_fb_mask_line)       = mask_data[0];
                *(p_fb_mask_line - 1)   = mask_data[1];
                *(p_fb_mask_line - 2)   = mask_data[2];
                p_fb_mask_line += (FONT_WIDTH * 4);
            }
        }
    }
}

uint8_t osd_load_fonts()
{
    // Process fonts, 1 byte code 4 pixels of font
    // 00 - black, 10 - white, 01 - transperent
    // we reorange and process bytes so that first 4 bits
    // codes px color and last 4 bits - its transperency
    // 4 high bits will be used for lelv_buff and 4 lowest -
    // for mask_buff
    
    //Now set bit shows that pixel is NOT transperent
    for (uint16_t row = 0; row < 256; row++)
    {   
        for (uint16_t indx = 0; indx < 54; indx++)
        {
            uint8_t val = 0b00001111;
            uint8_t px_data = font0[(row*54)+indx];

            if (px_data & 0b10000000)  val |=  0b10000000;
            if (px_data & 0b01000000)  val &= ~0b00001000;

            if (px_data & 0b00100000)  val |=  0b01000000;
            if (px_data & 0b00010000)  val &= ~0b00000100; 

            if (px_data & 0b00001000)  val |=  0b00100000;
            if (px_data & 0b00000100)  val &= ~0b00000010; 

            if (px_data & 0b00000010)  val |=  0b00010000;
            if (px_data & 0b00000001)  val &= ~0b00000001;
            char_mem[row][indx] = val;
               
        }
    }
    
    return 1;
}

AP_OSD_Backend *AP_OSD_INT::probe(AP_OSD &osd)
{
    printf("osd setup probr!!!!!!!!!!!!!!!!!\n");
    AP_OSD_INT *backend = new AP_OSD_INT(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

bool AP_OSD_INT::init()
{
    osd_buffer_levl = &osd_frame_buff_levl[0];
    osd_buffer_mask = &osd_frame_buff_mask[0];
    printf("osd setup start\n");
    osd_load_fonts();
    gpio_init();
    apll_clock_init();
    i2s_init();
    config_mcpwm();
    config_isr();
    printf("osd setup finish\n");
    return true;
}

float AP_OSD_INT::get_aspect_ratio_correction() const
{
    return 1.0f;
}

void AP_OSD_INT::clear()
{
    //Clear frame char buffer
    for (uint8_t row = 0; row < ROWS; row++)
    {
        for (uint8_t colmn= 0; colmn < CHARS + 2; colmn++)
        {
            frame[row][colmn] = 0;
        }   
    }
}

bool AP_OSD_INT::update_font()
{
    return true;
}

#endif