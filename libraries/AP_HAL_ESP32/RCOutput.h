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
 *
 * Code by Charles "Silvanosky" Villard and David "Buzz" Bussenschutt
 *
 */

#pragma once


#include <AP_HAL/RCOutput.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"
#include "driver/mcpwm.h"
#include "driver/rmt.h"
#define HAL_PARAM_DEFAULTS_PATH nullptr
#include <AP_HAL/Util.h>

namespace ESP32
{

//define dshot signal timing for dshot150
//all other timing calculates from it
#define DSHOT_TICKS_PER_BIT     534
#define DSHOT_TICKS_ZERO_HIGH   200
#define DSHOT_TICKS_ONE_HIGH    400
#define DSHOT_PACKET_LENGH      16

class RCOutput : public AP_HAL::RCOutput
{
public:
    RCOutput() {};

    ~RCOutput() {};

    static RCOutput *from(AP_HAL::RCOutput *rcoutput)
    {
        return static_cast<RCOutput *>(rcoutput);
    }

    void init() override;

    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;

    void enable_ch(uint8_t chan) override;
    void disable_ch(uint8_t chan) override;

    void write(uint8_t chan, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t* period_us, uint8_t len) override;

    void cork() override;
    void push() override;

    void set_default_rate(uint16_t rate_hz) override;

    /*
       force the safety switch on, disabling PWM output from the IO board
       */
    bool force_safety_on() override;

    /*
       force the safety switch off, enabling PWM output from the IO board
       */
    void force_safety_off() override;

    /*
       set PWM to send to a set of channels when the safety switch is
       in the safe state
       */
    void set_safety_pwm(uint32_t chmask, uint16_t period_us);

    /*
       get safety switch state, used by Util.cpp
       */
    AP_HAL::Util::safety_state _safety_switch_state();

    /*
       set PWM to send to a set of channels if the FMU firmware dies
       */
    void set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;

    /*
       set safety mask for IOMCU
       */
    void set_safety_mask(uint32_t mask)
    {
        safety_mask = mask;
    }

    /*
        mark the channels in chanmask as reversed.
        The chanmask passed is added (ORed) into any existing mask.
        The mask uses servo channel numbering
     */
    //void     set_reversed_mask(uint32_t chanmask) override;
    //uint32_t get_reversed_mask() override;

    void timer_tick() override;

    void set_output_mode(uint32_t mask, enum output_mode mode) override;
    enum output_mode get_output_mode(uint32_t& mask) override;
    void set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) override;

private:
    struct pwm_out {
        gpio_num_t gpio_num;            // GPIO number pin 
        enum output_mode current_mode;  // RC output mode (PWM NONE default)
        uint8_t chan;                   // Channel number
        uint16_t period_us;             // Channel pulse width in us

        mcpwm_unit_t        unit_num;
        mcpwm_timer_t       timer_num;
        mcpwm_io_signals_t  io_signal;
        mcpwm_operator_t    op;

        //#if HAL_WITH_BIDIR_DSHOT
        struct {
            bool is_bidirectional;      //Not used now
            rmt_channel_t rmt_channel;
            uint8_t mem_block_num;
            struct
            {
                rmt_item32_t zero;      //rmt item for dshot zero signal
                rmt_item32_t one;       //rmt item for dshot one signal
                rmt_item32_t period;    //rmt item to set signal sending period
            }timing;
            rmt_item32_t rmt_dshot_pckt[17];    //store full rmt dshot signal sequence
        }bdshot;
        //#endif
    };

    void write_int(uint8_t chan, uint16_t period_us);

    static pwm_out pwm_group_list[];

    bool _corked;
    uint16_t _pending[12]; //Max channel with 2 unit MCPWM
    uint32_t _pending_mask;
    uint32_t _reversed_mask;

    uint16_t safe_pwm[16]; // pwm to use when safety is on
    uint16_t _max_channels;

    // safety switch state
    AP_HAL::Util::safety_state safety_state;
    uint32_t safety_update_ms;
    uint8_t led_counter;
    int8_t safety_button_counter;
    uint8_t safety_press_count; // 0.1s units

    // mask of channels to allow when safety on
    uint32_t safety_mask;

    // update safety switch and LED
    void safety_update(void);

    void        rmt_init(uint8_t chan, enum output_mode mode);
    uint16_t    create_dshot_packet(const uint16_t value, bool telem_request, bool bidir_telem);
    void        dshot_packet_rmt_fill(uint16_t dshot_packet, rmt_item32_t* rmt_items, rmt_item32_t one, rmt_item32_t zero);

    bool _initialized;

};

}
