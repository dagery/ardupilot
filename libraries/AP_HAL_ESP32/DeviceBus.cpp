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

#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


using namespace ESP32;

extern const AP_HAL::HAL& hal;

DeviceBus::DeviceBus(uint8_t _thread_priority) :
    thread_priority(_thread_priority), semaphore()
{
#ifdef BUSDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
}

/*
  per-bus callback thread
*/

// volatile static uint64_t last = 0;
// volatile static uint64_t now = 0;
// volatile static uint64_t delta = 0;
// volatile static uint64_t counter = 0;
// void IRAM_ATTR DeviceBus::bus_thread(void *arg)
// {
// #ifdef BUSDEBUG
//     printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
// #endif
//     struct DeviceBus *binfo = (struct DeviceBus *)arg;

//     while (true) {
//         uint64_t now = AP_HAL::micros64();
//         DeviceBus::callback_info *callback;

//         // find a callback to run
//         for (callback = binfo->callbacks; callback; callback = callback->next) {
//             if (now >= callback->next_usec) {
//                 while (now >= callback->next_usec) {
//                     callback->next_usec += callback->period_usec;
//                 }
//                 // call it with semaphore held
//                 if (binfo->semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
//                     now = esp_timer_get_time();

//                     delta = last - now;
//                     if((counter++) % 500 == 0)
//                     {
//                         printf("callb d:%lld\n", delta);
//                     }
//                     last = now;
//                     callback->cb();
//                     binfo->semaphore.give();
//                 }
//             }
//         }

//         // work out when next loop is needed
//         uint64_t next_needed = 0;
//         now = AP_HAL::micros64();

//         for (callback = binfo->callbacks; callback; callback = callback->next) {
//             if (next_needed == 0 ||
//                 callback->next_usec < next_needed) {
//                 next_needed = callback->next_usec;
//                 if (next_needed < now) {
//                     next_needed = now;
//                 }
//             }
//         }

//         // delay for at most 50ms, to handle newly added callbacks
//         uint32_t delay = 50000;
//         if (next_needed >= now && next_needed - now < delay) {
//             delay = next_needed - now;
//         }
//         // don't delay for less than 100usec, so one thread doesn't
//         // completely dominate the CPU
//         if (delay < 100) {
//             delay = 100;
//         }
//         hal.scheduler->delay_microseconds(delay);
//     }
//     return;
// }

// AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
// {
// #ifdef BUSDEBUG
//     printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
// #endif
//     if (!thread_started) {
//         thread_started = true;
//         hal_device = _hal_device;
//         // setup a name for the thread
//         char name[configMAX_TASK_NAME_LEN];
//         switch (hal_device->bus_type()) {
//         case AP_HAL::Device::BUS_TYPE_I2C:
//             snprintf(name, sizeof(name), "APM_I2C:%u",
//                      hal_device->bus_num());
//             break;

//         case AP_HAL::Device::BUS_TYPE_SPI:
//             snprintf(name, sizeof(name), "APM_SPI:%u",
//                      hal_device->bus_num());
//             break;
//         default:
//             break;
//         }
// #ifdef BUSDEBUG
//         printf("%s:%d Thread Start\n", __PRETTY_FUNCTION__, __LINE__);
// #endif
//         // xTaskCreate(DeviceBus::bus_thread, name, Scheduler::DEVICE_SS,
//         //             this, thread_priority, &bus_thread_handle);
//         xTaskCreatePinnedToCore(DeviceBus::bus_thread, name, Scheduler::DEVICE_SS,
//                     this, thread_priority, &bus_thread_handle, 1);
//     }
//     DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
//     if (callback == nullptr) {
//         return nullptr;
//     }
//     callback->cb = cb;
//     callback->period_usec = period_usec;
//     callback->next_usec = AP_HAL::micros64() + period_usec;

//     // add to linked list of callbacks on thread
//     callback->next = callbacks;
//     callbacks = callback;

//     return callback;
// }


// volatile static uint64_t last = 0;
// volatile static uint64_t now = 0;
// volatile static uint64_t delta = 0;
// volatile static uint64_t counter = 0;

void IRAM_ATTR DeviceBus::periodic_timer_callback(void* arg)
{
   

    //etc_printf();
    DeviceBus::callback_info *callback = (DeviceBus::callback_info*)arg;
    
    //now = esp_timer_get_time();
    callback->cb();
    //delta = now - last;
    // if((counter++) % 1000 == 0)
    // {
    //     printf("callb d:%lld\n", delta);
    // }
    // last = now;
}

//Use timer based periodic callbacks for SPI devices
AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{

    DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;

    esp_timer_create_args_t periodic_timer_args = {
            .callback = &DeviceBus::periodic_timer_callback,
            .arg = callback,
            //.dispatch_method = ESP_TIMER_ISR,   //!< Call the callback from task or from ISR
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    /* Start the timer */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, period_usec));
    return 0;
}

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle) {
        return false;
    }
    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    return true;
}
