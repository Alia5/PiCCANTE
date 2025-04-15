/*
 * PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
 * Copyright (C) 2025 Peter Repukat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "sleep.hpp"
#include "led/led.hpp"
#include "Logger/Logger.hpp"
#include "hardware/structs/scb.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include <hardware/watchdog.h>
#include <pico/multicore.h>
#include <pico/sleep.h>

#include "SysShell/settings.hpp"
#ifdef WIFI_ENABLED
#include "wifi/wifi.hpp"
#include <pico/cyw43_arch.h>
#include <cyw43.h>
#include <cyw43_ll.h>
#endif

namespace piccante::power::sleep {

namespace {
bool sleeping = false;
TimerHandle_t idle_timer = nullptr;
TaskHandle_t idle_task_handle = nullptr;

void idle_timer_callback(TimerHandle_t timer) {
    if (!sleeping) {
        Log::info << "CAN idle timeout reached, entering sleep mode\n";
        enter_sleep_mode();
    }
}

void idle_detection_task(void* params) {
    (void)params;

    Log::info << "Idle detection task started\n";

    for (;;) {
        const auto idle_minutes = sys::settings::get_idle_sleep_minutes();
        if (idle_minutes > 0 && !sleeping) {
            if (idle_timer == nullptr) {
                idle_timer = xTimerCreate("IdleTimer",
                                          pdMS_TO_TICKS(idle_minutes * 60 * 1000),
                                          pdFALSE, // One-shot timer
                                          nullptr,
                                          idle_timer_callback);

                if (idle_timer == nullptr) {
                    Log::error << "Failed to create idle timer\n";
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }

                if (xTimerStart(idle_timer, 0) != pdPASS) {
                    Log::warning << "Failed to start idle timer\n";
                }

                Log::info << "Idle timer started: " << idle_minutes << " minutes\n";
            } else {
                xTimerChangePeriod(
                    idle_timer, pdMS_TO_TICKS(idle_minutes * 60 * 1000), 0);
            }
        } else if (idle_timer != nullptr && (idle_minutes == 0 || sleeping)) {
            xTimerStop(idle_timer, 0);
            xTimerDelete(idle_timer, 0);
            idle_timer = nullptr;

            if (idle_minutes == 0) {
                Log::info << "Idle timeout disabled\n";
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
} // namespace

void init() {
    BaseType_t result = xTaskCreate(idle_detection_task,
                                    "IdleDetect",
                                    configMINIMAL_STACK_SIZE,
                                    nullptr,
                                    tskIDLE_PRIORITY + 1,
                                    &idle_task_handle);

    if (result != pdPASS) {
        Log::error << "Failed to create idle detection task\n";
        return;
    }

    vTaskCoreAffinitySet(idle_task_handle, 0x01);
}

void reset_idle_timer() {
    if (!sleeping && idle_timer != nullptr) {
        xTimerReset(idle_timer, 0);
    }
}


void enter_sleep_mode() {
    if (sleeping) {
        return;
    }

    sleeping = true;

    Log::info << "Going to sleep\n";

    led::set_mode(led::MODE_OFF);

#ifdef WIFI_ENABLED
    wifi::stop();
    cyw43_arch_deinit();
#endif


    // Allow log to flush / other core to sleep.
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskSuspendAll();

    multicore_reset_core1();

    sleep_run_from_rosc();
    sleep_goto_dormant_until_edge_high(piccanteCAN0_RX_PIN);

    // If we get here, the device has woken up
    wake_up();
}

void wake_up() {
    if (!sleeping) {
        return;
    }

    sleeping = false;
    Log::info << "Waking up from deep sleep\n";

    watchdog_reboot(0, SRAM_END, 10);

    // Instead of restoring all functionality,
    // it's way easier to just reset the board entirely.
    // So for a first implementation, this is what we'll do
    // We loose a few CAN-Frames but for an initial implementation this is fine.
    // TODO: proper wakeup routine. Maybe.
}


} // namespace piccante::power::sleep