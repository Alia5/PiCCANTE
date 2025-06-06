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
#include "led.hpp"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef WIFI_ENABLED
#include "pico/cyw43_arch.h"
#endif

namespace piccante::led {

namespace {
Mode current_mode = MODE_CAN;

#ifdef WIFI_ENABLED
bool cyw43_initialized = false;
#else
// For non-W variants, we use GPIO 25 which is the onboard LED
constexpr uint8_t PICO_LED_PIN = 25;
#endif

constexpr TickType_t toggle_duration_ms = 50;

bool should_blink = false;

void set(bool state) {
#ifndef WIFI_ENABLED
    gpio_put(PICO_LED_PIN, state);
#else
    if (!cyw43_initialized) {
        return;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
#endif
}
void blink_task(void*) {
    for (;;) {
        if (should_blink) {
            set(false);
            vTaskDelay(pdMS_TO_TICKS(toggle_duration_ms));
            set(true);
            should_blink = false;
            vTaskDelay(pdMS_TO_TICKS(toggle_duration_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

} // namespace
void init(Mode mode) {
    current_mode = mode;
#ifndef WIFI_ENABLED
    gpio_init(PICO_LED_PIN);
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
    gpio_put(PICO_LED_PIN, current_mode > MODE_OFF ? 1 : 0);
#else
    cyw43_initialized = true;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, current_mode > MODE_OFF ? 1 : 0);
#endif
    static TaskHandle_t handle;
    xTaskCreate(blink_task, "led_blink", configMINIMAL_STACK_SIZE / 2, nullptr,
                tskIDLE_PRIORITY + 2, &handle);
}

void set_mode(Mode mode) {
    current_mode = mode;

    switch (mode) {
        case MODE_OFF:
            set(false);
            break;
        case MODE_PWR:
            set(true);
            break;
        case MODE_CAN:
            set(true);
            break;
    }
}

Mode get_mode() { return current_mode; }

void blink() {
    if (current_mode != MODE_CAN) {
        return;
    }
    should_blink = true;
}


} // namespace piccante::led