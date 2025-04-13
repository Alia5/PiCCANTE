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
#pragma once
#include <cstdint>
#include "led/led.hpp"

namespace piccante::sys::settings {
#pragma pack(push, 1)
struct system_settings {
    bool echo;
    uint8_t log_level;
    led::Mode led_mode;
    // bool wifi_enabled;
    // bool bt_enabled; ...
};
#pragma pack(pop)

bool load_settings();

const system_settings& get();

bool store();
bool get_echo();
void set_echo(bool enabled);

uint8_t get_log_level();
void set_log_level(uint8_t level);

void set_led_mode(led::Mode mode);
led::Mode get_led_mode();

} // namespace piccante::sys::settings