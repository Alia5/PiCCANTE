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
#include <string>
#include "led/led.hpp"

namespace piccante::sys::settings {
#pragma pack(push, 1)
struct system_settings {
    bool echo;
    uint8_t log_level;
    led::Mode led_mode;
    uint8_t wifi_mode;
    uint8_t idle_sleep_minutes;
    // bool bt_enabled; ...
};
#pragma pack(pop)

#ifdef WIFI_ENABLED
struct wifi_settings {
    std::string ssid;
    std::string password;
    uint8_t channel;
    struct telnet_settings {
        uint16_t port;
        bool enabled;
    } telnet{};
};
#endif

bool load_settings();

const system_settings& get();

bool store();
bool get_echo();
void set_echo(bool enabled);

uint8_t get_log_level();
void set_log_level(uint8_t level);

void set_led_mode(led::Mode mode);
led::Mode get_led_mode();

uint8_t get_idle_sleep_minutes();
void set_idle_sleep_minutes(uint8_t minutes);

#ifdef WIFI_ENABLED

uint8_t get_wifi_mode();
void set_wifi_mode(uint8_t mode);
const wifi_settings& get_wifi_settings();

void set_wifi_ssid(const std::string& ssid);
void set_wifi_password(const std::string& password);
void set_wifi_channel(uint8_t channel);
uint16_t get_telnet_port();
void set_telnet_port(uint16_t port);
bool telnet_enabled();
void set_telnet_enabled(bool enabled);

#endif

} // namespace piccante::sys::settings