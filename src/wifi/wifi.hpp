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

#include <string_view>

#include "FreeRTOS.h"
#include "task.h"
#include <optional>
#include <string>
#include <cstdint>

namespace piccante::wifi {

enum class Mode : uint8_t {
    NONE,
    CLIENT,      //
    ACCESS_POINT //
};

struct WifiStats {
    Mode mode;
    bool connected;
    std::string ssid;
    uint8_t channel;
    int32_t rssi;
    std::string ip_address;
    std::string mac_address;
};

struct ConnectionStatus {
    std::string state;
    std::string ssid;
    int error_code = 0;
};

TaskHandle_t task();
int connect_to_network(const std::string_view& ssid, const std::string_view& password,
                       uint32_t timeout_ms = 10000, TaskHandle_t task_handle = nullptr);
bool start_ap(const std::string_view& ssid, const std::string_view& password = "",
              uint8_t channel = 1);
bool is_connected();

std::optional<WifiStats> wifi_stats();
void stop();
ConnectionStatus get_connection_status();
bool cancel_connection();

} // namespace piccante::wifi