
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

#include <memory>
#include <cstdint>
#include "emulator.hpp"
#include "SysShell/settings.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "outstream/usb_cdc_stream.hpp"
#ifdef WIFI_ENABLED
#include "wifi/telnet/telnet_server.hpp"
#include "wifi/bt_spp/bt_spp.hpp"
#endif

namespace piccante::elm327 {

enum class interface : uint8_t {
    USB = 0,
    Bluetooth = 1,
    WiFi = 2,
};
void start();
void stop();

void reconfigure();

elm327::emulator* emu();
QueueHandle_t queue();


} // namespace piccante::elm327