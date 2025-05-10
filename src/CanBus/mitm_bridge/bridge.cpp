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
#include "bridge.hpp"
#include "../CanBus.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace piccante::can::mitm::bridge {

namespace {
std::pair<uint8_t, uint8_t> bridged{0, 0};
SemaphoreHandle_t bridge_mtx = xSemaphoreCreateMutex();
} // namespace

void set_bridge(uint8_t bus1, uint8_t bus2) {
    if (xSemaphoreTake(bridge_mtx, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (bus1 > NUM_BUSSES || bus2 > NUM_BUSSES) {
        xSemaphoreGive(bridge_mtx);
        return;
    }
    bridged = {bus1, bus2};

    store_bridge_settings(bridged);
    xSemaphoreGive(bridge_mtx);
}

bool handle(uint8_t bus, const can::frame& frame) {
    if (xSemaphoreTake(bridge_mtx, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    if (bridged.first == bridged.second) {
        xSemaphoreGive(bridge_mtx);
        return false;
    }
    bool sent = false;
    if (bus == bridged.first) {
        can::send_can(bridged.second, frame);
        sent = true;
    }
    if (bus == bridged.second) {
        can::send_can(bridged.first, frame);
        sent = true;
    }
    xSemaphoreGive(bridge_mtx);
    return sent;
}

} // namespace piccante::can::mitm::bridge