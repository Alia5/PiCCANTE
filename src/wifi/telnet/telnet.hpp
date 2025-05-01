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
#include "FreeRTOS.h"
#include "queue.h"
#include "outstream/stream.hpp"

namespace piccante::wifi::telnet {

bool initialize();
void stop();
bool reconfigure();
bool is_running();
QueueHandle_t get_rx_queue();
out::base_sink& get_sink();

bool enable();
void disable();
bool set_port(uint16_t port);

} // namespace piccante::wifi::telnet