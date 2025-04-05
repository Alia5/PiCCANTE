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
#include <span>
#include <ostream>
#include "../../CanBus/CanBus.hpp"


namespace piccante::gvret {

// My implementation of the gvret protocol (afaik) originally by Collin Kidder (Collin80)
// Based on the original (buggy...) code from:
// Protocol details (implementations can be found at:
// https://github.com/collin80/M2RET/blob/master/CommProtocol.txt
// esp32 arduino impl: https://github.com/collin80/ESP32RET/blob/master/gvret_comm.h
// SavvyCAN host impl:
// https://github.com/collin80/SavvyCAN/blob/master/connections/gvretserial.cpp
//
// check handler and states for more details
// binary protocol enum in proto.hpp
uint8_t check_sum(const std::span<uint8_t>& buff);

void comm_can_frame(uint busnumber, const can2040_msg& frame, std::ostream& out);

static constexpr auto SPEED_MASK = 0xFFFFFU;
static constexpr auto BUS_FLAG_MASK = 0x80000000UL;
static constexpr auto ENABLED_MASK = 0x40000000UL;
static constexpr auto LISTEN_ONLY_MASK = 0x20000000UL;
static constexpr auto MAX_BUS_SPEED = 1000000U;

} // namespace gvret