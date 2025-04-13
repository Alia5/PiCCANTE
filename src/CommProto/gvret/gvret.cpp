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
#include "gvret.hpp"
#include <cstdint>
#include <pico/types.h>
#include <numeric>
#include "outstream/stream.hpp"
#include "../../CanBus/CanBus.hpp"
#include "can2040.h"
#include "proto.hpp"
#include "util/bin.hpp"
#include <pico/time.h>
namespace piccante::gvret {

uint8_t check_sum(const std::span<uint8_t>& buff) {
    return std::accumulate(buff.begin(), buff.end(), 0);
}


} // namespace piccante::gvret