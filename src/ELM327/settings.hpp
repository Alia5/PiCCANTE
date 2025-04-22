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
#include <cstdint>

namespace piccante::elm327 {
struct settings {
    uint32_t obd_header;
    uint32_t timeout;
    bool line_feed;
    bool echo;
    bool white_spaces;
    bool dlc;
    bool monitor_mode;
    bool memory;
    bool print_headers;
    bool use_extended_frames;
    bool adaptive_timing;
};
} // namespace piccante::elm327