/*
 * PiCCANTE - Pi Car Controller Area Network Tool for Exploration
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
#include <cstddef>
#include <string_view>

namespace hex {

static unsigned int parse(char byte) {
    constexpr char base = 10;

    if (byte >= '0' && byte <= '9') {
        return byte - '0';
    }
    if (byte >= 'A' && byte <= 'F') {
        return base + byte - 'A';
    }
    if (byte >= 'a' && byte <= 'f') {
        return base + byte - 'a';
    }
    return 0;
}

static unsigned int parse(const std::string_view& str, size_t len = 0) {
    unsigned int res = 0;
    if (len == 0) {
        len = str.length();
    }
    for (size_t i = 0; i < len; i++) {
        res += parse(str[i]) << (sizeof(unsigned int) * (len - 1 - i));
    }
    return res;
}
} // namespace hex