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
#include <cstddef>
#include <string_view>

namespace piccante::hex {
/**
 * @brief Converts a hex character to its integer value.
 *
 * @param byte The hex character to convert.
 * @return The integer value of the hex character.
 */
inline unsigned int parse(char byte) {
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
/**
 * @brief Converts a string of hex values to integer value: 'DEADBEEF' -> 3735928559.
 *
 * @param str The hex string to convert.
 * @param len The length of the hex string. If 0, the length is determined automatically.
 * @return The integer value of the hex string.
 */
inline unsigned int parse(const std::string_view& str) {
    unsigned int res = 0;
    for (size_t i = 0; i < str.length(); i++) {
        res += parse(str[i]) << (sizeof(unsigned int) * (str.length() - 1 - i));
    }
    return res;
}
} // namespace piccante::hex