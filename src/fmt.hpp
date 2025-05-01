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

#include <string>
#include <string_view>
#include <cstdio>

namespace fmt
{

template <typename... Args>
static std::string sprintf(const std::string_view& fmtstr, const Args&... args) {
    std::string result;
    // NOLINTBEGIN // stolen from cppreference
    size_t size = std::snprintf(nullptr, 0, fmtstr.data(), args...);
    // NOLINTEND
    if (size <= 0) {
        return "";
        // throw std::runtime_error("Error during formatting.");
    }
    result.resize(size);
    // NOLINTNEXTLINE // stolen from cppreference
    if (std::sprintf(result.data(), fmtstr.data(), args...) < 0) {
        return "";
        // throw std::runtime_error("Error during formatting.");
    }
    return result;
}
}