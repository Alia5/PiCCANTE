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
#include <span>

#include "ff_headers.h"

namespace piccante::sd {
bool mount();
bool unmount();

bool is_mounted();

FF_FILE* open_file(const std::string_view& filename, const std::string_view& mode);
void close_file(FF_FILE* file);

bool read_file(FF_FILE* file, std::span<char, std::dynamic_extent> buffer);
bool write_file(FF_FILE* file, const std::span<char, std::dynamic_extent> data);
bool write_file(FF_FILE* file, const std::string_view data);


} // namespace piccante::sd