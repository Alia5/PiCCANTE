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
#include "commands.hpp"
#include <string_view>
#include <string>
#include <outstream/stream.hpp>
#include <ranges>
#include "Logger/Logger.hpp"
#include "util/hexstr.hpp"
#include "../emulator.hpp"
#include <charconv>
#include <unordered_map>

namespace piccante::elm327 {

void st::STDI(const std::string_view cmd) {
    out << "PiCCANTE ELM327 Emulator\r\r>";
    out.flush();
}

void st::handle(const std::string_view command) {
    // check longest commands first
    if (command.starts_with("STDI")) {
        STDI(command.substr(4));
    } else {
        Log::error << "Unknown ST command: " << command << "\n";
        out << "?\r\n>";
        return;
    }
    out.flush();
}
bool st::is_st_command(const std::string_view command) {
    return command.starts_with("ST") || command.starts_with("st");
}
} // namespace piccante::elm327
