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
#include "Logger.hpp"
#include <functional>
#include <ostream>
#include <map>

namespace piccante {

Log::Level Log::current_level = Log::LEVEL_INFO;

std::reference_wrapper<std::ostream> Log::out = std::cout;
std::reference_wrapper<std::ostream> Log::err = std::cerr;

std::map<Log::Level, std::string> Log::level_names = {
    {Log::LEVEL_DEBUG, "DEBUG"},
    {Log::LEVEL_INFO, "INFO"},
    {Log::LEVEL_WARNING, "WARNING"},
    {Log::LEVEL_ERROR, "ERROR"},
};
void Log::init(Level level, std::ostream& out_stream, std::ostream& err_stream) {
    Log::current_level = level;
    Log::out = out_stream;
    Log::err = err_stream;
}
void Log::set_log_level(Level level) { Log::current_level = level; }
}
