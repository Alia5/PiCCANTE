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
#include <outstream/stream.hpp>
#include <functional>
#include "../settings.hpp"

namespace piccante::elm327 {

class st {
        public:
    struct params;
    explicit st(out::stream& out, elm327::settings& settings,
                std::function<void(bool settings, bool timeout)> reset)
        : out(out), params(settings), reset(reset) {};
    void handle(const std::string_view command);

    bool is_st_command(const std::string_view command);

        private:
    out::stream& out;
    elm327::settings& params;
    std::function<void(bool all, bool timeout)> reset;

    inline std::string_view end_ok() {
        if (params.line_feed) {
            return "\r\nOK\r\n>";
        }
        return "\rOK\r\r>";
    };

    void STDI(const std::string_view cmd);
};
} // namespace piccante::elm327