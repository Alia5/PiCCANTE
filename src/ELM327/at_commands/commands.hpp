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

class at {
        public:
    struct params;
    explicit at(out::stream& out, elm327::settings& settings,
                std::function<void(bool settings, bool timeout)> reset)
        : out(out), params(settings), reset(reset) {};
    void handle(const std::string_view command);

    bool is_at_command(const std::string_view command);

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

    void ATD(const std::string_view cmd);
    void ATZ(const std::string_view cmd);
    void ATI(const std::string_view cmd);
    void ATat1(const std::string_view cmd);
    void ATEx(const std::string_view cmd);
    void ATMA(const std::string_view cmd);
    void ATCRA(const std::string_view cmd);
    void ATMx(const std::string_view cmd);
    void ATLx(const std::string_view cmd);
    void ATCPx(const std::string_view cmd);
    void ATSH(const std::string_view cmd);
    void ATSTx(const std::string_view cmd);
    void ATSx(const std::string_view cmd);
    void ATHx(const std::string_view cmd);
    void ATSPx(const std::string_view cmd);
    void ATATx(const std::string_view cmd);
    void ATDP(const std::string_view cmd);
    void ATVx(const std::string_view cmd);
    void ATAR(const std::string_view cmd);
    void ATPC();
    void ATDESC();
    void ATRV();
};
} // namespace piccante::elm327