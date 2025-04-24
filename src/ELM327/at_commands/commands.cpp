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

// Protocol Numbers
// 0 - Automatic
// 1 - SAE J1850 PWM (41.6 kbaud)
// 2 - SAE J1850 VPW (10.4 kbaud)
// 3 - ISO 9141-2 (5 baud init, 10.4 kbaud)
// 4 - ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
// 5 - ISO 14230-4 KWP (fast init, 10.4 kbaud)
// 6 - ISO 15765-4 CAN (11 bit ID, 500 kbaud)
// 7 - ISO 15765-4 CAN (29 bit ID, 500 kbaud)
// 8 - ISO 15765-4 CAN (11 bit ID, 250 kbaud)
// 9 - ISO 15765-4 CAN (29 bit ID, 250 kbaud)
// A - SAE J1939 CAN (29 bit ID, 250* kbaud)
// B - USER1 CAN (11* bit ID, 125* kbaud)
// C - USER2 CAN (11* bit ID, 50* kbaud)
//  default settings (user adjustable)*

namespace {
// TODO: gather proto via extended flag and bus speed.
char current_proto = '6';
std::unordered_map<char, std::string_view> proto_map = {
    {'6', "can11/500k"},
    {'7', "can29/500k"},
    {'8', "can11/250k"},
    {'9', "can29/250k"},
};
} // namespace

void at::ATD(const std::string_view cmd) {
    params.dlc = cmd.ends_with("1");
    out << end_ok();
}

void at::ATZ([[maybe_unused]] const std::string_view cmd) {
    // todo: set to defaults (what are the defaults ???)
    out << emulator::elm_id;
    out << end_ok();
}
void at::ATI([[maybe_unused]] const std::string_view cmd) {
    out << emulator::elm_id;
    out << end_ok();
}
void at::ATat1([[maybe_unused]] const std::string_view cmd) {
    out << emulator::obdlink_desc;
    out << end_ok();
}
void at::ATEx(const std::string_view cmd) {
    params.echo = cmd.ends_with("1");
    out << end_ok();
}
void at::ATMA(const std::string_view cmd) {
    params.monitor_mode = cmd.ends_with("1");
    out << end_ok();
}
void at::ATMx(const std::string_view cmd) {
    params.memory = cmd.ends_with("1");
    out << end_ok();
}
void at::ATLx(const std::string_view cmd) {
    params.line_feed = cmd.ends_with("1");
    out << end_ok();
}

void at::ATSH(const std::string_view cmd) {
    Log::debug << "ELM327 new ECU Address: " << cmd.substr(1) << "\n";
    if (cmd.starts_with("0x")) {
        params.obd_header = hex::parse(cmd.substr(2));
    } else if (cmd.starts_with("0")) {
        params.obd_header = hex::parse(cmd.substr(1));
    } else {
        params.obd_header = hex::parse(cmd);
    }
    out << end_ok();
}

void at::ATSTx(const std::string_view cmd) {
    auto timeout = hex::parse(cmd) * 4;
    if (timeout > 0) {
        params.timeout = std::max(timeout, 60U);
    } else {
        params.timeout = 250;
    }
    out << end_ok();
}

void at::ATSx(const std::string_view cmd) {
    params.white_spaces = cmd.ends_with("1");
    out << end_ok();
}
void at::ATHx(const std::string_view cmd) {
    params.print_headers = cmd.ends_with("1");
    out << end_ok();
}
void at::ATSPx(const std::string_view cmd) {
    char proto = cmd.back();

    switch (proto) {
        case '6':
            params.obd_header = emulator::obd2_11bit_broadcast;
            params.use_extended_frames = false;
            current_proto = '6';
            // can speed 500k
            break;
        case '7':
            params.obd_header = emulator::obd2_29bit_broadcast;
            params.use_extended_frames = true;
            current_proto = '7';
            // can speed 500k
            break;
        case '8':
            params.obd_header = emulator::obd2_11bit_broadcast;
            params.use_extended_frames = false;
            current_proto = '8';
            // can speed 250k
            break;
        case '9':
            params.obd_header = emulator::obd2_29bit_broadcast;
            params.use_extended_frames = true;
            current_proto = '9';
            // can speed 250k
            break;
    }

    out << end_ok();
}
void at::ATATx(const std::string_view cmd) {
    if (cmd == "0") {
        params.adaptive_timing = false;
        params.timeout = 250;
    } else {
        params.adaptive_timing = true;
        params.timeout = 250;
    }

    out << end_ok();
}

void at::ATPC() {
    // TODO: wouldn't hurt to close teh socket
    // Terminate, whatever
    out << end_ok();
}
void at::ATDP(const std::string_view cmd) {
    if (cmd.starts_with("N")) {
        out << current_proto;
    } else {
        out << proto_map[current_proto];
    }
    out << end_ok();
}
void at::ATDESC() { out << emulator::device_desc << end_ok(); }
void at::ATRV() {
    // TODO: Could read ADC here (if implemented...)
    out << "13.4V";
    if (params.white_spaces) {
        out << " ";
    }
    out << end_ok();
}
void at::handle(const std::string_view command) {
    std::string upper_str;
    upper_str.reserve(command.size());
    for (char c :
         command | std::views::transform([](char c) { return std::toupper(c); })) {
        upper_str.push_back(c);
    }
    // check longest commands first
    if (upper_str.starts_with("ATDESC")) {
        ATDESC();
    } else if (upper_str.starts_with("ATSP")) {
        ATSPx(upper_str.substr(4));
    } else if (upper_str.starts_with("ATAT")) {
        ATATx(upper_str.substr(4));
    } else if (upper_str.starts_with("ATST")) {
        ATSTx(upper_str.substr(4));
    } else if (upper_str.starts_with("ATDP")) {
        ATDP(upper_str.substr(4));
    } else if (upper_str.starts_with("ATPC")) {
        ATPC();
    } else if (upper_str.starts_with("ATRV")) {
        ATRV();
    } else if (upper_str.starts_with("AT@1")) {
        ATat1(upper_str.substr(4));
    } else if (upper_str.starts_with("ATSH")) {
        ATSH(upper_str.substr(4));
    } else if (upper_str.starts_with("ATMA")) {
        ATMA(upper_str.substr(4));
    } else if (upper_str.starts_with("ATE")) {
        ATEx(upper_str.substr(3));
    } else if (upper_str.starts_with("ATM")) {
        ATMx(upper_str.substr(3));
    } else if (upper_str.starts_with("ATL")) {
        ATLx(upper_str.substr(3));
    } else if (upper_str.starts_with("ATS")) {
        ATSx(upper_str.substr(3));
    } else if (upper_str.starts_with("ATH")) {
        ATHx(upper_str.substr(3));
    } else if (upper_str.starts_with("ATD")) {
        ATD(upper_str.substr(3));
    } else if (upper_str.starts_with("ATZ")) {
        ATZ(upper_str.substr(3));
    } else if (upper_str.starts_with("ATI")) {
        ATI(upper_str.substr(3));
    } else {
        Log::error << "Unknown AT command: " << upper_str << "\n";
        return;
    }
    out.flush();
}
bool at::is_at_command(const std::string_view command) {
    return command.starts_with("AT") || command.starts_with("at");
}
} // namespace piccante::elm327
