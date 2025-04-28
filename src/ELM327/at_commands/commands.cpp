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
    {'6', "ISO 15765-4 (CAN 11/500)"},
    {'7', "ISO 15765-4 (CAN 29/500)"},
    {'8', "ISO 15765-4 (CAN 11/250)"},
    {'9', "ISO 15765-4 (CAN 29/250)"},
};
} // namespace

void at::ATD(const std::string_view cmd) {
    params.dlc = cmd.ends_with("1");
    out << end_ok();
}

void at::ATZ([[maybe_unused]] const std::string_view cmd) {
    reset(true, true);
    out << emulator::elm_id;
    if (params.line_feed) {
        out << "\r\n>";
    } else {
        out << "\r\r>";
    }
}
void at::ATI([[maybe_unused]] const std::string_view cmd) {
    out << emulator::elm_id;
    if (params.line_feed) {
        out << "\r\n>";
    } else {
        out << "\r\r>";
    }
}
void at::ATat1([[maybe_unused]] const std::string_view cmd) {
    out << emulator::obdlink_desc;
    if (params.line_feed) {
        out << "\r\n>";
    } else {
        out << "\r\r>";
    }
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
    if (cmd.starts_with("0x")) {
        params.obd_header = hex::parse(cmd.substr(2));
    } else if (cmd.starts_with("0")) {
        params.obd_header = hex::parse(cmd.substr(1));
    } else if (cmd.size() >= 5) {
        std::string extended_cmd = "18" + std::string(cmd);
        params.obd_header = hex::parse(extended_cmd);
    } else {
        params.obd_header = hex::parse(cmd);
    }
    Log::debug << "ELM327 new ECU Address: " << fmt::sprintf("%x", params.obd_header)
               << "\n";
    out << end_ok();
}

void at::ATSTx(const std::string_view cmd) {
    Log::debug << "ATSTx: " << cmd << "\n";
    reset(false, true);
    auto timeout = hex::parse(cmd) * 4;
    if (timeout > 0) {
        params.timeout = std::max(timeout, 60U);
    } else {
        params.timeout = 250;
    }
    Log::debug << "ATSTx: timeout set to " << params.timeout << "\n";
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

void at::ATCRA(const std::string_view cmd) {
    if (cmd.empty()) {
        // restore filters
    } else {
        // set ID filter
    }
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
    reset(false, true);
    if (cmd == "0") {
        params.adaptive_timing = false;
    } else {
        params.adaptive_timing = true;
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
void at::ATVx(const std::string_view cmd) {
    // set variable dlc on or off
    // TODO: implement
    out << end_ok();
}

void at::ATCPx(const std::string_view cmd) {
    // sets priority bits in extended mode (0x18 by default)
    // TODO: implement
    out << end_ok();
}

void at::ATAR(const std::string_view cmd) {
    // Set Auto receive address
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
    // check longest commands first
    if (command.starts_with("ATDESC")) {
        ATDESC();
    } else if (command.starts_with("ATSP")) {
        ATSPx(command.substr(4));
    } else if (command.starts_with("ATAT")) {
        ATATx(command.substr(4));
    } else if (command.starts_with("ATST")) {
        ATSTx(command.substr(4));
    } else if (command.starts_with("ATDP")) {
        ATDP(command.substr(4));
    } else if (command.starts_with("ATPC")) {
        ATPC();
    } else if (command.starts_with("ATRV")) {
        ATRV();
    } else if (command.starts_with("ATCRA")) {
        ATCRA(command.substr(5));
    } else if (command.starts_with("AT@1")) {
        ATat1(command.substr(4));
    } else if (command.starts_with("ATSH")) {
        ATSH(command.substr(4));
    } else if (command.starts_with("ATCP")) {
        ATCPx(command.substr(4));
    } else if (command.starts_with("ATMA")) {
        ATMA(command.substr(4));
    } else if (command.starts_with("ATAR")) {
        ATAR(command.substr(4));
    } else if (command.starts_with("ATE")) {
        ATEx(command.substr(3));
    } else if (command.starts_with("ATM")) {
        ATMx(command.substr(3));
    } else if (command.starts_with("ATL")) {
        ATLx(command.substr(3));
    } else if (command.starts_with("ATS")) {
        ATSx(command.substr(3));
    } else if (command.starts_with("ATH")) {
        ATHx(command.substr(3));
    } else if (command.starts_with("ATV")) {
        ATVx(command.substr(3));
    } else if (command.starts_with("ATD")) {
        ATD(command.substr(3));
    } else if (command.starts_with("ATZ")) {
        ATZ(command.substr(3));
    } else if (command.starts_with("ATI")) {
        ATI(command.substr(3));
    } else {
        if (command.starts_with("ATPP")) {
            // TODO: emulate Programmable Parameters
            out << end_ok();
            out.flush();
            return;
        }
        Log::error << "Unknown AT command: " << command << "\n";
        out << "?\r\n>";
        return;
    }
    out.flush();
}
bool at::is_at_command(const std::string_view command) {
    return command.starts_with("AT") || command.starts_with("at");
}
} // namespace piccante::elm327
