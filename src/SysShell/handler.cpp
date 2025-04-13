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
#include "handler.hpp"
#include "outstream/usb_cdc_stream.hpp"
#include <string>
#include "CanBus/CanBus.hpp"

namespace piccante::sys::shell {

void handler::process_byte(char byte) {
    if (cfg.echo) {
        host_out << byte;
        host_out.flush();
    }
    if (byte == '\n' || byte == '\r') {
        handle_cmd(std::string_view(buffer.data(), buffer.size()));
        buffer.clear();
    } else {
        buffer.push_back(byte);
    }
}

void handler::handle_cmd(const std::string_view& cmd) {
    if (cmd.empty()) {
        return;
    }
    std::size_t space_pos = cmd.find(' ');
    std::string_view command =
        (space_pos != std::string_view::npos) ? cmd.substr(0, space_pos) : cmd;
    std::string_view arg =
        (space_pos != std::string_view::npos) ? cmd.substr(space_pos + 1) : "";

    auto it = commands.find(command);
    if (it != commands.end()) {
        it->second(this, arg);
    } else {
        host_out << "\nUnknown command: " << command << "\n";
    }
    host_out.flush();
}

std::map<std::string_view, handler::CommandInfo, std::less<>> handler::commands = {
    {"echo", //
     {"Toggle command echo (echo <on|off>)", &handler::cmd_echo}},
    {"help", //
     {"Display this help message", &handler::cmd_help}},
    {"can_enable", //
     {"Enable CAN bus with specified bitrate (can_enable <bus> <bitrate>)",
      &handler::cmd_can_enable}},
    {"can_disable", //
     {"Disable CAN bus (can_disable <bus>)", &handler::cmd_can_disable}},
    {"can_bitrate", //
     {"Change CAN bus bitrate (can_bitrate <bus> <bitrate>)", &handler::cmd_can_bitrate}},
    {"can_status", //
     {"Show status of CAN buses", &handler::cmd_can_status}},
    {"set_num_busses", //
     {"Set number of CAN buses (can_num_busses [number])", &handler::cmd_can_num_busses}},
    {"settings", {"Show current system settings", &handler::cmd_settings_show}},
    {"save", {"Save current settings to flash", &handler::cmd_settings_store}},
    {"log_level",
     {"Set log level (log_level <0-3>). 0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR",
      &handler::cmd_log_level}},
};

void handler::cmd_echo(const std::string_view& arg) {
    if (arg == "on") {
        settings::set_echo(true);
        host_out << "Echo enabled\n";
    } else if (arg == "off") {
        settings::set_echo(false);
        host_out << "Echo disabled\n";

    } else {
        host_out << "Usage: echo <on|off>\n";
    }
}

void handler::cmd_help([[maybe_unused]] const std::string_view& arg) {
    size_t max_cmd_length = 0;
    for (const auto& [cmd, info] : commands) {
        max_cmd_length = std::max(max_cmd_length, cmd.length());
    }

    max_cmd_length += 2;

    host_out << "\nAvailable Commands:\n";
    host_out << "------------------\n\n";

    for (const auto& [cmd, info] : commands) {
        const int padding = static_cast<int>(max_cmd_length - cmd.length());

        host_out << cmd;
        for (int i = 0; i < padding; i++) {
            host_out << ' ';
        }
        host_out << "- " << info.description << "\n";
    }

    host_out << "\n";
}

void handler::cmd_can_enable(const std::string_view& arg) {
    int bus = 0;
    unsigned int bitrate = 0;
    if (std::sscanf(arg.data(), "%d %u", &bus, &bitrate) == 2) {
        if (bus >= 0 && bus < can::get_num_busses()) {
            host_out << "Enabling CAN bus " << bus << " with bitrate " << bitrate << "\n";
            piccante::can::enable(bus, bitrate);
        } else {
            host_out << "Invalid bus number. Valid range: 0-"
                     << (can::get_num_busses() - 1) << "\n";
        }
    } else {
        host_out << "Usage: can_enable <bus> <bitrate>\n";
    }
}

void handler::cmd_can_disable(const std::string_view& arg) {
    int bus = 0;
    if (std::sscanf(arg.data(), "%d", &bus) == 1) {
        if (bus >= 0 && bus < can::get_num_busses()) {
            host_out << "Disabling CAN bus " << bus << "\n";
            piccante::can::disable(bus);
        } else {
            host_out << "Invalid bus number. Valid range: 0-"
                     << (can::get_num_busses() - 1) << "\n";
        }
    } else {
        host_out << "Usage: can_disable <bus>\n";
    }
}

void handler::cmd_can_bitrate(const std::string_view& arg) {
    int bus = 0;
    unsigned int bitrate = 0;
    if (std::sscanf(arg.data(), "%d %u", &bus, &bitrate) == 2) {
        if (bus >= 0 && bus < can::get_num_busses()) {
            host_out << "Setting CAN bus " << bus << " bitrate to " << bitrate << "\n";
            piccante::can::set_bitrate(bus, bitrate);
        } else {
            host_out << "Invalid bus number. Valid range: 0-"
                     << (can::get_num_busses() - 1) << "\n";
        }
    } else {
        host_out << "Usage: can_bitrate <bus> <bitrate>\n";
    }
}

void handler::cmd_can_status([[maybe_unused]] const std::string_view& arg) {
    host_out << "\nCAN BUS STATUS\n";
    host_out << "-------------\n\n";

    host_out << "Max supported buses: " << static_cast<int>(piccanteNUM_CAN_BUSSES)
             << "\n";
    host_out << "Enabled buses:        " << static_cast<int>(can::get_num_busses())
             << "\n\n";

    for (uint8_t i = 0; i < can::get_num_busses(); i++) {
        const bool enabled = piccante::can::is_enabled(i);
        const uint32_t bitrate = piccante::can::get_bitrate(i);
        const uint32_t rx_buffered = piccante::can::get_can_rx_buffered_frames(i);
        const uint32_t tx_buffered = piccante::can::get_can_tx_buffered_frames(i);
        const uint32_t rx_overflow = piccante::can::get_can_rx_overflow_count(i);

        host_out << "Bus " << static_cast<int>(i) << ":\n";
        host_out << "  Status:      " << (enabled ? "Enabled" : "Disabled") << "\n";

        if (enabled) {
            host_out << "  Bitrate:     " << bitrate << " bps\n";
            host_out << "  RX buffered: " << rx_buffered << "\n";
            host_out << "  TX buffered: " << tx_buffered << "\n";

            if (rx_overflow > 0) {
                host_out << "  RX overflow: " << rx_overflow << "\n";
            }

            // Add can2040 statistics
            can2040_stats stats;
            if (can::get_statistics(i, stats)) {
                host_out << "  Statistics:\n";
                host_out << "    RX total:     " << stats.rx_total << "\n";
                host_out << "    TX total:     " << stats.tx_total << "\n";
                host_out << "    TX attempts:  " << stats.tx_attempt << "\n";
                host_out << "    Parse errors: " << stats.parse_error << "\n";
            }
        }

        if (i < can::get_num_busses() - 1) {
            host_out << "\n";
        }
    }

    host_out << "\n";
}

void handler::cmd_can_num_busses(const std::string_view& arg) {
    int wanted;
    if (sscanf(arg.data(), "%d", &wanted) == 1) {
        if (wanted > 0 && wanted <= piccanteNUM_CAN_BUSSES) {
            host_out << "Setting number of CAN buses to " << wanted << "\n";
            piccante::can::set_num_busses(wanted);
        } else {
            host_out << "Invalid number of buses. Valid range: 1-"
                     << piccanteNUM_CAN_BUSSES << "\n";
        }
    } else {
        host_out << "Current number of CAN buses: " << piccante::can::get_num_busses()
                 << "\n"
                 << "Valid Range: 1-" << piccanteNUM_CAN_BUSSES << "\n";
        host_out << "Usage: can_num_busses <number>\n";
    }
}

void handler::cmd_settings_show(const std::string_view& arg) {
    host_out << "\nSystem Settings:\n";
    host_out << "--------------\n\n";

    constexpr int label_width = 15;

    host_out << "Echo:";
    for (int i = 0; i < label_width - 5; i++)
        host_out << ' ';
    host_out << (cfg.echo ? "On" : "Off") << "\n";

    std::string level_str{};
    if (cfg.log_level == 0)
        level_str = "DEBUG";
    else if (cfg.log_level == 1)
        level_str = "INFO";
    else if (cfg.log_level == 2)
        level_str = "WARNING";
    else
        level_str = "ERROR";

    host_out << "Log level:";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << static_cast<int>(cfg.log_level) << " (" << level_str << ")\n";

    host_out << "CAN buses:";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << static_cast<int>(piccante::can::get_num_busses()) << "\n\n";
}

void handler::cmd_settings_store(const std::string_view& arg) {
    if (settings::store()) {
        host_out << "Settings saved successfully\n";
    } else {
        host_out << "Failed to save settings\n";
    }
}

void handler::cmd_log_level(const std::string_view& arg) {
    int level = 0;
    if (sscanf(arg.data(), "%d", &level) == 1) {
        if (level >= 0 && level <= 3) {
            settings::set_log_level(static_cast<uint8_t>(level));
            host_out << "Log level set to " << level << " (";
            host_out << (level == 0 ? "DEBUG"
                                    : (level == 1 ? "INFO"
                                                  : (level == 2 ? "WARNING" : "ERROR")));
            host_out << ")\n";
        } else {
            host_out << "Invalid log level. Valid values: 0-3\n";
        }
    } else {
        host_out << "Current log level: " << static_cast<int>(cfg.log_level) << " ("
                 << (cfg.log_level == 0
                         ? "DEBUG"
                         : (cfg.log_level == 1
                                ? "INFO"
                                : (cfg.log_level == 2 ? "WARNING" : "ERROR")))
                 << ")\n";
        host_out << "Usage: log_level <0-3>\n";
        host_out << "  0: DEBUG\n";
        host_out << "  1: INFO\n";
        host_out << "  2: WARNING\n";
        host_out << "  3: ERROR\n";
    }
}

} // namespace piccante::sys::shell