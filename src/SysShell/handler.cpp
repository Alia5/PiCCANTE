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
#include <string>
#include <charconv>
#include "outstream/usb_cdc_stream.hpp"
#include "CanBus/CanBus.hpp"
#include "settings.hpp"
#include "Logger/Logger.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "stats/stats.hpp"
#include "CommProto/gvret/handler.hpp"

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
    {"binary", //
     {"Toggle GVRET binary mode (binary <on|off>)", &handler::cmd_toggle_binary}},
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
    {"sys_stats", //
     {"Display system information and resource usage (sys_stats [cpu|heap|tasks|uptime])",
      &handler::cmd_sys_stats}},
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

void handler::cmd_toggle_binary(const std::string_view& arg) {
    if (arg == "on") {
        gvret.set_binary_mode(true);
        host_out << "Binary mode enabled\n";
    } else if (arg == "off") {
        gvret.set_binary_mode(false);
        host_out << "Binary mode disabled\n";
    } else {
        host_out << "Binary mode: " << (gvret.get_binary_mode() ? "on" : "off") << "\n";
        host_out << "Usage: binary <on|off>\n";
    }
}

void handler::cmd_can_enable(const std::string_view& arg) {
    int bus = 0;
    unsigned int bitrate = 0;

    // Find the first space to separate arguments
    auto space_pos = arg.find(' ');
    if (space_pos != std::string_view::npos) {
        // Parse first argument (bus)
        auto bus_str = arg.substr(0, space_pos);
        auto [bus_ptr, bus_ec] =
            std::from_chars(bus_str.data(), bus_str.data() + bus_str.size(), bus);

        // Parse second argument (bitrate)
        auto bitrate_str = arg.substr(space_pos + 1);
        auto [bitrate_ptr, bitrate_ec] = std::from_chars(
            bitrate_str.data(), bitrate_str.data() + bitrate_str.size(), bitrate);

        // Check if both conversions succeeded
        if (bus_ec == std::errc() && bitrate_ec == std::errc()) {
            if (bus >= 0 && bus < can::get_num_busses()) {
                host_out << "Enabling CAN bus " << bus << " with bitrate " << bitrate
                         << "\n";
                piccante::can::enable(bus, bitrate);
            } else {
                host_out << "Invalid bus number. Valid range: 0-"
                         << (can::get_num_busses() - 1) << "\n";
            }
            return;
        }
    }

    host_out << "Usage: can_enable <bus> <bitrate>\n";
}
void handler::cmd_can_disable(const std::string_view& arg) {
    int bus = 0;
    auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), bus);

    if (ec == std::errc()) {
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

    auto space_pos = arg.find(' ');
    if (space_pos != std::string_view::npos) {
        auto bus_str = arg.substr(0, space_pos);
        auto [bus_ptr, bus_ec] =
            std::from_chars(bus_str.data(), bus_str.data() + bus_str.size(), bus);

        auto bitrate_str = arg.substr(space_pos + 1);
        auto [bitrate_ptr, bitrate_ec] = std::from_chars(
            bitrate_str.data(), bitrate_str.data() + bitrate_str.size(), bitrate);

        if (bus_ec == std::errc() && bitrate_ec == std::errc()) {
            if (bus >= 0 && bus < can::get_num_busses()) {
                host_out << "Setting CAN bus " << bus << " bitrate to " << bitrate
                         << "\n";
                piccante::can::set_bitrate(bus, bitrate);
            } else {
                host_out << "Invalid bus number. Valid range: 0-"
                         << (can::get_num_busses() - 1) << "\n";
            }
            return;
        }
    }

    host_out << "Usage: can_bitrate <bus> <bitrate>\n";
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
    int wanted = 0;
    auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), wanted);

    if (ec == std::errc()) {
        // Conversion succeeded
        if (wanted > 0 && wanted <= piccanteNUM_CAN_BUSSES) {
            host_out << "Setting number of CAN buses to " << wanted << "\n";
            piccante::can::set_num_busses(wanted);
        } else {
            host_out << "Invalid number of buses. Valid range: 1-"
                     << piccanteNUM_CAN_BUSSES << "\n";
        }
    } else {
        // Conversion failed or no argument provided
        host_out << "Current number of CAN buses: " << piccante::can::get_num_busses()
                 << "\n"
                 << "Valid Range: 1-" << piccanteNUM_CAN_BUSSES << "\n";
        host_out << "Usage: can_num_busses <number>\n";
    }
}

void handler::cmd_settings_show([[maybe_unused]] const std::string_view& arg) {
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

void handler::cmd_settings_store([[maybe_unused]] const std::string_view& arg) {
    if (settings::store()) {
        host_out << "Settings saved successfully\n";
    } else {
        host_out << "Failed to save settings\n";
    }
}

void handler::cmd_log_level(const std::string_view& arg) {
    int level = 0;
    auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), level);

    if (ec == std::errc()) {
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

void handler::cmd_sys_stats([[maybe_unused]] const std::string_view& arg) {
    bool show_all = arg.empty();
    bool show_memory = show_all || arg == "heap" || arg == "memory";
    bool show_tasks = show_all || arg == "tasks";
    bool show_cpu = show_all || arg == "cpu";
    bool show_uptime = show_all || arg == "uptime";

    // Check for invalid argument
    if (!show_all && !show_memory && !show_tasks && !show_cpu && !show_uptime) {
        host_out << "Unknown parameter: " << arg << "\n";
        host_out << "Usage: sys_stats [section]\n";
        host_out << "Available sections: cpu, heap, tasks, uptime\n";
        host_out << "If no section is specified, all information is displayed.\n";
        return;
    }

    host_out << "\nSYSTEM INFORMATION\n";
    host_out << "------------------\n\n";

    if (show_memory) {
        const auto memory = piccante::sys::stats::get_memory_stats();
        host_out << "Memory:\n";
        host_out << "  Free heap:       " << memory.free_heap << " bytes\n";
        host_out << "  Min free heap:   " << memory.min_free_heap << " bytes\n";
        host_out << "  Heap used:       " << memory.heap_used << " bytes";
        host_out << " (" << memory.heap_usage_percentage << "%)\n\n";
    }
    if (show_tasks) {
        const auto tasks = piccante::sys::stats::get_task_stats();
        host_out << "Task Statistics:\n";
        host_out << "---------------\n";
        host_out << "Name                        State  Prio   Stack   Num     Core\n";
        host_out << "----------------------------------------------------------------\n";

        for (const auto& task : tasks) {
            std::string name = task.name;
            if (name.length() > 27) {
                name = name.substr(0, 24) + "...";
            }
            host_out << name;
            for (size_t j = name.length(); j < 29; j++) {
                host_out << " ";
            }

            char state = 'U'; // Unknown
            switch (task.state) {
                case eTaskState::eRunning:
                    state = 'R';
                    break;
                case eTaskState::eReady:
                    state = 'r';
                    break;
                case eTaskState::eBlocked:
                    state = 'B';
                    break;
                case eTaskState::eSuspended:
                    state = 'S';
                    break;
                case eTaskState::eDeleted:
                    state = 'D';
                    break;
                default:
                    state = 'X';
                    break;
            }
            host_out << state << "      ";
            host_out << static_cast<int>(task.priority);
            for (size_t j = piccante::sys::stats::num_digits(task.priority); j < 7; j++) {
                host_out << " ";
            }

            host_out << task.stack_high_water;
            for (size_t j = piccante::sys::stats::num_digits(task.stack_high_water);
                 j < 8;
                 j++) {
                host_out << " ";
            }

            host_out << task.task_number;
            for (size_t j = piccante::sys::stats::num_digits(task.task_number); j < 8;
                 j++) {
                host_out << " ";
            }
            host_out << fmt::sprintf("0x%x\n", task.core_affinity);
        }
        host_out << "\n";
    }

    if (show_cpu) {
        const auto cpu_stats = piccante::sys::stats::get_cpu_stats(true);
        host_out << "CPU Usage:\n";
        host_out << "----------\n";
        host_out << "Task            Current %\n";
        host_out << "-------------------------\n";

        auto total_usage = std::array<float, NUM_CORES>{};
        if (cpu_stats.empty()) {
            host_out << "First measurement - run command again for results\n";
        } else {
            for (const auto& task : cpu_stats) {
                std::string name = task.name;
                if (name.length() > 15) {
                    name = name.substr(0, 12) + "...";
                }
                host_out << name;
                for (size_t j = name.length(); j < 17; j++) {
                    host_out << " ";
                }

                if (task.cpu_usage < 1.0f) {
                    host_out << "<1%";
                } else {
                    host_out << static_cast<int>(task.cpu_usage) << "%";
                }
                host_out << "\n";

                if (task.name.find("IDLE") == std::string::npos) {
                    const auto core =
                        task.core_affinity > NUM_CORES ? 0 : task.core_affinity - 1;
                    total_usage[core] += task.cpu_usage;
                }
            }
        }
        host_out << "\n";
        host_out << "Total CPU usage: \n";
        for (size_t i = 0; i < NUM_CORES; i++) {
            if (i > 0) {
                host_out << "\n";
            }
            host_out << "Core " << i << ": ";
            if (total_usage[i] < 1.0F) {
                host_out << "<1%";
            } else {
                host_out << static_cast<int>(total_usage[i]) << "%";
            }
        }
        host_out << "\n\n";
    }

    if (show_uptime) {
        auto uptime = piccante::sys::stats::get_uptime();

        host_out << "System Uptime: ";
        if (uptime.days > 0) {
            host_out << uptime.days << " days, ";
        }
        host_out << uptime.hours << " hours, " << uptime.minutes << " minutes, "
                 << uptime.seconds << " seconds\n\n";
    }
    host_out.flush();
}

} // namespace piccante::sys::shell