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
#include <cstddef>
#include <cstdint>
#include <can2040.h>
#include <hardware/platform_defs.h>
#include <hardware/regs/addressmap.h>
#include <string>
#include <charconv>
#include "led/led.hpp"
#include "CanBus/CanBus.hpp"
#include "settings.hpp"
#include "task.h"
#include "stats/stats.hpp"
#include "CommProto/gvret/handler.hpp"
#include <hardware/watchdog.h>
#include "power/sleep.hpp"
#include "ELM327/elm.hpp"
#ifdef WIFI_ENABLED
#include "wifi/wifi.hpp"
#endif
#include <ranges>

namespace piccante::sys::shell {

void handler::process_byte(char byte) {
    if (byte == '\b' || byte == 127) {
        if (!buffer.empty()) {
            buffer.pop_back();
            if (cfg.echo) {
                host_out << "\b \b";
                host_out.flush();
            }
        }
        return;
    }
    if (byte == 0) {
        return; // Ignore null bytes
    }

    if (byte == 3) { // Ctrl+C
        cancel_requested = true;
        if (cfg.echo) {
            host_out << "\n";
            host_out.flush();
        }
        return;
    }

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

bool handler::check_and_reset_cancel() {
    bool was_cancelled = cancel_requested;
    cancel_requested = false;
    return was_cancelled;
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

    std::string lowercase_command;
    lowercase_command.reserve(command.size());
    std::ranges::transform(command, std::back_inserter(lowercase_command),
                           [](unsigned char c) { return std::tolower(c); });

    auto it = commands.find(lowercase_command);
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
    {"can_lock_rate", //
     {"Prevent the can bus speed from changing via GVRET/SLCAN (can_lock_rate <on|off>)",
      &handler::cmd_can_baud_lockout}},
    {"can_bridge", //
     {"Bridge CAN interfaces (can_bridge <off | <bus1> <bus2>>)",
      &handler::cmd_can_bridge}},
    {"settings", {"Show current system settings", &handler::cmd_settings_show}},
    {"save", {"Save current settings to flash", &handler::cmd_settings_store}},
    {"led_mode", //
     {"Set LED mode (led_mode <0-3>) 0=OFF, 1=Power, 2=CAN Activity",
      &handler::cmd_led_mode}},
    {"log_level",
     {"Set log level (log_level <0-3>). 0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR",
      &handler::cmd_log_level}},
    {"sys_stats", //
     {"Display system information and resource usage (sys_stats "
#ifdef WIFI_ENABLED
      "[cpu|heap|fs|tasks|uptime|adc|wifi])",
#else
      "[cpu|heap|fs|tasks|uptime|adc])",
#endif
      &handler::cmd_sys_stats}},
    {"version", //
     {"Display version information (version)", &handler::cmd_version}},
#ifdef WIFI_ENABLED
    {"wifi", //
     {"Manage WiFi settings (wifi info | wifi connect <ssid> <password> | wifi ap "
      "<ssid> "
      "<password> "
      "<channel> | wifi disable)",
      &handler::cmd_wifi}},
    {"telnet", //
     {"Enable or disable Telnet and set port (telnet enable|disable | telnet <port>)",
      &handler::cmd_telnet}},
#endif
    {"reset", //
     {"Reset the system (reset)", &handler::cmd_reset}},
    {"sleep", //
     {"Enter deep sleep mode (sleep)", &handler::cmd_sleep}},
    {"idle_timeout", //
     {"Set idle timeout in minutes (idle_timeout disable|<minutes>)",
      &handler::cmd_idle_timeout}},
    {"elm", //
#ifdef WIFI_ENABLED
     {"Configure ELM327 interface and mode (elm <usb|bt PIN|wifi>  <can0|can1|can2>)",
#else
     {"Configure ELM327 interface and mode (elm <can0|can1|can2>)",
#endif
      &handler::cmd_elm}},
    {"atz", //
     {"Enable ELM mode (if ELM327-Emulator is on USB)", &handler::cmd_elm_atz}},
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
    host_out.flush();
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
    host_out.flush();
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
    host_out.flush();
}


void handler::cmd_settings_show([[maybe_unused]] const std::string_view& arg) {
    host_out << "\nSystem Settings:\n";
    host_out << "--------------\n\n";

    constexpr int label_width = 30;

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


    std::string led_mode_str{};
    if (cfg.led_mode == 0)
        led_mode_str = "Off";
    else if (cfg.led_mode == 1)
        led_mode_str = "Power";
    else if (cfg.led_mode == 2)
        led_mode_str = "Can Activity";

    host_out << "LED mode: ";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << static_cast<int>(cfg.led_mode) << " (" << led_mode_str << ")\n";

    host_out << "CAN buses:";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << static_cast<int>(piccante::can::get_num_busses()) << "\n";

    host_out << "CAN speed lock:";
    for (int i = 0; i < label_width - 15; i++)
        host_out << ' ';
    host_out << (cfg.baudrate_lockout ? "Enabled" : "Disabled") << "\n";

    host_out << "Idle timeout:";
    for (int i = 0; i < label_width - 13; i++)
        host_out << ' ';
    host_out << (settings::get_idle_sleep_minutes() == 0
                     ? "off"
                     : std::to_string(settings::get_idle_sleep_minutes()))
             << " minutes\n";


    host_out << "ELM327 CAN bus:";
    for (int i = 0; i < label_width - 15; i++)
        host_out << ' ';
    host_out << static_cast<int>(cfg.elm_can_bus) << "\n";

#ifdef WIFI_ENABLED

    const auto wifi_cfg = settings::get_wifi_settings();

    host_out << "WiFi mode:";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << (cfg.wifi_mode == 0 ? "off" : std::to_string(cfg.wifi_mode)) << "\n";
    host_out << "WiFi SSID:";
    for (int i = 0; i < label_width - 10; i++)
        host_out << ' ';
    host_out << settings::get_wifi_settings().ssid << "\n";
    host_out << "WiFi Channel (AP):";
    for (int i = 0; i < label_width - 18; i++)
        host_out << ' ';
    host_out << static_cast<int>(settings::get_wifi_settings().channel) << "\n";
    host_out << "Telnet:";
    for (int i = 0; i < label_width - 7; i++)
        host_out << ' ';
    host_out << (settings::get_wifi_settings().telnet.enabled ? "Enabled" : "Disabled")
             << "\n";
    host_out << "Telnet port:";
    for (int i = 0; i < label_width - 12; i++)
        host_out << ' ';
    host_out << static_cast<int>(settings::get_wifi_settings().telnet.port) << "\n";

    host_out << "ELM327 interface:";
    for (int i = 0; i < label_width - 17; i++)
        host_out << ' ';
    host_out << (wifi_cfg.elm_interface == 0
                     ? "USB"
                     : (wifi_cfg.elm_interface == 1 ? "Bluetooth" : "WiFi"))
             << "\n";
    host_out << "Bluetooth PIN:";
    for (int i = 0; i < label_width - 14; i++)
        host_out << ' ';
    host_out << static_cast<int>(wifi_cfg.bluetooth_pin) << "\n";
#endif

    host_out << "\n";
    host_out.flush();
}

void handler::cmd_settings_store([[maybe_unused]] const std::string_view& arg) {
    if (settings::store()) {
        host_out << "Settings saved successfully\n";
    } else {
        host_out << "Failed to save settings\n";
    }
    host_out.flush();
}

void handler::cmd_led_mode(const std::string_view& arg) {
    led::Mode mode = led::MODE_OFF;
    auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(),
                                     reinterpret_cast<uint8_t&>(mode));
    if (ec == std::errc()) {
        if (mode >= led::MODE_OFF && mode <= led::Mode::MODE_CAN) {
            settings::set_led_mode(mode);
            host_out << "LED mode set to " << static_cast<int>(mode) << "\n";
        } else {
            host_out << "Invalid LED mode. Valid values: 0-3\n";
        }
    } else {
        host_out << "Current LED mode: " << static_cast<int>(cfg.led_mode) << "\n";
        host_out << "Usage: led_mode <0-3>\n";
        host_out << "  0: OFF\n";
        host_out << "  1: Power\n";
        host_out << "  2: CAN Activity\n";
    }
    host_out.flush();
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
    host_out.flush();
}

void handler::cmd_sys_stats([[maybe_unused]] const std::string_view& arg) {
    bool show_all = arg.empty();
    bool show_memory = show_all || arg == "heap" || arg == "memory";
    bool show_tasks = show_all || arg == "tasks";
    bool show_cpu = show_all || arg == "cpu";
    bool show_uptime = show_all || arg == "uptime";
    bool show_fs = show_all || arg == "fs";
    bool show_adc = show_all || arg == "adc";
#ifdef WIFI_ENABLED
    bool show_wifi = show_all || arg == "wifi";
#endif


    // Check for invalid argument
    if (!show_all && !show_memory && !show_tasks && !show_cpu && !show_uptime &&
        !show_fs && !show_adc
#ifdef WIFI_ENABLED
        && !show_wifi
#endif
    ) {
        host_out << "Unknown parameter: " << arg << "\n";
        host_out << "Usage: sys_stats [section]\n";
        host_out << "Available sections: cpu, heap, fs, tasks, uptime, adc";
#ifdef WIFI_ENABLED
        host_out << ", wifi";
#endif
        host_out << "\n";
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

    if (show_fs) {
        const auto fs_stats = piccante::sys::stats::get_filesystem_stats();
        host_out << "Filesystem (LittleFS):\n";
        host_out << "  Total size:    " << fs_stats.total_size << " bytes";
        host_out << " (" << (fs_stats.total_size / 1024) << " KiB)\n";
        host_out << "  Used space:    " << fs_stats.used_size << " bytes";
        host_out << " (" << (fs_stats.used_size / 1024) << " KiB)\n";
        host_out << "  Free space:    " << fs_stats.free_size << " bytes";
        host_out << " (" << (fs_stats.free_size / 1024) << " KiB)\n";
        host_out << "  Usage:         " << static_cast<int>(fs_stats.usage_percentage)
                 << "%\n";
        host_out << "  Block size:    " << fs_stats.block_size << " bytes\n";
        host_out << "  Block count:   " << fs_stats.block_count << "\n\n";
    }

    stats::TaskStats task_stats;

    if (show_tasks || show_cpu) {
        task_stats = piccante::sys::stats::get_task_stats();
    }

    if (show_tasks) {
        const auto tasks = piccante::sys::stats::get_task_stats();
        host_out << "Task Statistics:\n";
        host_out << "---------------\n";
        host_out << "Name                    State  Prio   Stack   ID      CPU     "
                    "CORE0    "
                    "CORE1      Affinity\n";
        host_out << "-------------------------------------------------------------------"
                    "------"
                    "-----------------\n";

        for (const auto& task : tasks.tasks) {
            std::string name{task.name};
            if (name.length() > 23) {
                name = name.substr(0, 20) + "...";
            }
            host_out << name;
            for (size_t j = name.length(); j < 25; j++) {
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
            host_out << " " << state;
            host_out << fmt::sprintf("%7d", task.priority);
            host_out << " " << fmt::sprintf("%7d", task.stack_high_water);
            host_out << " " << fmt::sprintf("%5d", task.task_number);
            host_out << " "
                     << fmt::sprintf("%7.1f", task.cpu_usage[0] + task.cpu_usage[1])
                     << "%";
            host_out << " " << fmt::sprintf("%7.1f", task.cpu_usage[0]) << "%";
            host_out << " " << fmt::sprintf("%7.1f", task.cpu_usage[1]) << "%";
            host_out << " "
                     << fmt::sprintf("%15s",
                                     fmt::sprintf("0x%x\n", task.core_affinity).c_str());
        }
        host_out << "\n";
    }

    if (show_cpu) {
        host_out << "CPU Usage:\n";
        host_out << "----------\n";
        for (size_t i = 0; i < NUM_CORES; i++) {
            if (i > 0) {
                host_out << "\n";
            }
            host_out << "Core " << i << ": ";
            host_out << static_cast<int>(task_stats.cores[i]) << "%";
        }
        host_out << "\n\n";
    }

    if (show_adc) {
        host_out << "ADC Stats: \n";
        host_out << "-------------\n";

        const auto adc_stats = stats::get_adc_stats();
        for (const auto& stat : adc_stats) {
            if (stat.channel == 3) {
                // System voltage with extra detail
                host_out << fmt::sprintf("%s: %.3f %s (Raw: %d, GPIO%d)\n",
                                         stat.name.c_str(), stat.value, stat.unit.c_str(),
                                         stat.raw_value, stat.channel + 26);
            } else if (stat.channel == 4) {
                // Temperature sensor (no GPIO)
                host_out << fmt::sprintf("%s: %.1f %s (Raw: %d)\n", stat.name.c_str(),
                                         stat.value, stat.unit.c_str(), stat.raw_value);
            } else {
                // Regular ADC channels
                host_out << fmt::sprintf("%s: %.3f %s (Raw: %d, GPIO%d)\n",
                                         stat.name.c_str(), stat.value, stat.unit.c_str(),
                                         stat.raw_value, stat.channel + 26);
            }
        }
        host_out << "\n";
    }

#ifdef WIFI_ENABLED
    if (show_wifi) {
        const auto wifi_mode = static_cast<wifi::Mode>(sys::settings::get_wifi_mode());
        if (wifi_mode == wifi::Mode::NONE) {
            host_out << "WiFi is disabled\n";
        } else {
            const auto wifi_stats = wifi::wifi_stats();
            if (wifi_stats) {
                host_out << "WiFi Mode: ";
                if (wifi_mode == wifi::Mode::CLIENT) {
                    host_out << "Client\n";
                } else {
                    host_out << "Access Point\n";
                }
                host_out << "SSID: " << wifi_stats->ssid << "\n";
                host_out << "Channel: " << static_cast<int>(wifi_stats->channel) << "\n";
                host_out << "RSSI: " << wifi_stats->rssi << "\n";
                host_out << "IP Address: " << wifi_stats->ip_address << "\n";
                host_out << "MAC Address: " << wifi_stats->mac_address << "\n\n";
            } else {
                host_out << "Failed to retrieve WiFi statistics\n";
            }
        }
    }
    host_out << "\n";
#endif

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

void handler::cmd_reset([[maybe_unused]] const std::string_view& arg) {
    host_out << "Resetting system...\n";
    host_out.flush();
    watchdog_reboot(0, SRAM_END, 10);
}

void handler::cmd_sleep([[maybe_unused]] const std::string_view& arg) {
    host_out << "Entering deep sleep mode...\n";
    host_out.flush();
    piccante::power::sleep::enter_sleep_mode();
}

void handler::cmd_idle_timeout(const std::string_view& arg) {
    if (arg == "disable") {
        settings::set_idle_sleep_minutes(0);
        host_out << "Idle timeout disabled\n";
    } else {
        int timeout = 0;
        auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), timeout);
        if (ec == std::errc()) {
            if (timeout > 0) {
                settings::set_idle_sleep_minutes(timeout);
                host_out << "Idle timeout set to " << timeout << " minutes\n";
            } else {
                host_out << "Invalid timeout value. Must be greater than 0.\n";
            }
        } else {
            host_out << "Current idle timeout: "
                     << std::to_string(settings::get_idle_sleep_minutes())
                     << " minutes\n";
            host_out << "Usage: idle_timeout disable|<minutes>\n";
        }
    }
    host_out.flush();
}

void handler::cmd_elm(const std::string_view& cmd) {
    std::ranges::split_view splitted{cmd, ' '};
    auto it = splitted.begin();
    const auto end = splitted.end();

    if (it == end) {
#ifdef WIFI_ENABLED
        host_out << "Configure ELM327 interface and mode (elm <usb|bt PIN|wifi>  "
                    "<can0|can1|can2>)\n";
#else
        host_out << "Configure ELM327 interface and mode (elm <can0|can1|can2>)\n";
#endif
        host_out.flush();
        return;
    }

    elm327::stop();

    const std::string_view interface{(*it).data(), (*it).size()};
#ifdef WIFI_ENABLED
    const auto& wifi_cfg = settings::get_wifi_settings();
    if (interface == "usb") {
        settings::set_elm_interface(static_cast<uint8_t>(elm327::interface::USB));
    } else if (interface == "bt") {
        settings::set_elm_interface(static_cast<uint8_t>(elm327::interface::Bluetooth));
        ++it;
        const std::string_view pin{(*it).data(), (*it).size()};
        if (pin.empty()) {
            host_out << "Bluetooth PIN is required\n";
            host_out.flush();

            return;
        }
        int pin_value = 0;
        auto [ptr, ec] = std::from_chars(pin.data(), pin.data() + pin.size(), pin_value);
        if (ec != std::errc()) {
            host_out << "Invalid Bluetooth PIN\n";
            host_out.flush();

            return;
        }
        settings::set_bluetooth_pin(pin_value);
    } else if (interface == "wifi") {
        settings::set_elm_interface(static_cast<uint8_t>(elm327::interface::WiFi));
    } else {
        host_out << "Invalid ELM327 interface. Valid values: usb, bt, wifi\n";
        host_out.flush();

        return;
    }
    ++it;
#endif
    const std::string_view can_bus{(*it).data(), (*it).size()};
    if (can_bus == "can0") {
        settings::set_elm_can_bus(0);
    } else if (can_bus == "can1") {
        settings::set_elm_can_bus(1);
    } else if (can_bus == "can2") {
        settings::set_elm_can_bus(2);
    } else {
        host_out << "Invalid CAN bus. Valid values: can0, can1, can2\n";
        host_out.flush();

        return;
    }

    host_out << "\n";
    host_out.flush();

#ifdef WIFI_ENABLED
    if (wifi_cfg.elm_interface != static_cast<uint8_t>(elm327::interface::USB)) {
        elm327::start();
    }
#endif
}

void handler::cmd_elm_atz([[maybe_unused]] const std::string_view& arg) {
    const auto& cfg = settings::get();


#ifdef WIFI_ENABLED
    const auto& wifi_cfg = settings::get_wifi_settings();
    if (wifi_cfg.elm_interface == static_cast<uint8_t>(elm327::interface::USB)) {
#else
    if (true) {
#endif
        host_out << "ELM327 v1.4\r\n";
        elm327::start();
    } else {
        host_out << "ELM327 not enabled on USB\n";
    }

    host_out.flush();
}

void handler::cmd_version([[maybe_unused]] const std::string_view& arg) {
    host_out << "PiCCANTE version: " << PICCANTE_VERSION << "\n";
    host_out << "Build date: " << __DATE__ << "\n";
    host_out << "Build time: " << __TIME__ << "\n";
    host_out.flush();
}


} // namespace piccante::sys::shell