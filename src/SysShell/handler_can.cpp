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

#include <cstdint>
#include <can2040.h>

#include "handler.hpp"
#include "CanBus/CanBus.hpp"
#include "CanBus/mitm_bridge/bridge.hpp"
#include "settings.hpp"

namespace piccante::sys::shell {


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
    host_out.flush();
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
    host_out.flush();
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
            host_out.flush();
            return;
        }
    }

    host_out << "Usage: can_bitrate <bus> <bitrate>\n";
    host_out.flush();
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
    host_out.flush();
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
    host_out.flush();
}


void handler::cmd_can_baud_lockout(const std::string_view& arg) {
    if (arg == "on") {
        settings::set_baudrate_lockout(true);
        host_out << "CAN baud rate lockout enabled\n";
    } else if (arg == "off") {
        settings::set_baudrate_lockout(false);
        host_out << "CAN baud rate lockout disabled\n";
    } else {
        host_out << "Usage: can_lock_rate <on|off>\n";
    }
    host_out.flush();
}


void handler::cmd_can_bridge(const std::string_view& arg) {
    if (arg == "off") {
        piccante::can::mitm::bridge::set_bridge(0, 0);
        host_out << "CAN bridge disabled\n";
        host_out.flush();
        return;
    }
    const auto bus1 = arg.substr(0, arg.find(' '));
    const auto bus2 = arg.substr(arg.find(' ') + 1);
    if (bus1.empty() || bus2.empty()) {
        host_out << "Usage: can_bridge <bus1> <bus2>\n";
        host_out.flush();
        return;
    }
    int bus1_num = 0;
    int bus2_num = 0;
    auto [ptr1, ec1] = std::from_chars(bus1.data(), bus1.data() + bus1.size(), bus1_num);
    auto [ptr2, ec2] = std::from_chars(bus2.data(), bus2.data() + bus2.size(), bus2_num);
    if (ec1 == std::errc() && ec2 == std::errc()) {
        if (bus1_num >= 0 && bus1_num < can::get_num_busses() && bus2_num >= 0 &&
            bus2_num < can::get_num_busses()) {
            host_out << "Bridging CAN bus " << bus1_num << " and CAN bus " << bus2_num
                     << "\n";
            piccante::can::mitm::bridge::set_bridge(bus1_num, bus2_num);
        } else {
            host_out << "Invalid bus number. Valid range: 0-"
                     << (can::get_num_busses() - 1) << "\n";
        }
    } else {
        host_out << "Usage: can_bridge <bus1> <bus2>\n";
    }
    host_out.flush();
}

} // namespace piccante::sys::shell