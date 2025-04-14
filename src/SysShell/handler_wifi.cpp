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
#ifdef WIFI_ENABLED


#include "handler.hpp"
#include <string>
#include <charconv>
#include "settings.hpp"
#include "task.h"
#include "wifi/wifi.hpp"
#include <cyw43.h>
#include "wifi/telnet/telnet.hpp"
namespace piccante::sys::shell {

void handler::cmd_wifi(const std::string_view& arg) {
    const auto command = arg.substr(0, arg.find(' '));
    const auto params = arg.substr(arg.find(' ') + 1);
    if (command == "info") {
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
    } else if (command == "connect") {
        auto ssid_end = params.find(' ');
        if (ssid_end == std::string_view::npos) {
            host_out
                << "Error: Missing password. Usage: wifi connect <ssid> <password>\n";
            host_out.flush();
            return;
        }

        std::string ssid{params.substr(0, ssid_end).begin(),
                         params.substr(0, ssid_end).end()};
        if (ssid.empty()) {
            host_out << "Error: SSID cannot be empty\n";
            host_out.flush();
            return;
        }

        std::string password{params.substr(ssid_end + 1).begin(),
                             params.substr(ssid_end + 1).end()};


        host_out << "Connecting to WiFi network: " << ssid << "\n";
        host_out.flush();
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        const auto result = wifi::connect_to_network(ssid, password, 30000, current_task);

        uint8_t dots = 0;
        bool cancelled = false;

        vTaskDelay(pdMS_TO_TICKS(200));
        auto status = wifi::get_connection_status();

        while (status.state != "CONNECTED") {
            if (check_and_reset_cancel()) {
                if (wifi::cancel_connection()) {
                    cancelled = true;
                    host_out << "Cancelled\n";
                    break;
                }
            }

            host_out << ".";
            if (++dots >= 50) {
                host_out << "\n";
                dots = 0;
            }
            host_out.flush();

            vTaskDelay(pdMS_TO_TICKS(10));
            status = wifi::get_connection_status();
        }

        if (!cancelled) {
            if (result < 0) {
                std::string status_str{};
                host_out << "\nFailed to connect to WiFi network: " << status_str << "\n";
            } else {
                host_out << "\nConnected to WiFi network: " << ssid << "\n";
            }
        }

        host_out.flush();

    } else if (command == "ap") {
        auto ssid_end = params.find(' ');
        if (ssid_end == std::string_view::npos) {
            host_out
                << "Error: Invalid format. Usage: wifi ap <ssid> <password> [channel]\n";
            host_out.flush();
            return;
        }

        const auto ssid = params.substr(0, ssid_end);
        if (ssid.empty()) {
            host_out << "Error: SSID cannot be empty\n";
            host_out.flush();
            return;
        }

        auto password_start = ssid_end + 1;
        if (password_start >= params.size()) {
            host_out << "Error: Missing password. Usage: wifi ap <ssid> <password> "
                        "[channel]\n";
            host_out.flush();
            return;
        }

        auto password_end = params.find(' ', password_start);
        uint8_t channel = 1; // Default channel
        std::string_view password;

        if (password_end == std::string_view::npos) {
            // No channel specified, use all remaining text as password
            password = params.substr(password_start);
        } else {
            // Channel specified, parse password and channel
            password = params.substr(password_start, password_end - password_start);

            auto channel_start = password_end + 1;
            if (channel_start < params.size()) {
                const auto channel_str = params.substr(channel_start);

                int ch = 0;
                auto [ch_ptr, ch_ec] = std::from_chars(
                    channel_str.data(), channel_str.data() + channel_str.size(), ch);

                if (ch_ec != std::errc() || ch < 1 || ch > 13) {
                    host_out << "Error: Channel must be between 1 and 13 - Defaulting to "
                                "channel 1\n";
                } else {
                    channel = static_cast<uint8_t>(ch);
                }
            }
        }

        host_out << "Starting AP with SSID: " << ssid
                 << ", Channel: " << static_cast<int>(channel) << "\n";
        wifi::start_ap(ssid, password, channel);
    } else if (command == "disable") {
        wifi::stop();
        host_out << "WiFi disabled\n";
    } else {
        host_out << "Usage: wifi connect <ssid> <password> | wifi ap <ssid> <password> "
                    "<channel> | wifi disable\n";
    }
    host_out.flush();
}
void handler::cmd_telnet(const std::string_view& arg) {
    if (arg == "enable") {
        auto status = wifi::telnet::enable();
        if (!status) {
            host_out << "Failed to enable telnet: " << status << "\n";
            return;
        } else {
            host_out << "Telnet enabled on port " << sys::settings::get_telnet_port()
                     << "\n";
        }
    } else if (arg == "disable") {
        wifi::telnet::disable();
        host_out << "Telnet disabled\n";
    } else {
        int port = 0;
        auto [ptr, ec] = std::from_chars(arg.data(), arg.data() + arg.size(), port);
        if (ec == std::errc()) {
            if (port > 0 && port <= 65535) {
                auto status = wifi::telnet::set_port(port);
                if (!status) {
                    host_out << "Failed to set telnet port: " << status << "\n";
                    return;
                } else {
                    host_out << "Telnet enabled on port " << port << "\n";
                }
            } else {
                host_out << "Invalid port number. Valid range: 1-65535\n";
            }
        } else {
            host_out << "Telnet is "
                     << (sys::settings::telnet_enabled() ? "Enabled" : "Disabled")
                     << "; Port:" << sys::settings::get_telnet_port() << "\n";
            host_out << "Usage: telnet enable|disable|<port>\n";
        }
    }
}

} // namespace piccante::sys::shell

#endif