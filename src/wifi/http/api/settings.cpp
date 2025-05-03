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
#include "settings.hpp"
#include <string_view>
#include <string>
#include <cstdint>
#include <optional>
extern "C" {
#include "../server/httpserver.h"
}
#include "FreeRTOS.h"
#include "task.h"

#include <hardware/watchdog.h>

#include "../../../SysShell/settings.hpp"
#include "../../../fmt.hpp"
#include "Logger/Logger.hpp"
#include "util/json.hpp"
#include "CanBus/CanBus.hpp"
#include "wifi/telnet/telnet.hpp"
#include "wifi/wifi.hpp"
#include "ELM327/elm.hpp"


namespace piccante::httpd::api::settings {
bool get(http_connection conn, [[maybe_unused]] std::string_view url) {
    auto* const handle =
        http_server_begin_write_reply(conn, "200 OK", "application/json", "");

    const int num_buses = can::get_num_busses();
    auto can_settings_json =
        fmt::sprintf(R"("can_settings":{"max_supported":%d,"enabled":%d,)",
                     piccanteNUM_CAN_BUSSES, num_buses);

    for (int i = 0; i < num_buses; i++) {
        const auto enabled = can::is_enabled(i);
        const auto bitrate = can::get_bitrate(i);
        const auto listen_only = can::is_listenonly(i);

        can_settings_json +=
            fmt::sprintf(R"("can%d":{"enabled":%s,"listen_only":%s,"bitrate":%d})",
                         i,
                         enabled ? "true" : "false",
                         listen_only ? "true" : "false",
                         bitrate);

        if (i < num_buses - 1) {
            can_settings_json += ",";
        }
    }
    can_settings_json += "}";

    const auto elm_settings_json =
        fmt::sprintf(R"("elm_settings":{"interface":%d,"bus":%d,"bt_pin":%d})",
                     piccante::sys::settings::get_wifi_settings().elm_interface,
                     piccante::sys::settings::get().elm_can_bus,
                     piccante::sys::settings::get_wifi_settings().bluetooth_pin);


    const auto wifi_cfg = piccante::sys::settings::get_wifi_settings();
    const auto result = fmt::sprintf(
        R"({"echo":%s,"log_level":%d,"led_mode":%d,"wifi_mode":%d,"idle_sleep_minutes":%d,"wifi_settings":{
            "ssid":"%s","password":"%s","channel":%d,"telnet_port":%d,"telnet_enabled":%s
        },%s,%s})",
        piccante::sys::settings::get_echo() ? "true" : "false",
        piccante::sys::settings::get_log_level(),
        static_cast<uint8_t>(piccante::sys::settings::get_led_mode()),
        piccante::sys::settings::get_wifi_mode(),
        piccante::sys::settings::get_idle_sleep_minutes(), wifi_cfg.ssid.c_str(),
        wifi_cfg.password.c_str(), wifi_cfg.channel,
        piccante::sys::settings::get_telnet_port(),
        piccante::sys::settings::telnet_enabled() ? "true" : "false",
        can_settings_json.c_str(), elm_settings_json.c_str());
    http_server_write_raw(handle, result.c_str(), result.size());
    http_server_end_write_reply(handle, "");
    return true;
}

bool set(http_connection conn, [[maybe_unused]] std::string_view url) {
    std::string body{};
    char* body_line = nullptr;
    while ((body_line = http_server_read_post_line(conn)) != nullptr) {
        if (body_line[0] != '\0') {
            body += body_line;
        }
    }

    if (body.empty()) {
        http_server_send_reply(conn, "400 Bad Request", "application/json",
                               R"({"error":"Empty body"})", -1);
        return true;
    }
    std::string_view json(body);

    if (auto v = util::json::get_value(json, "echo")) {
        Log::debug << "Setting echo to: " << *v << "\n";
        piccante::sys::settings::set_echo(*v == "true");
    }
    if (auto v = util::json::get_value(json, "log_level")) {
        Log::debug << "Setting log level to: " << *v << "\n";
        piccante::sys::settings::set_log_level(std::stoi(std::string(*v)));
    }
    if (auto v = util::json::get_value(json, "led_mode")) {
        Log::debug << "Setting led mode to: " << *v << "\n";
        piccante::sys::settings::set_led_mode(
            static_cast<piccante::led::Mode>(std::stoi(std::string(*v))));
    }
    if (auto v = util::json::get_value(json, "wifi_mode")) {
        Log::debug << "Setting wifi mode to: " << *v << "\n";
        piccante::sys::settings::set_wifi_mode(std::stoi(std::string(*v)));
    }
    if (auto v = util::json::get_value(json, "idle_sleep_minutes")) {
        Log::debug << "Setting idle sleep minutes to: " << *v << "\n";
        piccante::sys::settings::set_idle_sleep_minutes(std::stoi(std::string(*v)));
    }

    // Handle nested wifi_settings
    if (auto wifi_json = piccante::util::json::get_object(json, "wifi_settings")) {
        bool wifi_settings_changed = false;
        if (auto v = util::json::get_value(*wifi_json, "ssid")) {
            Log::debug << "Setting wifi ssid to: " << *v << "\n";
            piccante::sys::settings::set_wifi_ssid(std::string(*v));
            wifi_settings_changed = true;
        }
        if (auto v = util::json::get_value(*wifi_json, "password")) {
            Log::debug << "Setting wifi password\n";
            piccante::sys::settings::set_wifi_password(std::string(*v));
            wifi_settings_changed = true;
        }
        if (auto v = util::json::get_value(*wifi_json, "channel")) {
            Log::debug << "Setting wifi channel to: " << *v << "\n";
            piccante::sys::settings::set_wifi_channel(std::stoi(std::string(*v)));
            wifi_settings_changed = true;
        }

        if (wifi_settings_changed) {
            const auto wifi_mode =
                static_cast<wifi::Mode>(piccante::sys::settings::get_wifi_mode());
            if (wifi_mode == wifi::Mode::NONE) {
                Log::debug << "Disabling WiFI\n";
                wifi::stop();
            } else if (wifi_mode == wifi::Mode::CLIENT) {
                wifi::connect_to_network(
                    piccante::sys::settings::get_wifi_settings().ssid,
                    piccante::sys::settings::get_wifi_settings().password, 30000,
                    xTaskGetCurrentTaskHandle());
            } else if (wifi_mode == wifi::Mode::ACCESS_POINT) {
                wifi::start_ap(piccante::sys::settings::get_wifi_settings().ssid,
                               piccante::sys::settings::get_wifi_settings().password,
                               piccante::sys::settings::get_wifi_settings().channel);
            }
        }

        if (auto v = util::json::get_value(*wifi_json, "telnet_port")) {
            auto port = std::stoi(std::string(*v));
            Log::debug << "Setting telnet port to: " << port << "\n";
            auto status = wifi::telnet::set_port(port);
            if (!status) {
                // TODO: Handle error
                Log::error << "Failed to set telnet port\n";
            }
        }
        if (auto v = util::json::get_value(*wifi_json, "telnet_enabled")) {
            Log::debug << "Setting telnet enabled to: " << *v << "\n";
            if (*v == "true") {
                wifi::telnet::enable();
            } else {
                wifi::telnet::disable();
            }
        }
    }

    if (auto can_settings_json = util::json::get_object(json, "can_settings")) {
        if (auto v = util::json::get_value(*can_settings_json, "enabled")) {
            char* end;
            long num = strtol(v->data(), &end, 10);
            if (end != v->data() && end == v->data() + v->size() && num >= 0 &&
                num <= piccanteNUM_CAN_BUSSES) {
                Log::debug << "Setting number of available can interfaces to: " << *v
                           << "\n";
                can::set_num_busses(static_cast<int>(num));
            }
        }

        for (int i = 0; i < can::get_num_busses(); i++) {
            if (auto v = util::json::get_object(*can_settings_json,
                                                fmt::sprintf("can%d", i))) {
                Log::debug << "Setting can bus " << i << " settings to: " << *v << "\n";
                auto enabled = util::json::get_value(*v, "enabled");
                auto bitrate = util::json::get_value(*v, "bitrate");
                auto listen_only = util::json::get_value(*v, "listen_only");
                if (enabled.has_value() && *enabled != "") {
                    if (enabled == "true") {
                        const auto stored_bitrate = can::get_bitrate(i);
                        if (!bitrate.has_value() || *bitrate == "") {
                            Log::debug << "Enabling canbus " << i
                                       << " with stored bitrate: " << stored_bitrate
                                       << "\n";
                            can::enable(i, stored_bitrate);
                        } else {
                            Log::debug << "Enabling canbus " << i
                                       << " with new bitrate: " << *bitrate << "\n";
                            can::enable(i, std::stoi(std::string(*bitrate)));
                        }
                    } else {
                        Log::debug << "Disabling canbus " << i << "\n";
                        can::disable(i);
                    }
                }
                if (!enabled.has_value() && bitrate.has_value() && *bitrate != "") {
                    Log::debug << "Setting " << i << " bitrate to: " << *bitrate << "\n";
                    can::set_bitrate(i, std::stoi(std::string(*bitrate)));
                }
                if (listen_only.has_value() && *listen_only != "") {
                    Log::debug << "Setting " << i << " listen only to: " << *listen_only
                               << "\n";
                    can::set_listenonly(i, *listen_only == "true");
                }
            }
        }
    }

    if (auto elm_settings_json = util::json::get_object(json, "elm_settings")) {
        if (auto v = util::json::get_value(*elm_settings_json, "bus")) {
            Log::debug << "Setting ELM327 bus to: " << *v << "\n";
            const auto& wifi_cfg = piccante::sys::settings::get_wifi_settings();
            piccante::elm327::stop();
            piccante::sys::settings::set_elm_can_bus(std::stoi(std::string(*v)));
            if (wifi_cfg.elm_interface !=
                static_cast<uint8_t>(piccante::elm327::interface::USB)) {
                piccante::elm327::start();
            }
        }
        if (auto v = util::json::get_value(*elm_settings_json, "interface")) {
            Log::debug << "Setting ELM327 interface to: " << *v << "\n";
            const auto& wifi_cfg = piccante::sys::settings::get_wifi_settings();
            piccante::elm327::stop();
            piccante::sys::settings::set_elm_interface(std::stoi(std::string(*v)));
            if (wifi_cfg.elm_interface !=
                static_cast<uint8_t>(piccante::elm327::interface::USB)) {
                piccante::elm327::start();
            }
        }
        if (auto v = util::json::get_value(*elm_settings_json, "bt_pin")) {
            Log::debug << "Setting ELM327 bluetooth pin to: " << *v << "\n";
            const auto& wifi_cfg = piccante::sys::settings::get_wifi_settings();
            piccante::elm327::stop();
            piccante::sys::settings::set_bluetooth_pin(std::stoi(std::string(*v)));
            if (wifi_cfg.elm_interface !=
                static_cast<uint8_t>(piccante::elm327::interface::USB)) {
                piccante::elm327::start();
            }
        }
    }

    http_server_send_reply(conn, "200 OK", "application/json", R"({"status":"ok"})", -1);

    return true;
}

bool save(http_connection conn, [[maybe_unused]] std::string_view url) {
    std::string body{};
    char* body_line = nullptr;
    while ((body_line = http_server_read_post_line(conn)) != nullptr) {
        if (body_line[0] != '\0') {
            body += body_line;
        }
    }

    bool should_reset = false;

    if (!body.empty()) {
        std::string_view json(body);

        if (auto v = util::json::get_value(json, "reset")) {
            should_reset = (*v == "true");
            Log::debug << "Reset requested: " << should_reset << "\n";
        }
    }

    const auto success = sys::settings::store();

    if (success) {
        http_server_send_reply(conn, "200 OK", "application/json",
                               R"({"status":"settings saved"})", -1);

        if (should_reset) {
            vTaskDelay(pdMS_TO_TICKS(100));
            Log::info << "Reset device requested\n";
            watchdog_enable(1, 1);
            while (1) { /* Wait for watchdog to trigger */
            }
        }
    } else {
        http_server_send_reply(conn, "500 Internal Server Error", "application/json",
                               R"({"error":"Failed to save settings"})", -1);
    }

    return true;
}

} // namespace piccante::httpd::api::settings