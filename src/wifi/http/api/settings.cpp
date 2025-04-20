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

namespace piccante::httpd::api::settings {
bool get(http_connection conn, [[maybe_unused]] std::string_view url) {
    const auto handle =
        http_server_begin_write_reply(conn, "200 OK", "application/json", "");
    const auto wifi_cfg = piccante::sys::settings::get_wifi_settings();
    const auto result = fmt::sprintf(
        R"({"echo":%s,"log_level":%d,"led_mode":%d,"wifi_mode":%d,"idle_sleep_minutes":%d,"wifi_settings":{
            "ssid":"%s","password":"%s","channel":%d,"telnet_port":%d,"telnet_enabled":%s
        }})",
        piccante::sys::settings::get_echo() ? "true" : "false",
        piccante::sys::settings::get_log_level(),
        static_cast<uint8_t>(piccante::sys::settings::get_led_mode()),
        piccante::sys::settings::get_wifi_mode(),
        piccante::sys::settings::get_idle_sleep_minutes(), wifi_cfg.ssid.c_str(),
        wifi_cfg.password.c_str(), wifi_cfg.channel,
        piccante::sys::settings::get_telnet_port(),
        piccante::sys::settings::telnet_enabled() ? "true" : "false");
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
        if (auto v = util::json::get_value(*wifi_json, "ssid")) {
            Log::debug << "Setting wifi ssid to: " << *v << "\n";
            piccante::sys::settings::set_wifi_ssid(std::string(*v));
        }
        if (auto v = util::json::get_value(*wifi_json, "password")) {
            Log::debug << "Setting wifi password\n";
            piccante::sys::settings::set_wifi_password(std::string(*v));
        }
        if (auto v = util::json::get_value(*wifi_json, "channel")) {
            Log::debug << "Setting wifi channel to: " << *v << "\n";
            piccante::sys::settings::set_wifi_channel(std::stoi(std::string(*v)));
        }
        if (auto v = util::json::get_value(*wifi_json, "telnet_port")) {
            Log::debug << "Setting telnet port to: " << *v << "\n";
            piccante::sys::settings::set_telnet_port(std::stoi(std::string(*v)));
        }
        if (auto v = util::json::get_value(*wifi_json, "telnet_enabled")) {
            Log::debug << "Setting telnet enabled to: " << *v << "\n";
            piccante::sys::settings::set_telnet_enabled(*v == "true");
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