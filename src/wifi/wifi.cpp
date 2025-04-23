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
#include "wifi.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <pico/cyw43_arch.h>
#include <cyw43.h>
#include <cyw43_ll.h>
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include <pico/error.h>
#include <projdefs.h>
#include <string>


#include "led/led.hpp"
#include "Logger/Logger.hpp"
#include "SysShell/settings.hpp"
#include "telnet/telnet.hpp"
#include "http/httpd.hpp"
extern "C" {
#include "dhcpd/dhcpserver.h"
}

namespace piccante::wifi {

namespace {

enum class ConnectionState { IDLE, CONNECTING, CONNECTED, FAILED };

ConnectionState connection_state = ConnectionState::IDLE;
int connection_error = 0;
std::string connecting_ssid;
TaskHandle_t connecting_task = nullptr;
bool cancel_requested = false;

TaskHandle_t handle = nullptr;
SemaphoreHandle_t mutex = nullptr;

bool was_connected = false;
bool auto_reconnect = true;

dhcp_server_t dhcp_server;

void wifi_task(void*) {
    vTaskDelay(50);
    const auto err = cyw43_arch_init();
    if (err != PICO_OK) {
        piccante::Log::error << "Failed to initialize cyw43: " << err << "\n";
    } else {
        piccante::Log::debug << "cyw43 initialized successfully\n";
    }

    const auto cfg = sys::settings::get();
    piccante::led::init(cfg.led_mode);

    mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        piccante::Log::error << "Failed to create WiFi mutex\n";
    }

    const auto& wifi_cfg = sys::settings::get_wifi_settings();
    auto mode = static_cast<Mode>(cfg.wifi_mode);

    switch (mode) {
        case Mode::CLIENT: {
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                connection_state = ConnectionState::CONNECTING;
                connecting_ssid = wifi_cfg.ssid;
                connection_error = 0;
                cancel_requested = false;
                xSemaphoreGive(mutex);
            }

            cyw43_arch_enable_sta_mode();
            if (cyw43_arch_wifi_connect_async(wifi_cfg.ssid.c_str(),
                                              wifi_cfg.password.c_str(),
                                              CYW43_AUTH_WPA2_AES_PSK) != 0) {
                Log::error << "Failed to start WiFi reconnection\n";
                connection_state = ConnectionState::IDLE;
            }
            was_connected = true; // enable autoreconn
            break;
        }
        case Mode::ACCESS_POINT: {
            start_ap(wifi_cfg.ssid.c_str(), wifi_cfg.password.c_str(), wifi_cfg.channel);
            break;
        }
        default:
            break;
    }

    httpd::start();

    for (;;) {
        const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());
        if (mode == Mode::CLIENT && mutex != nullptr) {
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (connection_state == ConnectionState::CONNECTING) {
                    if (cancel_requested) {
                        Log::info << "WiFi connection cancelled\n";
                        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
                        connection_state = ConnectionState::IDLE;
                        cancel_requested = false;

                        if (connecting_task != nullptr) {
                            xTaskNotify(connecting_task, CYW43_LINK_FAIL,
                                        eSetValueWithOverwrite);
                            connecting_task = nullptr;
                        }
                    } else {
                        const auto link_status =
                            cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

                        if (link_status >=
                            0 /* == CYW43_LINK_UP */) { // TODO: WTF?! I dunno either,
                                                        // sometimes I get link down
                                                        // on connection ¯\_(ツ)_/¯
                            connection_state = ConnectionState::CONNECTED;
                            Log::info << "WiFi connected to: " << connecting_ssid << "\n";

                            if (connecting_task != nullptr) {
                                xTaskNotify(connecting_task, 0, eNoAction);
                                connecting_task = nullptr;
                            }

                            if (!telnet::is_running() &&
                                sys::settings::telnet_enabled()) {
                                telnet::reconfigure();
                            }

                        } else if (link_status < 0) {
                            connection_state = ConnectionState::FAILED;
                            connection_error = link_status;
                            Log::error
                                << "WiFi connection failed with code: " << link_status
                                << "\n";

                            if (connecting_task != nullptr) {
                                xTaskNotify(connecting_task, link_status,
                                            eSetValueWithOverwrite);
                                connecting_task = nullptr;
                            }
                        }
                    }
                } else if (connection_state == ConnectionState::CONNECTED) {
                    const auto link_status =
                        cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
                    if (link_status != CYW43_LINK_UP) {
                        Log::warning << "WiFi connection lost\n";
                        connection_state = ConnectionState::IDLE;
                        was_connected = true;
                    } else {
                        was_connected = true;
                    }
                } else if (connection_state == ConnectionState::IDLE && was_connected &&
                           auto_reconnect) {
                    Log::info << "Attempting to reconnect to WiFi...\n";

                    const auto& wifi_cfg = sys::settings::get_wifi_settings();
                    connection_state = ConnectionState::CONNECTING;
                    connecting_ssid = wifi_cfg.ssid;
                    connection_error = 0;
                    cancel_requested = false;

                    cyw43_arch_enable_sta_mode();
                    if (cyw43_arch_wifi_connect_async(wifi_cfg.ssid.c_str(),
                                                      wifi_cfg.password.c_str(),
                                                      CYW43_AUTH_WPA2_AES_PSK) != 0) {
                        Log::error << "Failed to start WiFi reconnection\n";
                        connection_state = ConnectionState::IDLE;
                    }
                }
                xSemaphoreGive(mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
} // namespace

TaskHandle_t task() {
    if (handle == nullptr) {
        xTaskCreate(wifi_task, "WiFi", configMINIMAL_STACK_SIZE, nullptr,
                    tskIDLE_PRIORITY + 1, &handle);
    }
    return handle;
}

int connect_to_network(const std::string_view& ssid, const std::string_view& password,
                       uint32_t timeout_ms, TaskHandle_t task_handle) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (connection_state == ConnectionState::CONNECTING &&
            connecting_task != nullptr &&
            connecting_task != xTaskGetCurrentTaskHandle()) {
            cancel_requested = true;

            xSemaphoreGive(mutex);
            vTaskDelay(pdMS_TO_TICKS(500)); // Brief delay for cancellation to complete

            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(800)) != pdTRUE) {
                return -10;
            }
        }
    } else {
        return -10;
    }

    connection_state = ConnectionState::CONNECTING;
    connection_error = 0;
    connecting_ssid = std::string(ssid);
    connecting_task = task_handle ? task_handle : xTaskGetCurrentTaskHandle();
    cancel_requested = false;

    const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());
    if (mode == Mode::ACCESS_POINT) {
        cyw43_arch_disable_ap_mode();
    }
    if (mode == Mode::CLIENT) {
        cyw43_arch_disable_sta_mode();
    }

    std::string ssid_str(ssid);
    std::string password_str(password);

    sys::settings::set_wifi_mode(static_cast<uint8_t>(Mode::CLIENT));
    sys::settings::set_wifi_ssid(ssid_str);
    sys::settings::set_wifi_password(password_str);

    Log::info << "Connecting to WiFi network: " << ssid << "\n";

    cyw43_arch_enable_sta_mode();


    if (cyw43_arch_wifi_connect_async(ssid_str.c_str(), password_str.c_str(),
                                      CYW43_AUTH_WPA2_AES_PSK) != 0) {
        Log::error << "Failed to start WiFi connection\n";
        connection_state = ConnectionState::FAILED;
        connection_error = -1;
        connecting_task = nullptr;
        xSemaphoreGive(mutex);
        return -1;
    }

    xSemaphoreGive(mutex);
    uint32_t notification;
    if (xTaskNotifyWait(0, ULONG_MAX, &notification, pdMS_TO_TICKS(timeout_ms)) ==
        pdTRUE) {
        return notification;
    }

    cancel_connection();
    return CYW43_LINK_FAIL;
}

bool start_ap(const std::string_view& ssid, const std::string_view& password,
              uint8_t channel) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }


    const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());
    if (mode == Mode::CLIENT) {
        cyw43_arch_disable_sta_mode();
    }
    if (mode == Mode::ACCESS_POINT) {
        cyw43_arch_disable_ap_mode();
    }

    std::string ssid_str(ssid);
    std::string password_str(password);

    sys::settings::set_wifi_mode(static_cast<uint8_t>(Mode::ACCESS_POINT));
    sys::settings::set_wifi_ssid(ssid_str);
    sys::settings::set_wifi_password(password_str);
    sys::settings::set_wifi_channel(channel);

    Log::info << "Starting AP mode with SSID: " << ssid
              << ", Channel: " << static_cast<int>(channel) << "\n";


    cyw43_arch_enable_ap_mode(ssid_str.c_str(), password_str.c_str(),
                              CYW43_AUTH_WPA2_AES_PSK);

    ip_addr_t ip = cyw43_state.netif[CYW43_ITF_AP].ip_addr;
    ip_addr_t nm = cyw43_state.netif[CYW43_ITF_AP].netmask;

    dhcp_server_init(&dhcp_server, &ip, &nm);
    Log::info << "DHCP server started with IP: "
              << (ip4_addr_get_u32(ip_2_ip4(&ip)) & 0xff) << "."
              << (ip4_addr_get_u32(ip_2_ip4(&ip)) >> 8 & 0xff) << "."
              << (ip4_addr_get_u32(ip_2_ip4(&ip)) >> 16 & 0xff) << "."
              << (ip4_addr_get_u32(ip_2_ip4(&ip)) >> 24 & 0xff) << "\n";

    xSemaphoreGive(mutex);
    return true;
}

bool is_connected() {
    const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());

    if (mode == Mode::NONE) {
        return false;
    }

    if (mode == Mode::ACCESS_POINT) {
        return true;
    }

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false; // TODO: last know status
    }

    const auto status =
        (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);

    xSemaphoreGive(mutex);
    return status;
}


std::optional<WifiStats> wifi_stats() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return std::nullopt;
    }

    const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());
    if (mode == Mode::NONE) {
        xSemaphoreGive(mutex);
        return std::nullopt;
    }

    WifiStats stats{};
    stats.mode = mode;
    const auto& wifi_cfg = sys::settings::get_wifi_settings();
    stats.ssid = wifi_cfg.ssid;
    stats.channel = wifi_cfg.channel;

    if (mode == Mode::CLIENT) {
        stats.connected =
            (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);
        if (stats.connected) {
            int32_t rssi;
            if (cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
                stats.rssi = rssi;
            } else {
                stats.rssi = 0;
            }

            std::array<char, 16> ip_str{};
            ip4addr_ntoa_r(&cyw43_state.netif[CYW43_ITF_STA].ip_addr, ip_str.data(),
                           ip_str.size());
            stats.ip_address = ip_str.data();
        } else {
            stats.rssi = 0;
            stats.ip_address = "0.0.0.0";
        }
    } else if (mode == Mode::ACCESS_POINT) {
        stats.connected = true;
        stats.rssi = 0; // Not applicable for AP mode

        std::array<char, 16> ip_str{};
        ip4addr_ntoa_r(&cyw43_state.netif[CYW43_ITF_AP].ip_addr, ip_str.data(),
                       ip_str.size());
        stats.ip_address = ip_str.data();
    }

    stats.mac_address = fmt::sprintf(
        "%02x:%02x:%02x:%02x:%02x:%02x", cyw43_state.mac[0], cyw43_state.mac[1],
        cyw43_state.mac[2], cyw43_state.mac[3], cyw43_state.mac[4], cyw43_state.mac[5]);


    xSemaphoreGive(mutex);
    return stats;
}

void stop() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
    const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());

    if (mode == Mode::CLIENT) {
        cyw43_arch_disable_sta_mode();
    } else if (mode == Mode::ACCESS_POINT) {
        dhcp_server_deinit(&dhcp_server);
        Log::info << "DHCP server stopped\n";
        cyw43_arch_disable_ap_mode();
    }

    sys::settings::set_wifi_mode(static_cast<uint8_t>(Mode::NONE));

    Log::info << "WiFi stopped\n";

    xSemaphoreGive(mutex);
}

// Add to wifi.cpp
ConnectionStatus get_connection_status() {
    ConnectionStatus status;

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        status.state = "BUSY";
        return status;
    }

    switch (connection_state) {
        case ConnectionState::IDLE:
            status.state = "IDLE";
            break;
        case ConnectionState::CONNECTING:
            status.state = "CONNECTING";
            status.ssid = connecting_ssid;
            break;
        case ConnectionState::CONNECTED:
            status.state = "CONNECTED";
            status.ssid = connecting_ssid;
            break;
        case ConnectionState::FAILED:
            status.state = "FAILED";
            status.ssid = connecting_ssid;
            status.error_code = connection_error;
            break;
    }

    xSemaphoreGive(mutex);
    return status;
}

bool cancel_connection() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }

    if (connection_state != ConnectionState::CONNECTING) {
        xSemaphoreGive(mutex);
        return false; // Nothing to cancel
    }

    // Set cancel flag - will be processed in wifi_task
    cancel_requested = true;

    xSemaphoreGive(mutex);
    return true;
}

} // namespace piccante::wifi