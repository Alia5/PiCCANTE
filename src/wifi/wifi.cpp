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

#include <cstdint>
#include <pico/cyw43_arch.h>
#include <cyw43.h>
#include <cyw43_ll.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string>
#include <string_view>


#include "led/led.hpp"
#include "Logger/Logger.hpp"
#include "SysShell/settings.hpp"


namespace piccante::wifi {

namespace {
TaskHandle_t handle = nullptr;
SemaphoreHandle_t mutex = nullptr;

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
    const auto mode = static_cast<Mode>(cfg.wifi_mode);
    auto success = false;

    while (!success && mode != Mode::NONE) {
        switch (mode) {
            case Mode::CLIENT: {
                const auto& result = connect_to_network(wifi_cfg.ssid.c_str(),
                                                        wifi_cfg.password.c_str(), 10000);
                if (result >= 0) {
                    success = true;
                }
                break;
            }
            case Mode::ACCESS_POINT: {
                success = start_ap(wifi_cfg.ssid.c_str(), wifi_cfg.password.c_str(),
                                   wifi_cfg.channel);
                break;
            }
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    bool is_connected_flag = false;
    for (;;) {
        const auto mode = static_cast<Mode>(sys::settings::get_wifi_mode());
        if (mode == Mode::CLIENT && mutex != nullptr) {
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                const auto was_connected = is_connected_flag;
                is_connected_flag = (cyw43_tcpip_link_status(
                                         &cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);

                if (was_connected && !is_connected_flag) {
                    piccante::Log::info
                        << "WiFi connection lost, attempting to reconnect\n";
                    if (wifi_cfg.ssid != "") {
                        cyw43_arch_wifi_connect_timeout_ms(
                            wifi_cfg.ssid.c_str(),
                            wifi_cfg.password == "" ? nullptr : wifi_cfg.password.c_str(),
                            CYW43_AUTH_WPA2_AES_PSK,
                            10000);
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
                       uint32_t timeout_ms) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return -10;
    }

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


    const auto result = cyw43_arch_wifi_connect_timeout_ms(
        ssid_str.c_str(), password_str.c_str(), CYW43_AUTH_WPA2_AES_PSK, timeout_ms);

    if (result < 0) {
        Log::error << "Failed to connect to WiFi network: " << result << "\n";
        xSemaphoreGive(mutex);
        return result;
    }
    Log::info << "Connected to WiFi network: " << ssid << "\n";

    xSemaphoreGive(mutex);
    return result;
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
        cyw43_arch_disable_ap_mode();
    }

    sys::settings::set_wifi_mode(static_cast<uint8_t>(Mode::NONE));

    Log::info << "WiFi stopped\n";

    xSemaphoreGive(mutex);
}

} // namespace piccante::wifi