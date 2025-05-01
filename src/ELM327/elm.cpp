
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
#include "elm.hpp"

#include <memory>
#include <cstdint>
#include "emulator.hpp"
#include "SysShell/settings.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "outstream/usb_cdc_stream.hpp"
#ifdef WIFI_ENABLED
#include "wifi/telnet/telnet_server.hpp"
#include "wifi/bt_spp/bt_spp.hpp"
#endif

namespace piccante::elm327 {

namespace {
std::unique_ptr<elm327::emulator> elm_emu = nullptr;
out::sink_mux elm_sink;
out::stream elm_stream{elm_sink};
QueueHandle_t elm_queue = nullptr;

#ifdef WIFI_ENABLED
static constexpr uint16_t ELM_TELNET_PORT = 35000;
TaskHandle_t bt_spp_task_handle = nullptr;
std::unique_ptr<wifi::telnet::server> elm_telnet_server = nullptr;

#endif

} // namespace

void stop();

void start() {
    if (elm_emu) {
        return;
    }
    const auto& cfg = sys::settings::get();

#ifdef WIFI_ENABLED
    const auto& wifi_cfg = sys::settings::get_wifi_settings();
    if (wifi_cfg.elm_interface == static_cast<uint8_t>(interface::USB)) {
        elm_sink.add_sink(usb_cdc::sinks[0].get());
        elm_queue = xQueueCreate(32, sizeof(uint8_t));
    } else if (wifi_cfg.elm_interface == static_cast<uint8_t>(interface::Bluetooth)) {
        bt_spp_task_handle = bluetooth::create_task();
        if (bt_spp_task_handle == nullptr) {
            Log::error << "Failed to create Bluetooth SPP task\n";
            stop();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        elm_sink.add_sink(&bluetooth::get_sink());
        elm_queue = bluetooth::get_rx_queue();
    } else if (wifi_cfg.elm_interface == static_cast<uint8_t>(interface::WiFi)) {
        if (elm_telnet_server) {
            elm_telnet_server->stop();
            elm_telnet_server = nullptr;
        }
        elm_telnet_server =
            std::make_unique<wifi::telnet::server>("ELM327 Telnet", 35000, ">");
        if (elm_telnet_server == nullptr) {
            Log::error << "Failed to create ELM327 Telnet server\n";
            stop();
            return;
        }
        elm_sink.add_sink(&elm_telnet_server->get_all_sink());
        elm_queue = elm_telnet_server->get_rx_queue();
        elm_telnet_server->start();
    }
#else
    elm_sink.add_sink(usb_cdc::sinks[0].get());
    elm_queue = xQueueCreate(32, sizeof(uint8_t));
#endif


    elm_emu = std::make_unique<elm327::emulator>(elm_stream, elm_queue, cfg.elm_can_bus);
    if (!elm_emu->start()) {
        Log::error << "ELM327: Failed to start emulator\n";
        elm_emu = nullptr;
        elm_queue = nullptr;
        vQueueDelete(elm_queue);
        return;
    }
};
void stop() {
#ifdef WIFI_ENABLED
    const auto wifi_cfg = sys::settings::get_wifi_settings();
#endif
    if (elm_emu) {
        elm_emu->stop();
    }
    elm_emu = nullptr;
    vTaskDelay(pdMS_TO_TICKS(100));
    elm_sink.clear_sinks();
#ifdef WIFI_ENABLED
    bluetooth::stop();
    bt_spp_task_handle = nullptr;
    if (elm_telnet_server) {
        elm_telnet_server->stop();
        elm_telnet_server = nullptr;
        Log::info << "Telnet server stopped\n";
    }
    if (wifi_cfg.elm_interface == static_cast<uint8_t>(interface::USB)) {
        if (elm_queue) {
            vQueueDelete(elm_queue);
        }
    }
#else
    if (elm_queue) {
        vQueueDelete(elm_queue);
    }
#endif
    elm_queue = nullptr;
}

void reconfigure() {
    stop();
    start();
}

elm327::emulator* emu() { return elm_emu.get(); }
QueueHandle_t queue() { return elm_queue; }


} // namespace piccante::elm327