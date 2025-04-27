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
#include <cstring>
#include <bsp/board_api.h>

extern "C" {
#include "usb/usb_descriptors.h"
}

#include "Logger/Logger.hpp"
#include "CanBus/CanBus.hpp"
#include "FreeRTOSConfig.h"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"
#include "queue.h"
#include "task.h"

#include "timers.h"
#include "tusb.h"
#include <portable.h>
#include <projdefs.h>

#include "outstream/usb_cdc_stream.hpp"
#include "outstream/uart_stream.hpp"

#include "fs/littlefs_driver.hpp"
#include "CommProto/slcan/slcan.hpp"
#include "CommProto/gvret/handler.hpp"
#include "SysShell/handler.hpp"
#include "SysShell/settings.hpp"
#include "stats/stats.hpp"
#include "led/led.hpp"
#include "power/sleep.hpp"
#ifdef WIFI_ENABLED
#include "wifi/wifi.hpp"
#include "wifi/telnet/telnet.hpp"
#include "wifi/bt_spp/bt_spp.hpp"
#endif
// TODO: remove
#include "wifi/telnet/telnet_server.hpp"
#include "ELM327/emulator.hpp"
#include "fmt.hpp"
#include <pico/time.h>


static void usbDeviceTask(void* parameters) {
    (void)parameters;

    vTaskDelay(108);
    tud_init(0);

    TickType_t wake = 0;
    wake = xTaskGetTickCount();

    for (;;) {
        tud_task();
        if (tud_suspended() || !tud_connected()) {
            xTaskDelayUntil(&wake, 10);
        } else if (!tud_task_event_ready()) {
            xTaskDelayUntil(&wake, 1);
        }
    }
}


std::unique_ptr<piccante::elm327::emulator> elmulator = nullptr;

static std::array<std::unique_ptr<piccante::slcan::handler>, piccanteNUM_CAN_BUSSES>
    slcan_handler = {nullptr};
static std::unique_ptr<piccante::gvret::handler> gvret_handler = nullptr;
static void can_recieveTask(void* parameter) {
    const auto bus = reinterpret_cast<int>(parameter);
    // Wait until can is up.
    vTaskDelay(7000 + 100 * bus); // TODO
    const auto num_busses = piccante::can::get_num_busses();

    if (bus >= num_busses) {
        return;
    }
    piccante::Log::info << "Starting CAN Receive (Bus" << bus << ") Task!\n";

    can2040_msg msg{};
    for (;;) {
        auto received = false;
        if (piccante::can::receive(bus, msg) >= 0) {
            if (bus == 0) {
                elmulator->handle_can_frame(msg);
            }

            gvret_handler->comm_can_frame(bus, msg);
            slcan_handler[bus]->comm_can_frame(msg);
            received = true;
        }
        if (received) {
            piccante::led::blink();
            piccante::power::sleep::reset_idle_timer();
        } else {
            taskYIELD();
        }
    }
}


std::unique_ptr<piccante::wifi::telnet::server> elm_telnet_server = nullptr;

class elm_sink : public piccante::out::base_sink {
        public:
    void write(const char* data, std::size_t len) override {
        if (!elm_telnet_server || !elm_telnet_server->is_running()) {
            return;
        }

        if (xSemaphoreTake(elm_telnet_server->get_clients_mutex(), pdMS_TO_TICKS(100)) !=
            pdTRUE) {
            return;
        }

        for (const auto& client : elm_telnet_server->get_clients()) {
            if (client.sink) {
                client.sink->write(data, len);
            }
        }

        xSemaphoreGive(elm_telnet_server->get_clients_mutex());
    }

    void flush() override {
        if (!elm_telnet_server || !elm_telnet_server->is_running()) {
            return;
        }

        if (xSemaphoreTake(elm_telnet_server->get_clients_mutex(), pdMS_TO_TICKS(100)) !=
            pdTRUE) {
            return;
        }

        for (const auto& client : elm_telnet_server->get_clients()) {
            if (client.sink) {
                client.sink->flush();
            }
        }

        xSemaphoreGive(elm_telnet_server->get_clients_mutex());
    }
};

static void cmd_gvret_task(void* parameter) {
    (void)parameter;
    vTaskDelay(6000);
    piccante::Log::info << ("Starting PiCCANTE CMD + GVRET Task!\n");

#ifdef WIFI_ENABLED
    auto sink = piccante::out::sink_mux{};
    sink.add_sink(&piccante::wifi::telnet::get_sink());
    // sink.add_sink(&piccante::bluetooth::get_sink());
    sink.add_sink(piccante::usb_cdc::sinks[0].get());
    auto outstream = piccante::out::stream{sink};
#else
    const auto sink = piccante::usb_cdc::sinks[0].get();
    auto outstream = piccante::out::stream{*sink};
#endif

    gvret_handler = std::make_unique<piccante::gvret::handler>(outstream);
    piccante::sys::shell::handler shell_handler(*gvret_handler.get(), outstream);

    // elm_telnet_server =
    //     std::make_unique<piccante::wifi::telnet::server>("ELM327 Telnet", 35000, ">");
    piccante::out::stream elm_stream =
        piccante::out::stream{piccante::bluetooth::get_sink()};
    const auto elm_queue = piccante::bluetooth::get_rx_queue();

    elmulator = std::make_unique<piccante::elm327::emulator>(elm_stream, elm_queue);
    elm_telnet_server->start();
    elmulator->start();

    for (;;) {
        auto received = false;
        while (tud_cdc_n_available(0) > 0) {
            received = true;
            const auto byte = tud_cdc_n_read_char(0);
            const auto used = gvret_handler->process_byte(byte);
            if (!used) {
                shell_handler.process_byte(byte);
            }
        }

#ifdef WIFI_ENABLED
        const auto telnet_queue = piccante::wifi::telnet::get_rx_queue();
        if (telnet_queue) {
            uint8_t byte;
            while (xQueueReceive(telnet_queue, &byte, 0) == pdTRUE) {
                received = true;
                const auto used = gvret_handler->process_byte(byte);
                if (!used) {
                    shell_handler.process_byte(byte);
                }
            }
        }
#endif
        if (received) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}


int main() {
    piccante::uart::sink0.init(0, 1, piccanteUART_SPEED);

#ifdef DEBUG
    piccante::Log::set_log_level(piccante::Log::Level::LEVEL_DEBUG);
#endif

    // littleFS
    if (piccante::fs::init()) {
        piccante::Log::debug << "LittleFS mounted successfully\n";
    } else {
        piccante::Log::error << "LittleFS mount failed\n";
    }

    const auto& cfg = piccante::sys::settings::get();
    piccante::can::load_settings();
    num_extra_cdc = piccante::can::get_num_busses();

    // Initialize TinyUSB stack
    board_init();
    tusb_init();


    board_init_after_tusb();


#ifndef DEBUG
    piccante::Log::set_log_level(static_cast<piccante::Log::Level>(cfg.log_level));
#else
    piccante::sys::settings::set_log_level(piccante::Log::Level::LEVEL_DEBUG);
#endif
    piccante::sys::stats::init_stats_collection();

#ifndef WIFI_ENABLED
    piccante::led::init(cfg.led_mode);
#endif

    piccante::power::sleep::init();


    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t piccanteAndGvretTaskHandle;


    xTaskCreate(usbDeviceTask, "USB", configMINIMAL_STACK_SIZE, nullptr,
                configMAX_PRIORITIES - 6, &usbTaskHandle);
    xTaskCreate(cmd_gvret_task, "PiCCANTE+GVRET", configMINIMAL_STACK_SIZE, nullptr, 3,
                &piccanteAndGvretTaskHandle);

    for (size_t i = 0; i < piccanteNUM_CAN_BUSSES; i++) {
        slcan_handler[i] = std::make_unique<piccante::slcan::handler>(
            piccante::usb_cdc::out(i + 1), i + 1, i);
        auto slcanTaskHandle = slcan_handler[i]->create_task();

        TaskHandle_t canRxHandle;
        xTaskCreate(can_recieveTask, fmt::sprintf("CAN RX%d", i).c_str(),
                    configMINIMAL_STACK_SIZE, reinterpret_cast<void*>(i),
                    configMAX_PRIORITIES - 10, &canRxHandle);
    }

    static TaskHandle_t canTaskHandle = piccante::can::create_task();

    vTaskCoreAffinitySet(canTaskHandle, 0x02);

#ifdef WIFI_ENABLED
    static TaskHandle_t wifiTaskHandle = piccante::wifi::task();

    static TaskHandle_t btSppTaskHandle = piccante::bluetooth::create_task();

#endif

    vTaskStartScheduler();

    return 0;
}

// NOLINTNEXTLINE
void vApplicationMallocFailedHook() { configASSERT((volatile void*)nullptr); }
// NOLINTNEXTLINE
void vApplicationIdleHook() {
    volatile size_t xFreeHeapSpace = 0;

    xFreeHeapSpace = xPortGetFreeHeapSize();

    (void)xFreeHeapSpace;
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    /* Check pcTaskName for the name of the offending task,
     * or pxCurrentTCB if pcTaskName has itself been corrupted. */
    (void)xTask;
    (void)pcTaskName;
}

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */
