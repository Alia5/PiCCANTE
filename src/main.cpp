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
#include "CanBus/mitm_bridge/bridge.hpp"
#include "class/cdc/cdc_device.h"
#include "device/usbd.h"

#include "FreeRTOS.h"
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
#include "ELM327/elm.hpp"
#include "ELM327/emulator.hpp"
#include "fmt.hpp"
#include <pico/time.h>

#include "sd_card/sd_card.hpp"

#ifdef WIFI_ENABLED
#include "wifi/wifi.hpp"
#include "wifi/telnet/telnet.hpp"
#include "wifi/telnet/buffered_sink.hpp"
#include "wifi/bt_spp/bt_spp.hpp"
#endif


static std::array<std::unique_ptr<piccante::slcan::handler>, piccanteNUM_CAN_BUSSES>
    slcan_handler = {};
static std::unique_ptr<piccante::gvret::handler> gvret_handler = nullptr;
static TaskHandle_t piccanteAndGvretTaskHandle;


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


static void can_recieveTask(void* parameter) {
    vTaskDelay(1200);
    const auto num_busses = piccante::can::get_num_busses();


    piccante::Log::info << "Starting CAN Receive Task!\n";

    const auto& cfg = piccante::sys::settings::get();
    piccante::can::frame msg{};
    for (;;) {
        auto received = false;
        ulTaskNotifyTake(pdTRUE, 0);
        for (int bus = 0; bus < num_busses; bus++) {
            if (piccante::can::receive(bus, msg, 0) >= 0) {
                const auto bridged = piccante::can::mitm::bridge::handle(bus, msg);
                if (bridged) {
                    taskYIELD();
                }

                if (bus == cfg.elm_can_bus && piccante::elm327::emu() != nullptr) {
                    piccante::elm327::emu()->handle_can_frame(msg);
                }

                gvret_handler->comm_can_frame(bus, msg);
                slcan_handler[bus]->comm_can_frame(msg);
                received = true;
            }
        }
        if (received) {
            piccante::led::blink();
            piccante::power::sleep::reset_idle_timer();
            xTaskNotifyGive(piccanteAndGvretTaskHandle);
        } else {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        }
    }
}


static void main_interface_task(void* parameter) {
    (void)parameter;
    vTaskDelay(1000);
    piccante::Log::info << ("Starting Main Interface + GVRET Task!\n");

#ifdef WIFI_ENABLED
    auto sink = piccante::out::sink_mux{};
    auto buffered_telnet_sink =
        piccante::out::timeout_buffer_sink{piccante::wifi::telnet::get_sink()};
    sink.add_sink(&buffered_telnet_sink);
    sink.add_sink(piccante::usb_cdc::sinks[0].get());
    auto outstream = piccante::out::stream{sink};
#else
    const auto sink = piccante::usb_cdc::sinks[0].get();
    auto outstream = piccante::out::stream{*sink};
#endif

    gvret_handler = std::make_unique<piccante::gvret::handler>(outstream);
    piccante::sys::shell::handler shell_handler(*gvret_handler.get(), outstream);

    const auto& cfg = piccante::sys::settings::get();
#ifdef WIFI_ENABLED
    const auto& wifi_cfg = piccante::sys::settings::get_wifi_settings();
#endif

    for (;;) {
        ulTaskNotifyTake(pdTRUE, 0); // clear notifications
        auto received = false;
        while (tud_cdc_n_available(0) > 0) {
            received = true;
            const auto byte = tud_cdc_n_read_char(0);
            auto used = gvret_handler->process_byte(byte);
#ifdef WIFI_ENABLED
            if (wifi_cfg.elm_interface ==
                static_cast<uint8_t>(piccante::elm327::interface::USB)) {
                if (piccante::elm327::emu() != nullptr) {
                    used = true;
                    xQueueSendToBack(piccante::elm327::queue(), &byte, 0);
                }
            }
#else
            if (piccante::elm327::emu() != nullptr) {
                used = true;
                xQueueSendToBack(piccante::elm327::queue(), &byte, 0);
            }
#endif
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
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
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


    xTaskCreate(usbDeviceTask, "USB", configMINIMAL_STACK_SIZE / 2, nullptr,
                configMAX_PRIORITIES - 6, &usbTaskHandle);
    xTaskCreate(main_interface_task, "PiCCANTE+GVRET", configMINIMAL_STACK_SIZE * 2,
                nullptr, 3, &piccanteAndGvretTaskHandle);

    for (size_t i = 0; i < piccanteNUM_CAN_BUSSES; i++) {
        slcan_handler[i] = std::make_unique<piccante::slcan::handler>(
            piccante::usb_cdc::out(i + 1), i + 1, i);
        auto slcanTaskHandle = slcan_handler[i]->create_task();
    }
    static TaskHandle_t canRxHandle;
    xTaskCreate(can_recieveTask, "CAN RX", configMINIMAL_STACK_SIZE * 2, nullptr,
                configMAX_PRIORITIES - 10, &canRxHandle);
    piccante::can::set_rx_task_handle(canRxHandle);
    static TaskHandle_t canTaskHandle = piccante::can::create_task();

    vTaskCoreAffinitySet(canTaskHandle, 0x02);

#ifdef WIFI_ENABLED
    static TaskHandle_t wifiTaskHandle = piccante::wifi::task();
    const auto& wifi_cfg = piccante::sys::settings::get_wifi_settings();
    if (wifi_cfg.elm_interface !=
        static_cast<uint8_t>(piccante::elm327::interface::USB)) {
        static TaskHandle_t elm_task_handle = nullptr;
        xTaskCreate(
            [](void* param) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                piccante::elm327::start();
                vTaskDelete(nullptr);
            },
            "ELM_Init",
            configMINIMAL_STACK_SIZE,
            nullptr,
            3,
            &elm_task_handle);
    }
#endif

    vTaskStartScheduler();

    return 0;
}

// NOLINTNEXTLINE
void vApplicationMallocFailedHook() {
    //
    piccante::Log::error << "Malloc failed!\n";
    configASSERT((volatile void*)nullptr);
}
// NOLINTNEXTLINE
void vApplicationIdleHook() {
    volatile size_t xFreeHeapSpace = 0;

    xFreeHeapSpace = xPortGetFreeHeapSize();

    (void)xFreeHeapSpace;
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    piccante::Log::error << "STACK OVERFLOW: Task " << pcTaskName
                         << " overflowed its stack!\n";
}

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */


