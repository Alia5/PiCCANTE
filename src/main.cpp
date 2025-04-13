
// NOLINTNEXTLINE
#include <array>
#include <cstdint>
#include <cstring>
#include <bsp/board_api.h>

#include "Logger/Logger.hpp"
#include "CanBus/CanBus.hpp"
#include "FreeRTOSConfig.h"
#include "device/usbd.h"
#include "queue.h"
#include "task.h"

#include "timers.h"
#include "tusb.h"
#include <portable.h>

#include "outstream/usb_cdc_stream.hpp"
#include "outstream/uart_stream.hpp"

#include "fs/littlefs_driver.hpp"
#include "CommProto/slcan/slcan.hpp"
#include "CommProto/gvret/gvret.hpp"
#include "CommProto/gvret/handler.hpp"
#include "SysShell/handler.hpp"
#include "SysShell/settings.hpp"
#include "stats/stats.hpp"
#include "led/led.hpp"
#ifdef WIFI_ENABLED
#include <pico/cyw43_arch.h>
#endif

static void usbDeviceTask(void* parameters) {
    (void)parameters;

    vTaskDelay(108);
    tud_init(0);

    TickType_t wake = 0;
    wake = xTaskGetTickCount();

    for (;;) {
        tud_task();
        if (tud_suspended() || !tud_connected()) {
            xTaskDelayUntil(&wake, piccanteIDLE_SLEEP_MS);
        } else if (!tud_task_event_ready()) {
            xTaskDelayUntil(&wake, 1);
        }
    }
}

static std::array<std::unique_ptr<piccante::slcan::handler>, piccanteNUM_CAN_BUSSES>
    slcan_handler = {nullptr};
static std::unique_ptr<piccante::gvret::handler> gvret_handler = nullptr;
static void can_recieveTask(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(2000); // TODO

    piccante::Log::info << ("Starting CAN Receive Task!\n");

    can2040_msg msg{};
    for (;;) {
        auto received = false;
        for (uint8_t bus = 0; bus < piccanteNUM_CAN_BUSSES; bus++) {
            if (piccante::can::receive(bus, msg) >= 0) {
                piccante::led::toggle();
                gvret_handler->comm_can_frame(bus, msg);
                auto handler = slcan_handler[bus].get();
                if (handler != nullptr) {
                    handler->comm_can_frame(msg);
                }
                received = true;
            }
        }
        if (received) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(piccanteIDLE_SLEEP_MS));
        }
    }
}


static void cmd_gvret_task(void* parameter) {
    (void)parameter;
    vTaskDelay(60);
    piccante::Log::info << ("Starting PiCCANTE CMD + GVRET Task!\n");

    gvret_handler = std::make_unique<piccante::gvret::handler>(piccante::usb_cdc::out(0));
    piccante::sys::shell::handler shell_handler(*gvret_handler.get(),
                                                piccante::usb_cdc::out(0));

    for (;;) {
        auto received = false;
        while (tud_cdc_n_available(0) > 0) {
            received = true;
            const auto c = tud_cdc_n_read_char(0);
            const auto used = gvret_handler->process_byte(c);
            if (!used) {
                shell_handler.process_byte(c);
            }
        }
        if (received) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(piccanteIDLE_SLEEP_MS));
        }
    }
}

#ifdef WIFI_ENABLED
// TODO: proper
static void wifi_task(void* params) {
    vTaskDelay(50);
    const auto err = cyw43_arch_init();
    if (err != PICO_OK) {
        piccante::Log::error << "Failed to initialize cyw43: " << err << "\n";
    } else {
        piccante::Log::debug << "cyw43 initialized successfully\n";
    }

    const auto cfg = static_cast<piccante::sys::settings::system_settings*>(params);
    piccante::led::init(cfg->led_mode);

    for (;;) {
        // TODO:
        vTaskDelay(pdMS_TO_TICKS(piccanteIDLE_SLEEP_MS));
    }
}
#endif

int main() {
    piccante::uart::sink0.init(0, 1, piccanteUART_SPEED);

    // Initialize TinyUSB stack
    board_init();
    tusb_init();


    board_init_after_tusb();

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
#ifndef DEBUG
    piccante::Log::set_log_level(static_cast<piccante::Log::Level>(cfg.log_level));
#else
    piccante::sys::settings::set_log_level(piccante::Log::Level::LEVEL_DEBUG);
#endif
    piccante::sys::stats::init_stats_collection();

#ifndef WIFI_ENABLED
    piccante::led::init(cfg.led_mode);
#endif


    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t txCanTaskHandle;
    static TaskHandle_t piccanteAndGvretTaskHandle;


    xTaskCreate(usbDeviceTask, "USB", configMINIMAL_STACK_SIZE, nullptr,
                configMAX_PRIORITIES - 6, &usbTaskHandle);
    xTaskCreate(can_recieveTask, "CAN RX", configMINIMAL_STACK_SIZE, nullptr, 5,
                &txCanTaskHandle);
    xTaskCreate(cmd_gvret_task, "PiCCANTE+GVRET", configMINIMAL_STACK_SIZE, nullptr, 6,
                &piccanteAndGvretTaskHandle);

    for (uint8_t i = 0; i < piccanteNUM_CAN_BUSSES; i++) {
        slcan_handler[i] = std::make_unique<piccante::slcan::handler>(
            piccante::usb_cdc::out(i + 1), i + 1, i);
        auto slcanTaskHandle = slcan_handler[i]->create_task();
        vTaskCoreAffinitySet(slcanTaskHandle, 0x01);
    }

    static TaskHandle_t canTaskHandle = piccante::can::create_task();

    vTaskCoreAffinitySet(usbTaskHandle, 0x01);
    vTaskCoreAffinitySet(txCanTaskHandle, 0x01);
    vTaskCoreAffinitySet(piccanteAndGvretTaskHandle, 0x01);
    vTaskCoreAffinitySet(canTaskHandle, 0x02);

#ifdef WIFI_ENABLED
    static TaskHandle_t wifiTaskHandle;
    xTaskCreate(wifi_task, "WIFI", configMINIMAL_STACK_SIZE, (void*)&cfg,
                configMAX_PRIORITIES - 5, &wifiTaskHandle);
    vTaskCoreAffinitySet(wifiTaskHandle, 0x01);
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
