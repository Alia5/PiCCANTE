
// NOLINTNEXTLINE
#include <array>
#include <boards/pico2_w.h>
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

static std::array<std::unique_ptr<piccante::slcan::handler>, piccanteNUM_CAN_BUSSES>
    slcan_handler = {nullptr};
static void can_recieveTask(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(2000); // TODO

    piccante::Log::info << ("Starting CAN Receive Task!\n");

    can2040_msg msg{};
    for (;;) {
        while (piccante::can::receive(0, msg) >= 0) {
            for (auto& handler : slcan_handler) {
                if (handler != nullptr) {
                    handler->comm_can_frame(msg);
                }
            }
        }
        // receive blocks waiting for a queue, no need to yield
    }
}

int main() {
    piccante::uart::sink0.init(0, 1, 115200);

    // Initialize TinyUSB stack
    board_init();
    tusb_init();


    board_init_after_tusb();

    piccante::Log::set_log_level(piccante::Log::Level::LEVEL_DEBUG);
    piccante::Log::info << "Starting up...\n";


    // littleFS

    if (piccante::fs::init()) {
        piccante::Log::info << "LittleFS mounted successfully\n";
    } else {
        piccante::Log::error << "LittleFS mount failed\n";
    }

    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t txCanTaskHandle;


    xTaskCreate(usbDeviceTask, "USB", configMINIMAL_STACK_SIZE, nullptr,
                configMAX_PRIORITIES - 6, &usbTaskHandle);
    xTaskCreate(can_recieveTask, "CAN RX", configMINIMAL_STACK_SIZE, nullptr, 5,
                &txCanTaskHandle);

    for (uint8_t i = 0; i < piccanteNUM_CAN_BUSSES; i++) {
        slcan_handler[i] = std::make_unique<piccante::slcan::handler>(
            piccante::usb_cdc::out(i + 1), 0, i);
        auto slcanTaskHandle = slcan_handler[i]->create_task();
        vTaskCoreAffinitySet(slcanTaskHandle, 0x01);
    }

    static TaskHandle_t canTaskHandle = piccante::can::create_task();

    vTaskCoreAffinitySet(usbTaskHandle, 0x01);   // Set the task to run on core 1
    vTaskCoreAffinitySet(canTaskHandle, 0x02);   // Set the task to run on core 2
    vTaskCoreAffinitySet(txCanTaskHandle, 0x01);

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
