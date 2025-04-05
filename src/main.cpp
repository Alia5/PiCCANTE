
// NOLINTNEXTLINE
#include <pico/stdio.h>
#include <bsp/board_api.h>

#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "timers.h"
#include "tusb.h"
#include <string>

#include <pico/stdlib.h>
#include <cstdio>

#include "outstream/usb_cdc_stream.hpp" // NOLINT

#include "Logger/Logger.hpp"

#include "CommProto/Lawicel/Lawicel.hpp"
#include "CommProto/gvret/handler.hpp" // NOLINT (stupid...)
#include "fmt.hpp"


static void blinkTask(void *pvParameters) {
    (void) pvParameters; // unused parameter

    vTaskDelay(100);

    while (1) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void printTask(void *pvParameters) {
    (void) pvParameters; // unused parameter

    vTaskDelay(300);

    while (1) {
        // tud_cdc_n_write_flush(1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void usbDeviceTask(void* parameters) {
    (void)parameters;

    vTaskDelay(108);
    tud_init(0);

    for (;;) {
        tud_task(); // tinyusb device task
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


static void DebugCommandTask(void* parameters) {
    (void)parameters;

    vTaskDelay(1000);

    // Buffer for command input
    static char cmdBuffer[64];
    static int bufferIndex = 0;

    while (1) {
        // Check if data is available on stdin without blocking
        int c = getchar_timeout_us(0); // Non-blocking getchar

        if (c != PICO_ERROR_TIMEOUT) {
            // Handle backspace
            if (c == '\b' || c == 127) {
                if (bufferIndex > 0) {
                    bufferIndex--;
                    printf("\b \b"); // Erase character
                }
            }
            // Handle newline/carriage return
            else if (c == '\r' || c == '\n') {
                cmdBuffer[bufferIndex] = '\0'; // Null terminate
                printf("\r\n");                // Echo newline

                // Process command if buffer not empty
                if (bufferIndex > 0) {
                    std::string cmd(cmdBuffer);

                    if (cmd == "stat") {
                        // char buffer[500];
                        // vTaskGetRunTimeStatistics(buffer, 500);
                        // printf("%s", buffer);
                    }
                }

                // Reset buffer for next command
                bufferIndex = 0;
            }
            // Regular character
            else if (bufferIndex < sizeof(cmdBuffer) - 1) {
                cmdBuffer[bufferIndex++] = c;
                putchar(c); // Echo character
            }
        }


        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static Lawicel::Handler handler;

static void lawicel_task(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(6000); // TODO

    Log::info << ("Starting Lavicel Handler!\n");

    auto gvret_handler = gvret::handler(usb_cdc::out0);

    uint8_t buff[128] = {0};
    for (;;) {
        while (tud_cdc_n_available(0) > 0) {
            char c = tud_cdc_n_read_char(0);
            gvret_handler.process_byte(c);
        }

        can2040_msg msg;
        while (receive_can0(msg) >= 0) {
            // Process received message
            gvret::comm_can_frame(0, msg, usb_cdc::out0);
        }

        // char cmdBuffer[128];
        // int serialCnt = 0;

        // serialCnt = 0;
        // while (tud_cdc_n_available(1) > 0) {
        //     char c = tud_cdc_n_read_char(1);
        //     cmdBuffer[serialCnt++] = c;
        //     if (serialCnt >= sizeof(cmdBuffer) - 1) {
        //         break; // Prevent buffer overflow
        //     }
        //     if (c == '\r' || c == '\n') {
        //         break; // End of command
        //     }
        // }
        // if (serialCnt == 2 && (cmdBuffer[1] == '\r' || cmdBuffer[1] == '\n')) {
        //     // Handle single character commands
        //     char cmd = cmdBuffer[0];
        //     handler.handleShortCmd(cmd);
        // }
        // if (serialCnt > 2) {
        //     cmdBuffer[serialCnt] = '\0'; // Null terminate
        //     serialCnt = 0;
        //     // Process command
        //     handler.handleLongCmd(cmdBuffer);
        // }

        // // get buffered frames from can
        // can2040_msg msg;
        // while (receive_can0(msg) >= 0) {
        //     // Process received message

        //     Log::Debug("CAN0: Received message ID:",
        //                fmt::sprintf("0x%x,", msg.id),
        //                "Data:",
        //                fmt::sprintf("0x%x", msg.data32[0]));

        //     // TODO:
        //     handler.sendFrameToBuffer(msg, 0);
        // }


        vTaskDelay(pdMS_TO_TICKS(10)); // shorter delay for more responsive TX
    }
}


int main() {
    // Debugger fucks up when sleep is called...
    // timer_hw->dbgpause = 0;

    // Initialize TinyUSB stack
    board_init();
    tusb_init();

    // TinyUSB board init callback after init
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    stdio_init_all();

    Log::set_log_level(Log::Level::LEVEL_DEBUG);
    Log::info << "Starting up...\n";

    cyw43_arch_init();

    static TaskHandle_t blinkTaskHandle;
    static TaskHandle_t printTaskHandle;
    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t debugTaskHandle;

    static TaskHandle_t lawicelTaskHandle;


    xTaskCreate(blinkTask, "Blink", configMINIMAL_STACK_SIZE, nullptr, 2,
                &blinkTaskHandle);
    xTaskCreate(printTask, "Print", configMINIMAL_STACK_SIZE, nullptr, 1,
                &printTaskHandle);
    xTaskCreate(usbDeviceTask, "USB", configMINIMAL_STACK_SIZE, nullptr,
                configMAX_PRIORITIES - 6, &usbTaskHandle);
    xTaskCreate(DebugCommandTask, "Debug", configMINIMAL_STACK_SIZE, nullptr, 1,
                &debugTaskHandle);
    xTaskCreate(lawicel_task, "Lawicel", configMINIMAL_STACK_SIZE, nullptr, 5,
                &lawicelTaskHandle);

    static TaskHandle_t canTaskHandle = createCanTask(nullptr);

    vTaskCoreAffinitySet(blinkTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(printTaskHandle, 0x01); // Set the task to run on core 2
    vTaskCoreAffinitySet(usbTaskHandle, 0x01);   // Set the task to run on core 1
    vTaskCoreAffinitySet(debugTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(canTaskHandle, 0x02);   // Set the task to run on core 2
    vTaskCoreAffinitySet(lawicelTaskHandle, 0x01);


    vTaskStartScheduler();

    return 0;
}

// callback when data is received on a CDC interface
// NOLINTNEXTLINE
void tud_cdc_rx_cb(uint8_t itf)
{
    // Log::debug << "CDC RX Callback\n";
    if (itf == 1) {
        // CDC is handled on Lawicel-Task
        return; // ignore CDC1
    }
    if (itf == 0) {
        return;
    }
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    // read the available data 
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
}

// NOLINTNEXTLINE
void vApplicationMallocFailedHook() { configASSERT((volatile void*)nullptr); }
// NOLINTNEXTLINE
void vApplicationIdleHook() {
    volatile size_t xFreeHeapSpace;

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
