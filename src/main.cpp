
// NOLINTNEXTLINE
#include <boards/pico2_w.h>
#include <cstring>
#include <pico/error.h>
#include <bsp/board_api.h>

#include "CommProto/gvret/gvret.hpp"
#include "Logger/Logger.hpp"
#include "device/usbd.h"
#include "class/cdc/cdc_device.h"
#include "CanBus/CanBus.hpp"
#include "FreeRTOSConfig.h"
#include "pico/cyw43_arch.h"

#include "projdefs.h"
#include "portable.h"
#include "queue.h"
#include "task.h"

#include "timers.h"
#include "tusb.h"
#include <string>

#include "outstream/usb_cdc_stream.hpp"
#include "outstream/uart_stream.hpp"


#include "CommProto/lawicel/lawicel.hpp"
#include "CommProto/gvret/handler.hpp"
#include "tusb_config.h"

#include <lfs.h>
#include "fs/littlefs_driver.hpp"


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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static piccante::lawicel::handler lawicel_handler(piccante::usb_cdc::out1, 0);
static piccante::gvret::handler gvret_handler =
    piccante::gvret::handler(piccante::usb_cdc::out0);

static void lawicel_task(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(6000); // TODO

    piccante::Log::info << ("Starting Lavicel Handler!\n");

    for (;;) {
        std::string lawicel_cmd;
        while (tud_cdc_n_available(1) > 0) {
            char const c = tud_cdc_n_read_char(1);
            lawicel_cmd += c;
            if (c == '\r' || c == '\n') {
                break; // End of command
            }
        }
        if (!lawicel_cmd.empty()) {
            // Process the command
            lawicel_handler.handleCmd(lawicel_cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // shorter delay for more responsive TX
    }
}

static void gvret_task(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(2000); // TODO

    piccante::Log::info << ("Starting Gvret Handler!\n");

    for (;;) {
        while (tud_cdc_n_available(0) > 0) {
            char c = tud_cdc_n_read_char(0);
            gvret_handler.process_byte(c);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // shorter delay for more responsive TX
    }
}

static void can_recieveTask(void* parameter) {
    (void)parameter;
    // Wait until can is up.
    vTaskDelay(2000); // TODO

    piccante::Log::info << ("Starting CAN Receive Task!\n");

    for (;;) {
        can2040_msg msg{};
        while (piccante::can::receive(0, msg) >= 0) {
            piccante::gvret::comm_can_frame(0, msg, piccante::usb_cdc::out0);
            lawicel_handler.handleCanFrame(msg);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // shorter delay for more responsive TX
    }
}


int main() {
    piccante::uart::sink0.init(0, 1, 115200);

    // Initialize TinyUSB stack
    board_init();
    tusb_init();

    // TinyUSB board init callback after init
    if (board_init_after_tusb != nullptr) {
        board_init_after_tusb();
    }

    piccante::Log::set_log_level(piccante::Log::Level::LEVEL_DEBUG);
    piccante::Log::info << "Starting up...\n";

    cyw43_arch_init();

    // littleFS

    if (piccante::fs::init()) {
        piccante::Log::info << "LittleFS mounted successfully\n";

        // Example: Read a file
        lfs_file_t readFile;
        int err = lfs_file_open(&piccante::fs::lfs, &readFile, "test.txt", LFS_O_RDONLY);
        if (err == LFS_ERR_OK) {
            char buffer[64];
            lfs_ssize_t const bytesRead =
                lfs_file_read(&piccante::fs::lfs, &readFile, buffer, sizeof(buffer) - 1);
            if (bytesRead >= 0) {
                buffer[bytesRead] = '\0'; // Null-terminate the string
                piccante::Log::info << "Read from file: " << buffer << "\n";
            }
            lfs_file_close(&piccante::fs::lfs, &readFile);
        } else {
            piccante::Log::error << "Failed to read file\n";
        }

        // // Example: Write a file
        // lfs_file_t file;
        // err = lfs_file_open(&piccante::fs::lfs, &file, "hello.txt",
        //                     LFS_O_WRONLY | LFS_O_CREAT);
        // if (err == LFS_ERR_OK) {
        //     const char* data = "Hello from LittleFS on Pico!";
        //     lfs_file_write(&piccante::fs::lfs, &file, data, strlen(data));
        //     lfs_file_close(&piccante::fs::lfs, &file);
        //     piccante::Log::info << "File written successfully\n";
        // }
    }

    //

    static TaskHandle_t blinkTaskHandle;
    static TaskHandle_t printTaskHandle;
    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t debugTaskHandle;

    static TaskHandle_t gvretTaskHandle;
    static TaskHandle_t lawicelTaskHandle;
    static TaskHandle_t txCanTaskHandle;


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

    xTaskCreate(gvret_task, "Gvret", configMINIMAL_STACK_SIZE, nullptr, 5,
                &gvretTaskHandle);
    xTaskCreate(can_recieveTask, "CAN RX", configMINIMAL_STACK_SIZE, nullptr, 5,
                &txCanTaskHandle);

    static TaskHandle_t canTaskHandle = piccante::can::createTask(nullptr);

    vTaskCoreAffinitySet(blinkTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(printTaskHandle, 0x01); // Set the task to run on core 2
    vTaskCoreAffinitySet(usbTaskHandle, 0x01);   // Set the task to run on core 1
    vTaskCoreAffinitySet(debugTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(canTaskHandle, 0x02);   // Set the task to run on core 2
    vTaskCoreAffinitySet(lawicelTaskHandle, 0x01);
    vTaskCoreAffinitySet(gvretTaskHandle, 0x01);
    vTaskCoreAffinitySet(txCanTaskHandle, 0x01);


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
    uint32_t const count = tud_cdc_n_read(itf, buf, sizeof(buf));
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
