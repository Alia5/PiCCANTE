
#include <pico/stdio.h>
#include <bsp/board_api.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#include "usb/usb_descriptors.h"
#include <iostream>
#include <string>

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
        tud_cdc_n_write_str(1, "Hello from USB CDC1!!\n");
        //tud_cdc_n_write_str(0, "Hello from USB CDC000!!\n");
        tud_cdc_n_write_flush(1);
        // tud_cdc_n_write_flush(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    while (1) {
        // read from stdin (using std:cin) and switch cmd
        std::string cmd;
        if (std::getline(std::cin, cmd)) {
            if (cmd == "cpu") {
                char buffer[configSTATS_BUFFER_MAX_LENGTH];
                vTaskGetRunTimeStats(buffer);
                std::cout << buffer << std::endl;
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void) {

    // Initialize TinyUSB stack
    board_init();
    tusb_init();

    // TinyUSB board init callback after init
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    stdio_init_all();

    printf("FreeRTOS on RP2040\n");
    cyw43_arch_init();

    static TaskHandle_t blinkTaskHandle;
    static TaskHandle_t printTaskHandle;
    static TaskHandle_t usbTaskHandle;

    xTaskCreate(blinkTask, "Blink", 256, nullptr, 1, &blinkTaskHandle);
    xTaskCreate(printTask, "Print", 256, nullptr, 1, &printTaskHandle);
    xTaskCreate(usbDeviceTask, "USB", 256, nullptr, configMAX_PRIORITIES-2, &usbTaskHandle);

    vTaskCoreAffinitySet(blinkTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(printTaskHandle, 0x02); // Set the task to run on core 2

    vTaskStartScheduler();

    return 0;
}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    // read the available data 
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
}

void vApplicationMallocFailedHook() { configASSERT((volatile void*)nullptr); }

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