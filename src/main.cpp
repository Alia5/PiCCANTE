
#include <pico/stdio.h>
#include <bsp/board_api.h>

#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "tusb.h"
#include "usb/usb_descriptors.h"
#include <iostream>
#include <string>

#include <hardware/irq.h>
#include <hardware/regs/intctrl.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <memory.h>


#include "can2040.pio.h"
extern "C" {
#include "can2040.h"
}


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


static struct can2040 cbus;

static void can2040_cb(struct can2040* cd, uint32_t notify, struct can2040_msg* msg) {
    // Add message processing code here...
    if (notify == CAN2040_NOTIFY_RX) {
        // Process received message
        // printf("Received CAN message: ID=%x, Data=%x\n", msg->id, msg->data[0]);
    } else if (notify == CAN2040_NOTIFY_TX) {
        // Process transmitted message
        // printf("Transmitted CAN message: ID=%x, Data=%x\n", msg->id, msg->data[0]);
    } else if (notify == CAN2040_NOTIFY_ERROR) {
        // Handle error
        // printf("CAN error occurred\n");
    }
}
static void PIOx_IRQHandler(void) {
    // Handle the PIO interrupt
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void) {
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    // irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    // NVIC_SetPriority(PIO0_IRQ_0, 1);
    // NVIC_EnableIRQ(PIO0_IRQ_0);

    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

static void canTask(void* parameters) {
    (void)parameters;

    vTaskDelay(1000);

    printf("CAN Task started\n");
    canbus_setup();

    int counter = 0;


    while (1) {
        // Send a test message every second
        struct can2040_msg msg;
        msg.id = 0x123; // Standard ID
        msg.dlc = 8;    // 8 bytes of data
        msg.data[0] = counter & 0xFF;
        msg.data[1] = (counter >> 8) & 0xFF;
        msg.data[2] = 0xAA;
        msg.data[3] = 0xBB;
        msg.data[4] = 0xCC;
        msg.data[5] = 0xDD;
        msg.data[6] = 0xEE;
        msg.data[7] = 0xFF;

        printf("Sending CAN message, counter=%d\n", counter);
        int ret = can2040_transmit(&cbus, &msg);
        printf("Transmit result: %d\n", ret);

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
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


        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


int main(void) {
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

    printf("FreeRTOS on RP2040\n");
    cyw43_arch_init();


    static TaskHandle_t blinkTaskHandle;
    static TaskHandle_t printTaskHandle;
    static TaskHandle_t usbTaskHandle;
    static TaskHandle_t debugTaskHandle;
    static TaskHandle_t canTaskHandle;


    xTaskCreate(blinkTask, "Blink", 256, nullptr, 2, &blinkTaskHandle);
    xTaskCreate(printTask, "Print", 256, nullptr, 1, &printTaskHandle);
    xTaskCreate(usbDeviceTask, "USB", 256, nullptr, configMAX_PRIORITIES - 6,
                &usbTaskHandle);
    xTaskCreate(DebugCommandTask, "Debug", 256, nullptr, 1, &debugTaskHandle);
    xTaskCreate(canTask, "CAN", 256, nullptr, configMAX_PRIORITIES - 2, &canTaskHandle);


    vTaskCoreAffinitySet(blinkTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(printTaskHandle, 0x02); // Set the task to run on core 2
    vTaskCoreAffinitySet(usbTaskHandle, 0x01);   // Set the task to run on core 1
    vTaskCoreAffinitySet(debugTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(canTaskHandle, 0x02);   // Set the task to run on core 2


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
