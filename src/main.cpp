
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"


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

    while (1) {
        printf("Hello from FreeRTOS task!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    stdio_init_all();

    printf("FreeRTOS on RP2040\n");
    cyw43_arch_init();

    static TaskHandle_t blinkTaskHandle;
    static TaskHandle_t printTaskHandle;

    xTaskCreate(blinkTask, "Blink", 256, nullptr, 1, &blinkTaskHandle);
    xTaskCreate(printTask, "Print", 256, nullptr, 1, &printTaskHandle);

    vTaskCoreAffinitySet(blinkTaskHandle, 0x01); // Set the task to run on core 1
    vTaskCoreAffinitySet(printTaskHandle, 0x02); // Set the task to run on core 2

    vTaskStartScheduler();

    return 0;
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