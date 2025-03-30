#include "CanBus.hpp"

#include <hardware/irq.h>
#include "FreeRTOS.h"
#include "queue.h"

#include "../Logger/Logger.hpp"


static QueueHandle_t can0_rx_queue;
static QueueHandle_t can0_tx_queue;

//


static struct can2040 cbus0;
static void can2040_cb_can0(struct can2040* cd, uint32_t notify,
                            struct can2040_msg* msg) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Add message processing code here...
    if (notify == CAN2040_NOTIFY_RX) {
        // Process received message - add to queue from ISR
        if (xQueueSendFromISR(can0_rx_queue, msg, &higher_priority_task_woken) !=
            pdTRUE) {
            // Queue is full - log overflow
            // Can't directly call logger from ISR, so set a flag or counter
        }
        portYIELD_FROM_ISR(higher_priority_task_woken);
    } else if (notify == CAN2040_NOTIFY_TX) {
        // Process transmitted message
    } else if (notify == CAN2040_NOTIFY_ERROR) {
        // Handle error
    }
}
static void PIOx_IRQHandler_CAN0(void) { can2040_pio_irq_handler(&cbus0); }

void canbus_setup(uint32_t can0_bitrate) {
    uint32_t pio_num = 0;
    uint32_t sys_clock = SYS_CLK_HZ, bitrate = can0_bitrate;
    uint32_t gpio_rx = CAN0_GPIO_RX, gpio_tx = CAN0_GPIO_TX;

    // Setup canbus
    can2040_setup(&cbus0, pio_num);
    can2040_callback_config(&cbus0, can2040_cb_can0);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler_CAN0);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Start canbus
    can2040_start(&cbus0, sys_clock, bitrate, gpio_rx, gpio_tx);
}


static void can0Task(void* parameters) {
    (void)parameters;

    vTaskDelay(5000); // TODO

    Log::Info("Starting CAN0 task...");

    // Create our message queues (64 messages each)
    can0_rx_queue = xQueueCreate(64, sizeof(can2040_msg));
    can0_tx_queue = xQueueCreate(64, sizeof(can2040_msg));

    if (can0_rx_queue == NULL || can0_tx_queue == NULL) {
        Log::Error("Failed to create CAN queues");
        return;
    }


    canbus_setup(500000);


    can2040_msg msg;
    for (;;) {
        // Process any pending TX messages from the queue
        while (xQueueReceive(can0_tx_queue, &msg, 0) == pdTRUE) {
            int res = can2040_transmit(&cbus0, &msg);
            if (res < 0) {
                Log::Error("CAN0: Failed to send message");
            }
        }

        // while (xQueueReceive(can0_rx_queue, &msg, 0) == pdTRUE) {
        //     // Process received message
        //     Log::Info("CAN0: Received message ID: ", Log::fmt("%x", msg.id));
        //     Log::Info("CAN0: Data: ");
        //     for (unsigned int i = 0; i < msg.dlc; i++) {
        //         Log::Info(" ", Log::fmt("%x", msg.data[i]));
        //     }
        //     // Add any additional processing here
        // }

        // Add any other periodic processing here
        vTaskDelay(pdMS_TO_TICKS(10)); // shorter delay for more responsive TX
    }
}

static TaskHandle_t can0TaskHandle;
TaskHandle_t& createCanTask(void* parameters) {
    (void)parameters;

    xTaskCreate(can0Task, "CAN0", CAN_STACK_DEPTH, nullptr, CAN_TASK_PRIORITY,
                &can0TaskHandle);
    return can0TaskHandle;
}

uint8_t get_can_0_rx_buffered_frames(void) {
    return (uint8_t)uxQueueMessagesWaiting(can0_rx_queue);
}

uint8_t get_can_0_tx_buffered_frames(void) {
    return (uint8_t)uxQueueMessagesWaiting(can0_tx_queue);
}

int send_can0(can2040_msg& msg) {
    // Queue the message for transmission by the CAN task
    if (xQueueSend(can0_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        Log::Error("CAN0: TX queue full");
        return -1;
    }
    return 0;
}

int receive_can0(can2040_msg& msg) {
    // Try to get a message from the queue
    if (xQueueReceive(can0_rx_queue, &msg, 0) == pdTRUE) {
        return get_can_0_rx_buffered_frames();
    }
    return -1;
}