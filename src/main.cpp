#include <pico/stdio.h>
#include <stdio.h>

#include <hardware/irq.h>
#include <hardware/regs/intctrl.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <memory.h>


#define CAN2040_ISR_IN_RAM 1

extern "C" {
#include "can2040.h"
}


volatile bool can_error = false;
struct can2040_stats can_stats;

#define MSG_BUFFER_SIZE 8
volatile struct {
    struct can2040_msg msg;
    uint32_t notify;
    bool valid;
} msg_buffer[MSG_BUFFER_SIZE];
volatile uint8_t write_idx = 0;
volatile uint8_t read_idx = 0;

static struct can2040 cbus;

static void CAN2040_ISR_FUNC(can2040_cb)(struct can2040* cd, uint32_t notify,
                                         struct can2040_msg* msg) {
    if (notify == CAN2040_NOTIFY_ERROR) {
        can_error = true;
        return;
    }

    // Store message in buffer for main loop
    uint8_t next_idx = (write_idx + 1) % MSG_BUFFER_SIZE;
    if (next_idx != read_idx) { // Check if buffer not full
        memcpy((void*)&msg_buffer[write_idx].msg, msg, sizeof(struct can2040_msg));
        msg_buffer[write_idx].notify = notify;
        msg_buffer[write_idx].valid = true;
        write_idx = next_idx;
    }
}

static int isr_counter = 0;

static void CAN2040_ISR_FUNC(PIOx_IRQHandler)(void) {
    isr_counter++;
    // Handle the PIO interrupt
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void) {
    uint32_t pio_num = 0;
    uint32_t sys_clock = SYS_CLK_HZ, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    // irq_set_exclusive_handler(PIO1_IRQ_0_IRQn, PIOx_IRQHandler);
    // NVIC_SetPriority(PIO1_IRQ_0_IRQn, 1);
    // NVIC_EnableIRQ(PIO1_IRQ_0_IRQn);

    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}


int main(void) {
    // Debugger fucks up when sleep is called...
    timer_hw->dbgpause = 0;

    stdio_init_all();

    // cyw43_arch_init();
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    printf("Hello Canbus?\n");

    canbus_setup();

    int counter = 0;

    while (1) {
        printf("ISR counter: %d\n", isr_counter);
        if (can_error) {
            can_error = false;
            can2040_get_statistics(&cbus, &can_stats);
            printf("CAN errors: parse=%u, rx=%u, tx=%u, tx_attempt=%u\n",
                   can_stats.parse_error, can_stats.rx_total, can_stats.tx_total,
                   can_stats.tx_attempt);
        }
        while (read_idx != write_idx) {
            if (msg_buffer[read_idx].valid) {
                struct can2040_msg* msg = (struct can2040_msg*)&msg_buffer[read_idx].msg;
                uint32_t notify = msg_buffer[read_idx].notify;

                printf("CAN message: notify=%d, ID=%x, Data=%x\n", notify, msg->id,
                       msg->data[0]);

                msg_buffer[read_idx].valid = false;
            }
            read_idx = (read_idx + 1) % MSG_BUFFER_SIZE;
        }

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
        sleep_ms(1000);
    }


    return 0;
}
