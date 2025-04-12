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
#include "CanBus.hpp"

#include <cstdint>
#include <cstddef>
#include <hardware/irq.h>
#include <hardware/platform_defs.h>
#include <hardware/regs/intctrl.h>
#include "can2040.h"
#include "projdefs.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "semphr.h"
#include "Logger/Logger.hpp"
#include <array>
#include "fmt.hpp"
#include <hardware/structs/pio.h>
#include <lfs.h>
#include <fs/littlefs_driver.hpp>

namespace piccante::can {


struct CanQueues {
    QueueHandle_t rx;
    QueueHandle_t tx;
};

namespace {
// NOLINTNEXTLINE: cppcoreguidelines-avoid-non-const-global-variables
std::array<CanQueues, NUM_BUSSES> can_queues = {};
// NOLINTNEXTLINE: cppcoreguidelines-avoid-non-const-global-variables
std::array<can2040, NUM_BUSSES> can_buses = {};
} // namespace

struct CanGPIO {
    uint8_t pin_rx;
    uint8_t pin_tx;
    uint8_t pio_num;
    uint8_t pio_irq;
};


constexpr std::array<const CanGPIO, NUM_BUSSES> CAN_GPIO = {
    {{
         .pin_rx = piccanteCAN0_RX_PIN,
         .pin_tx = piccanteCAN0_TX_PIN,
         .pio_num = 0,
         .pio_irq = PIO0_IRQ_0,
     },
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_2 ||                                       \
    piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
     {
         .pin_rx = piccanteCAN1_RX_PIN,
         .pin_tx = piccanteCAN1_TX_PIN,
         .pio_num = 1,
         .pio_irq = PIO1_IRQ_0,
     },
#endif
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
     {
         .pin_rx = piccanteCAN2_RX_PIN,
         .pin_tx = piccanteCAN2_TX_PIN,
         .pio_num = 2,
         .pio_irq = PIO2_IRQ_0,
     }
#endif
    }};


#pragma pack(push, 1)
struct can_settings_file {
    uint8_t num_busses;
    std::array<CanSettings, 3> bus_config;
};
#pragma pack(pop)

namespace {

// NOLINTNEXTLINE: cppcoreguidelines-avoid-non-const-global-variables
can_settings_file settings = {};

void can2040_cb_can0(struct can2040* /*cd*/, uint32_t notify, // NOLINT
                     struct can2040_msg* msg) {               // NOLINT
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Add message processing code here...
    if (notify == CAN2040_NOTIFY_RX) {
        // Process received message - add to queue from ISR
        if (xQueueSendFromISR(can_queues[0].rx, msg, &higher_priority_task_woken) !=
            pdTRUE) {
            // Queue is full - log overflow
            // Can't directly call logger from ISR, so set a flag or counter
        }
    }
    // else if (notify == CAN2040_NOTIFY_TX) {
    //     // Process transmitted message
    // } else if (notify == CAN2040_NOTIFY_ERROR) {
    //     // Handle error
    // }
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_2 ||                                       \
    piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
void can2040_cb_can1(struct can2040* /*cd*/, uint32_t notify, // NOLINT
                     struct can2040_msg* msg) {               // NOLINT
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Add message processing code here...
    if (notify == CAN2040_NOTIFY_RX) {
        // Process received message - add to queue from ISR
        if (xQueueSendFromISR(can_queues[1].rx, msg, &higher_priority_task_woken) !=
            pdTRUE) {
            // Queue is full - log overflow
            // Can't directly call logger from ISR, so set a flag or counter
        }
    }
    // else if (notify == CAN2040_NOTIFY_TX) {
    //     // Process transmitted message
    // } else if (notify == CAN2040_NOTIFY_ERROR) {
    //     // Handle error
    // }
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#endif
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
void can2040_cb_can2(struct can2040* /*cd*/, uint32_t notify, // NOLINT
                     struct can2040_msg* msg) {               // NOLINT
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Add message processing code here...
    if (notify == CAN2040_NOTIFY_RX) {
        // Process received message - add to queue from ISR
        if (xQueueSendFromISR(can_queues[2].rx, msg, &higher_priority_task_woken) !=
            pdTRUE) {
            // Queue is full - log overflow
            // Can't directly call logger from ISR, so set a flag or counter
        }
    }
    // else if (notify == CAN2040_NOTIFY_TX) {
    //     // Process transmitted message
    // } else if (notify == CAN2040_NOTIFY_ERROR) {
    //     // Handle error
    // }
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#endif
void PIOx_IRQHandler_CAN0() {
    BaseType_t const higher_priority_task_woken = pdFALSE;
    can2040_pio_irq_handler(&(can_buses[0]));
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_2 ||                                       \
    piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3

void PIOx_IRQHandler_CAN1() {
    BaseType_t const higher_priority_task_woken = pdFALSE;
    can2040_pio_irq_handler(&(can_buses[1]));
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#endif
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3

void PIOx_IRQHandler_CAN2() {
    BaseType_t const higher_priority_task_woken = pdFALSE;
    can2040_pio_irq_handler(&(can_buses[2]));
    portYIELD_FROM_ISR(higher_priority_task_woken); // NOLINT
}
#endif


std::array<bool, 3> canbus_initial_setup_done = {
    false,
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_2 ||                                       \
    piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
    false,
#endif
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
    false,
#endif
};

void canbus_setup_initial(uint8_t bus) {
    can2040_setup(&(can_buses[bus]), CAN_GPIO[bus].pio_num);


    switch (bus) { // NOLINT
        case 0:
            can2040_callback_config(&(can_buses[bus]), can2040_cb_can0);
            irq_set_exclusive_handler(CAN_GPIO[bus].pio_irq, PIOx_IRQHandler_CAN0);
            break;
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_2 ||                                       \
    piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
        case 1:
            can2040_callback_config(&(can_buses[bus]), can2040_cb_can1);
            irq_set_exclusive_handler(CAN_GPIO[bus].pio_irq, PIOx_IRQHandler_CAN1);
            break;
#endif
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3

        case 2:
            can2040_callback_config(&(can_buses[bus]), can2040_cb_can2);
            irq_set_exclusive_handler(CAN_GPIO[bus].pio_irq, PIOx_IRQHandler_CAN2);
            break;
#endif
        default:
            Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
            return;
    }

    irq_set_enabled(CAN_GPIO[bus].pio_irq, false);
    // Set core affinity for this interrupt

    if (CAN_GPIO[bus].pio_num == 0) {
        // For PIO0 interrupts
        hw_set_bits(&pio0_hw->inte1, (1u << CAN_GPIO[bus].pio_irq)); // Enable on core 1
        hw_clear_bits(&pio0_hw->inte0,
                      (1u << CAN_GPIO[bus].pio_irq)); // Disable on core 0
    } else if (CAN_GPIO[bus].pio_num == 1) {
        // For PIO1 interrupts
        hw_set_bits(&pio1_hw->inte1, (1u << CAN_GPIO[bus].pio_irq)); // Enable on core 1
        hw_clear_bits(&pio1_hw->inte0,
                      (1u << CAN_GPIO[bus].pio_irq)); // Disable on core 0
    }
#if piccanteNUM_CAN_BUSSES == piccanteCAN_NUM_3
    else if (CAN_GPIO[bus].pio_num == 2) {
        // For PIO2 interrupts if your board supports it
        hw_set_bits(&pio2_hw->inte1, (1u << CAN_GPIO[bus].pio_irq)); // Enable on core 1
        hw_clear_bits(&pio2_hw->inte0,
                      (1u << CAN_GPIO[bus].pio_irq)); // Disable on core 0
    }
#endif

    irq_set_priority(CAN_GPIO[bus].pio_irq, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    irq_set_enabled(CAN_GPIO[bus].pio_irq, true);
    canbus_initial_setup_done[bus] = true;
}

void canbus_setup(uint8_t bus, uint32_t bitrate) {
    if (!canbus_initial_setup_done[bus]) {
        canbus_setup_initial(bus);
    }

    can2040_start(&(can_buses[bus]), SYS_CLK_HZ, bitrate, CAN_GPIO[bus].pin_rx,
                  CAN_GPIO[bus].pin_tx);
}

SemaphoreHandle_t settings_mutex = nullptr;

void store_settings() {
    Log::debug << "Storing CAN settings...\n";
    if (settings_mutex == nullptr ||
        xSemaphoreTake(settings_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Log::error << "Failed to take settings mutex for store_settings\n";
        return;
    }

    lfs_file_t writeFile;
    const int err = lfs_file_open(&piccante::fs::lfs, &writeFile, "can_settings",
                                  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err == LFS_ERR_OK) {
        if (lfs_file_write(&piccante::fs::lfs, &writeFile, &settings, sizeof(settings)) <
            0) {
            Log::error << "Failed to write CAN settings file\n";
        }
        if (const auto err = lfs_file_close(&piccante::fs::lfs, &writeFile);
            err != LFS_ERR_OK) {
            Log::error << "Failed to close CAN settings file: " << fmt::sprintf("%d", err)
                       << "\n";
        }
    } else {
        Log::error << "Failed to write CAN settings file\n";
    }
    xSemaphoreGive(settings_mutex);
}

void canTask(void* parameters) {
    (void)parameters;

    Log::info << "Starting CAN task...\n";

    settings_mutex = xSemaphoreCreateMutex();
    if (settings_mutex == nullptr) {
        Log::error << "Failed to create settings mutex\n";
        return;
    }

    lfs_file_t readFile;
    const int err =
        lfs_file_open(&piccante::fs::lfs, &readFile, "can_settings", LFS_O_RDONLY);
    if (err == LFS_ERR_OK) {
        lfs_ssize_t const bytesRead =
            lfs_file_read(&piccante::fs::lfs, &readFile, &settings, sizeof(settings));
        if (bytesRead >= 0) {
            for (std::size_t i = 0; i < settings.num_busses && i < piccanteNUM_CAN_BUSSES;
                 i++) {
                if (settings.bus_config[i].enabled) {
                    Log::info << "Enabling CAN bus " << fmt::sprintf("%d", i)
                              << " with bitrate " << settings.bus_config[i].bitrate
                              << " from stored settings\n";
                    canbus_setup(i, settings.bus_config[i].bitrate);
                }
            }
        }
        lfs_file_close(&piccante::fs::lfs, &readFile);
    } else {
        Log::error << "Failed to read CAN settings file\n";
    }

    for (std::size_t i = 0; i < NUM_BUSSES; i++) {
        can_queues[i].rx = xQueueCreate(CAN_QUEUE_SIZE, sizeof(can2040_msg));
        can_queues[i].tx = xQueueCreate(CAN_QUEUE_SIZE, sizeof(can2040_msg));

        if (can_queues[i].rx == NULL || can_queues[i].tx == NULL) {
            Log::error << "Failed to create CAN queues for bus " << i << "\n";
            return;
        }
    }


    can2040_msg msg = {};
    for (;;) {
        bool did_tx = false;
        for (std::size_t i = 0; i < NUM_BUSSES; i++) {
            while (xQueueReceive(can_queues[i].tx, &msg, 0) == pdTRUE) {
                did_tx = true;
                int res = can2040_transmit(&(can_buses[i]), &msg);
                if (res < 0) {
                    Log::error << "CAN" << fmt::sprintf("%d", i)
                               << ": Failed to send message\n";
                }
            }
        }
        if (!did_tx) {
            // No messages to send, so we can sleep
            vTaskDelay(pdMS_TO_TICKS(CAN_IDLE_SLEEP_TIME_MS));
        }
    }
}

TaskHandle_t canTaskHandle; // NOLINT
} // namespace

TaskHandle_t& create_task() {
    xTaskCreate(canTask, "CAN", configMINIMAL_STACK_SIZE, nullptr, CAN_TASK_PRIORITY,
                &canTaskHandle);
    return canTaskHandle;
}

int send_can(uint8_t bus, can2040_msg& msg) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return -1;
    }
    if (!settings.bus_config[bus].enabled) {
        Log::error << "CAN bus " << fmt::sprintf("%d", bus) << " is not enabled\n";
        return -1;
    }
    if (settings.bus_config[bus].listen_only) {
        Log::error << "CAN bus " << fmt::sprintf("%d", bus)
                   << " is in listen-only mode\n";
        return -1;
    }
    if (xQueueSend(can_queues[bus].tx, &msg, pdMS_TO_TICKS(CAN_QUEUE_TIMEOUT_MS)) !=
        pdTRUE) {
        Log::error << "CAN bus " << fmt::sprintf("%d", bus) << ": TX queue full\n";
        return -1;
    }
    return 0;
}
int receive(uint8_t bus, can2040_msg& msg) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return -1;
    }
    if (xQueueReceive(can_queues[bus].rx, &msg, configTICK_RATE_HZ) == pdTRUE) {
        return get_can_rx_buffered_frames(bus);
    }
    return -1;
}

int get_can_rx_buffered_frames(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return -1;
    }
    return (uint8_t)uxQueueMessagesWaiting(can_queues[bus].rx);
}
int get_can_tx_buffered_frames(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return -1;
    }
    return (uint8_t)uxQueueMessagesWaiting(can_queues[bus].tx);
}

void enable(uint8_t bus, uint32_t bitrate) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return;
    }
    if (settings.bus_config[bus].enabled) {
        Log::warning << "CAN bus " << fmt::sprintf("%d", bus)
                     << " is already enabled - resetting\n";
        set_bitrate(bus, bitrate);
        return;
    }
    canbus_setup(bus, bitrate);
    settings.bus_config[bus].bitrate = bitrate;
    settings.bus_config[bus].enabled = true;
    store_settings();
}

void disable(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return;
    }
    can2040_stop(&(can_buses[bus]));
    if (!settings.bus_config[bus].enabled) {
        settings.bus_config[bus].enabled = false;
        store_settings();
    }
}

void set_bitrate(uint8_t bus, uint32_t bitrate) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return;
    }
    if (settings.bus_config[bus].enabled) {
        can2040_stop(&(can_buses[bus]));
        canbus_setup(bus, bitrate);
        settings.bus_config[bus].enabled = true;
    }
    if (settings.bus_config[bus].bitrate != bitrate) {
        settings.bus_config[bus].bitrate = bitrate;
        store_settings();
    }
}

bool is_enabled(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return false;
    }
    return settings.bus_config[bus].enabled;
}
uint32_t get_bitrate(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return DEFAULT_BUS_SPEED;
    }
    return settings.bus_config[bus].bitrate;
}

bool is_listenonly(uint8_t bus) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return false;
    }
    return settings.bus_config[bus].listen_only;
}

void set_listenonly(uint8_t bus, bool listen_only) {
    if (bus >= piccanteNUM_CAN_BUSSES || bus >= settings.num_busses) {
        Log::error << "Invalid CAN bus number: " << fmt::sprintf("%d", bus) << "\n";
        return;
    }
    if (settings.bus_config[bus].listen_only == listen_only) {
        return;
    }
    settings.bus_config[bus].listen_only = listen_only;
    store_settings();
}

} // namespace piccante::can