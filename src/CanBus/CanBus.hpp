#pragma once

extern "C" {
#include "can2040.h"
}
#include "FreeRTOS.h"
#include "task.h"

#include <array>

constexpr uint32_t CAN0_GPIO_RX = 4;
constexpr uint32_t CAN0_GPIO_TX = 5;

constexpr UBaseType_t CAN_TASK_PRIORITY = configMAX_PRIORITIES - 5;
constexpr uint32_t CAN_STACK_DEPTH = 256;

TaskHandle_t& createCanTask(void* parameters);

uint8_t get_can_0_rx_buffered_frames(void);
uint8_t get_can_0_tx_buffered_frames(void);

int send_can0(can2040_msg& msg);
int receive_can0(can2040_msg& msg);