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
#pragma once

extern "C" {
#include "can2040.h"
}
#include "FreeRTOS.h"
#include "task.h"

#include <array>

namespace piccante::can {

constexpr uint8_t NUM_BUSSES = piccanteNUM_CAN_BUSSES;

constexpr uint8_t CAN_QUEUE_SIZE = piccanteCAN_QUEUE_SIZE;

constexpr uint8_t CAN_IDLE_SLEEP_TIME_MS = piccanteCAN_IDLE_SLEEP_MS;
constexpr uint8_t CAN_QUEUE_TIMEOUT_MS = piccanteCAN_QUEUE_TIMEOUT_MS;


constexpr uint32_t CAN0_GPIO_RX = piccanteCAN0_RX_PIN;
constexpr uint32_t CAN0_GPIO_TX = piccanteCAN0_TX_PIN;

#if piccanteNUM_CAN_BUSSES == picacanteCAN_NUM_2
constexpr uint32_t CAN1_GPIO_RX = piccanteCAN1_RX_PIN;
constexpr uint32_t CAN1_GPIO_TX = piccanteCAN1_TX_PIN;
#endif
#if piccanteNUM_CAN_BUSSES == picacanteCAN_NUM_3
constexpr uint32_t CAN2_GPIO_RX = piccanteCAN2_RX_PIN;
constexpr uint32_t CAN2_GPIO_TX = piccanteCAN2_TX_PIN;
#endif

constexpr UBaseType_t CAN_TASK_PRIORITY = configMAX_PRIORITIES - 5;

struct CanSettings {
    bool enabled;
    uint32_t bitrate;
};

TaskHandle_t& createTask(void* parameters);

int send_can(uint8_t bus, can2040_msg& msg);
int receive(uint8_t bus, can2040_msg& msg);

int get_can_rx_buffered_frames(uint8_t bus);
int get_can_tx_buffered_frames(uint8_t bus);

void enable(uint8_t bus, uint32_t bitrate);
void disable(uint8_t bus);
void set_bitrate(uint8_t bus, uint32_t bitrate);
uint32_t get_bitrate(uint8_t bus);
}