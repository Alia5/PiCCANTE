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

constexpr uint8_t CAN_QUEUE_TIMEOUT_MS = piccanteCAN_QUEUE_TIMEOUT_MS;

constexpr UBaseType_t CAN_TASK_PRIORITY = configMAX_PRIORITIES - 5;
constexpr uint32_t DEFAULT_BUS_SPEED = 500000;

#pragma pack(push, 1)
struct CanSettings {
    bool enabled;
    bool listen_only;
    uint32_t bitrate;
};
#pragma pack(pop)

TaskHandle_t& create_task();

int send_can(uint8_t bus, can2040_msg& msg);
int receive(uint8_t bus, can2040_msg& msg);

int get_can_rx_buffered_frames(uint8_t bus);
int get_can_tx_buffered_frames(uint8_t bus);
uint32_t get_can_rx_overflow_count(uint8_t bus);
uint32_t get_can_tx_overflow_count(uint8_t bus);

bool get_statistics(uint8_t bus, can2040_stats& stats);


void set_num_busses(uint8_t num_busses);
uint8_t get_num_busses();

void enable(uint8_t bus, uint32_t bitrate);
void disable(uint8_t bus);
void set_bitrate(uint8_t bus, uint32_t bitrate);
bool is_enabled(uint8_t bus);
uint32_t get_bitrate(uint8_t bus);
bool is_listenonly(uint8_t bus);
void set_listenonly(uint8_t bus, bool listen_only);
void load_settings();
}