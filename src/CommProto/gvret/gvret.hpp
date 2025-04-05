#pragma once

#include <cstdint>
#include <span>
#include <ostream>
#include "../../CanBus/CanBus.hpp"


namespace gvret {

// My implementation of the gvret protocol (afaik) originally by Collin Kidder (Collin80)
// Based on the original (buggy...) code from:
// Protocol details (implementations can be found at:
// https://github.com/collin80/M2RET/blob/master/CommProtocol.txt
// esp32 arduino impl: https://github.com/collin80/ESP32RET/blob/master/gvret_comm.h
// SavvyCAN host impl:
// https://github.com/collin80/SavvyCAN/blob/master/connections/gvretserial.cpp
//
// check handler and states for more details
// binary protocol enum in proto.hpp
uint8_t check_sum(const std::span<uint8_t>& buff);

void comm_can_frame(uint busnumber, const can2040_msg& frame, std::ostream& out);

static constexpr auto SPEED_MASK = 0xFFFFFU;
static constexpr auto BUS_FLAG_MASK = 0x80000000UL;
static constexpr auto ENABLED_MASK = 0x40000000UL;
static constexpr auto LISTEN_ONLY_MASK = 0x20000000UL;
static constexpr auto MAX_BUS_SPEED = 1000000U;

} // namespace gvret