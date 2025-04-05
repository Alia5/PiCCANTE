#pragma once
#include <cstdint>
namespace gvret {

enum Protocol : uint8_t {
    // CMDS
    SEND_CAN_FRAME = 0,
    TIME_SYNC = 1,
    GET_D_INPUTS = 2, // get Digial Inputs
    GET_A_INPUTS = 3, // Get Analog Inputs
    SET_D_OUT = 4,    // Set Digital Outputs
    SETUP_CANBUS_1_2 = 5,
    GET_CANBUS_PARAMS_1_2 = 6,
    GET_DEV_INFO = 7,
    SET_SW_MODE = 8,
    KEEPALIVE = 9,
    SET_SYSTYPE = 10,
    ECHO_CAN_FRAME = 11,
    GET_NUM_BUSES = 12,
    GET_CANBUS_PARAMS_3_4_5 = 13,
    SET_CANBUS_PARAMS_3_4_5 = 14,
    // Seems not implemented anywhere
    // We don't have support for CAN-FD anyway on default config.
    SEND_FD_FRAME = 20,
    SETUP_FD = 21,
    GET_FD_SETTINGS = 22,
    // BASE
    GET_COMMAND = 0xF1,
    ENABLE_BINARY_MODE = 0xE7,
    IDLE = 0xFF
};
}