#pragma once

#include <string_view>
#include <cstdint>
#include <functional>
#include "outstream/stream.hpp"

struct can2040_msg;

namespace piccante::lawicel {


enum SHORT_CMD {
    OPEN = 'O',
    CLOSE = 'C',
    OPEN_LISTEN_ONLY = 'L',
    POLL_ONE = 'P',
    POLL_ALL = 'A',
    READ_STATUS_BITS = 'F',
    GET_VERSION = 'V',
    GET_SERIAL = 'N',
    SET_EXTENDED_MODE = 'x',
    LIST_SUPPORTED_BUSSES = 'B',
    FIRMWARE_UPGRADE = 'X',
    //
};

enum LONG_CMD {
    TX_STANDARD_FRAME = 't',
    TX_EXTENDED_FRAME = 'T',
    SET_SPEED_OR_PACKET = 'S',
    SET_CANBUS_BAUDRATE = 's',
    SEND_RTR_FRAME = 'r',
    SET_RECEIVE_TRAFFIC = 'R',
    SET_AUTOPOLL = 'A',
    SET_FILTER_MODE = 'W',
    SET_ACCEPTENCE_MASK = 'm',
    SET_FILTER_MASK = 'M',
    HALT_RECEPTION = 'H',
    SET_UART_SPEED = 'U',
    TOGGLE_TIMESTAMP = 'Z',
    TOGGLE_AUTO_START = 'Q',
    CONFIGURE_BUS = 'C',
};

class handler {
        public:
    explicit handler(out::stream& out_stream, uint8_t bus_num = 0)
        : host_out(out_stream), bus(bus_num) {};
    virtual ~handler() = default;


    void handleShortCmd(char cmd);
    void handleLongCmd(const std::string_view& cmd);
    void sendFrameToBuffer(can2040_msg& frame, uint8_t bus);
    void handleCmd(const std::string_view& cmd);


        private:
    std::reference_wrapper<out::stream> host_out;
    uint8_t bus = 0;

    bool extended_mode = false;
    bool auto_poll = false;
    bool time_stamping = false;
    uint32_t poll_counter = 0;

    void printBusName(int bus) const;
};

} // namespace piccante::lawicel