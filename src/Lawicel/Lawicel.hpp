#pragma once

#include <string_view>
#include <cstdint>

struct can2040_msg;

namespace Lawicel {


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
    TODO_EXTENDED_MODE = 'X',
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

class Handler {
        public:
    Handler() = default;
    void handleShortCmd(char cmd);
    void handleLongCmd(const std::string_view& cmd);
    void sendFrameToBuffer(can2040_msg& frame, uint8_t bus);

        private:
    char tokens[14][10];
    bool extended_mode = false;
    bool auto_poll = false;
    bool time_stamping = false;
    uint32_t poll_counter = 0;

    void tokenizeCmdString(char* cmd);
    void uppercaseToken(char* token);
    void printBusName(int bus);
    bool parseLawicelCANCmd(can2040_msg& frame);
};

} // namespace Lawicel