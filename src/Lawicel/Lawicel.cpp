#include "Lawicel.hpp"
#include <cstring>
#include <cctype>
#include <cstdlib>

#include <string>
#include <pico/time.h>

#include "tusb.h"

#include "../CanBus/CanBus.hpp"
#include "../Logger/Logger.hpp"

#include "../util/hexstr.hpp"
#include "../outstream/usb_cdc_stream.hpp"
#include "../fmt.hpp"


namespace Lawicel {

constexpr uint8_t NUM_BUSES = 1;

// TODO: Provide better options for this
static void writeUsbSerial(const std::string_view& s) {
    tud_cdc_n_write_str(1, s.data());
    tud_cdc_n_write_flush(1);
}
static void writeUsbSerial(const char* s) {
    tud_cdc_n_write_str(1, s);
    tud_cdc_n_write_flush(1);
}
static void writeUsbSerial(char s) {
    tud_cdc_n_write_char(1, s);
    tud_cdc_n_write_flush(1);
}

void Handler::handleShortCmd(char cmd) {
    Log::Debug("Lawicel Sjort Command:", cmd);


    switch (cmd) {
        case SHORT_CMD::OPEN: // LAWICEL open canbus port
            // CAN0.setListenOnlyMode(false);
            // CAN0.begin(settings.canSettings[0].nomSpeed, 255);
            // CAN0.enable();
            writeUsbSerial('\r'); // OK
            // SysSettings.lawicelMode = true;
            break;
        case SHORT_CMD::CLOSE: // LAWICEL close canbus port (First one)
            // CAN0.disable();
            writeUsbSerial('\r'); // OK
            break;
        case SHORT_CMD::OPEN_LISTEN_ONLY: // LAWICEL open canbus port in listen only
                                          // mode
            // CAN0.setListenOnlyMode(true);
            // CAN0.begin(settings.canSettings[0].nomSpeed, 255);
            // CAN0.enable();
            writeUsbSerial('\r'); // OK
            // SysSettings.lawicelMode = true;
            break;
        case SHORT_CMD::POLL_ONE: // LAWICEL - poll for one waiting frame. Or, just CR
            if (get_can_0_rx_buffered_frames() > 0) {
                poll_counter = 1; // AVAILABLE CAN_FRAMES;
            } else {
                writeUsbSerial('\r'); // OK
            }
            break;
        case SHORT_CMD::POLL_ALL: // LAWICEL - poll for all waiting frames - CR if no
            // frames
            // SysSettings.lawicelPollCounter = CAN0.available();
            poll_counter = get_can_0_rx_buffered_frames();
            if (poll_counter == 0) {
                writeUsbSerial('\r'); // OK
            }
            break;
        case SHORT_CMD::READ_STATUS_BITS: // LAWICEL - read status bits
            // Serial.print(
            //     "F00"); // bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error
            //     warning, 3
            //     =
            //             // Data overrun, 5= Error passive, 6 = Arb. Lost, 7 = Bus
            //             Error
            writeUsbSerial("F00"); // bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error
            writeUsbSerial('\r');  // OK
            break;
        case SHORT_CMD::GET_VERSION: // LAWICEL - get version number
            writeUsbSerial("V1013\n");
            break;
        case SHORT_CMD::GET_SERIAL: // LAWICEL - get serial number
            writeUsbSerial("PicoRET\n");
            // SysSettings.lawicelMode = true;
            break;
        case SHORT_CMD::SET_EXTENDED_MODE:
            extended_mode = !extended_mode;
            if (extended_mode) {
                writeUsbSerial("V2\n");
            } else {
                writeUsbSerial("LAWICEL\n");
            }
            break;
        case SHORT_CMD::LIST_SUPPORTED_BUSSES: // LAWICEL V2 - Output list of
            if (extended_mode) {
                for (int i = 0; i < NUM_BUSES; i++) {
                    printBusName(i);
                    writeUsbSerial('\n');
                }
            }
            break;
        case SHORT_CMD::FIRMWARE_UPGRADE:
            if (extended_mode) {
                // TODO: ???
            }
            break;
        default:
            Log::Error("Unknown command:", cmd);
            break;
    }
}

void Handler::handleLongCmd(const std::string_view& cmd) {
    can2040_msg out_frame;


    Log::Debug("Lawicel Long Command:", cmd);

    switch (cmd[0]) {
        {
            case LONG_CMD::TX_STANDARD_FRAME:
                out_frame.id = hex::parse(cmd.data() + 1, 3);
                out_frame.dlc = cmd.data()[4] - '0';
                if (out_frame.dlc < 0) {
                    out_frame.dlc = 0;
                } else if (out_frame.dlc > 8) {
                    out_frame.dlc = 8;
                }
                for (unsigned int i = 0; i < out_frame.dlc; i++) {
                    out_frame.data[i] = hex::parse(cmd.data() + 5 + i * 2, 2);
                }
                // CAN0.SendFrame(out_frame); // TODO:
                if (auto_poll) {
                    writeUsbSerial("z");
                }
                break;
            case LONG_CMD::TX_EXTENDED_FRAME:
                out_frame.id = hex::parse(cmd.data() + 1, 8);
                out_frame.dlc = cmd.data()[9] - '0';
                if (out_frame.dlc < 0) {
                    out_frame.dlc = 0;
                }
                if (out_frame.dlc > 8) {
                    out_frame.dlc = 8;
                }
                for (unsigned int data = 0; data < out_frame.dlc; data++) {
                    out_frame.data[data] = hex::parse(cmd.data() + 10 + (2 * data), 2);
                }
                // CAN0.sendFrame(out_frame); // TODO
                if (auto_poll) {
                    writeUsbSerial("Z");
                }
                break;
            case LONG_CMD::SET_SPEED_OR_PACKET:
                if (extended_mode) {
                    // Send packet to specified BUS
                    uint8_t bytes[8];
                    uint32_t id;
                    int num_bytes = 0;
                    // id = std::strtol(tokens[2], nullptr, 16);

                    id = 0x123; // TODO:

                    // for (int b = 0; b < 8; b++) {
                    //     if (tokens[3 + b][0] != 0) {
                    //         bytes[b] = strtol(tokens[3 + b], nullptr, 16);
                    //         num_bytes++;
                    //     } else {
                    //         break; // break for loop because we're obviously done.
                    //     }
                    // }
                    // if (!strcasecmp(tokens[1], "CAN0")) { // TODO
                    //     can2040_msg out_frame;
                    //     out_frame.id = id;
                    //     out_frame.dlc = num_bytes;
                    //     for (int b = 0; b < num_bytes; b++)
                    //         out_frame.data[b] = bytes[b];
                    //     // CAN0.sendFrame(outFrame); // TODO
                    // }
                    // if (!strcasecmp(tokens[1], "CAN1")) { // TODO
                    //     can2040_msg out_frame;
                    //     out_frame.id = id;
                    //     out_frame.dlc = num_bytes;
                    //     for (int b = 0; b < num_bytes; b++)
                    //         out_frame.data[b] = bytes[b];
                    //     // CAN1.sendFrame(outFrame); // TODO
                    // }
                } else {
                    // Set can speed...
                    // TODO:
                }
                break;
            case LONG_CMD::SET_CANBUS_BAUDRATE:
                // TODO;
                break;
            case LONG_CMD::SEND_RTR_FRAME:
                // deprecated and unsupported on can2040; ignore...
                break;
            case LONG_CMD::SET_RECEIVE_TRAFFIC:
                if (extended_mode) {
                    // TODO:
                    // if (!strcasecmp(tokens[1], "CAN0")) {
                    //     // SysSettings.lawicelBusReception[0] = true;
                    // }
                    // if (!strcasecmp(tokens[1], "CAN1")) {
                    //     // SysSettings.lawicelBusReception[1] = true;
                    // }
                } else {
                    // V1 send rtr frame, see above...
                }
                break;
            case LONG_CMD::SET_AUTOPOLL:
                auto_poll = cmd[1] == '1';
                break;
            case LONG_CMD::SET_FILTER_MODE:
                // unsupported.
                break;
            case LONG_CMD::SET_ACCEPTENCE_MASK:
                // unsupported.
                break;
            case LONG_CMD::SET_FILTER_MASK:
                if (extended_mode) {
                    // TODO: impl. softwareFilter
                } else {
                    // V1 set acceptance mode
                }
                break;
            case LONG_CMD::HALT_RECEPTION:
                if (extended_mode) {
                    // TODO: disable CAN
                }
                break;
            case LONG_CMD::SET_UART_SPEED:
                // USB_CDC IGNORES BAUD
                break;
            case LONG_CMD::TOGGLE_TIMESTAMP:
                time_stamping = cmd[1] == '1';
                break;
            case LONG_CMD::TOGGLE_AUTO_START:
                // TODO: Maybe?
                break;
            case LONG_CMD::CONFIGURE_BUS:
                if (extended_mode) {
                    // TODO:
                    int speed = 500000; //  std::atoi(tokens[2]);
                    // if (!strcasecmp(tokens[1], "CAN0")) {
                    //     // TODO: START CAN
                    //     // CAN0.begin(speed, 255);
                    // }
                    // if (!strcasecmp(tokens[1], "CAN1")) {
                    //     // TODO: START CAN
                    //     // CAN1.begin(speed, 255);
                    // }
                }
                break;
            default:
                Log::Error("Unknown command: ", cmd);
                break;
        }
    }
    writeUsbSerial('\r'); // OK
}

void Handler::sendFrameToBuffer(can2040_msg& frame, uint8_t bus) {
    uint8_t buff[40];
    uint8_t writtenBytes;
    uint8_t temp;
    uint32_t now = to_us_since_boot(get_absolute_time());
    uint32_t millis = to_ms_since_boot(get_absolute_time());

    if (poll_counter > 0) {
        poll_counter--;
    }

    if (extended_mode) {
        writeUsbSerial(std::to_string(now) + " - " + fmt::sprintf("%x", frame.id));
        if (false /*TODO: frame.extended*/) {
            writeUsbSerial(" X ");
        } else {
            writeUsbSerial(" S ");
        }
        printBusName(bus);
        for (int i = 0; i < frame.dlc; i++) {
            // TODO: Provide more elegant write
            writeUsbSerial(" ");
            writeUsbSerial(fmt::sprintf("%x", frame.data[i]));
        }
    } else {
        if (false /*TODO: frame.extended*/) {
            writeUsbSerial("T");
            writeUsbSerial(fmt::sprintf("%08x", frame.id));
        } else {
            writeUsbSerial("t");
            writeUsbSerial(fmt::sprintf("%03x", frame.id));
        }
        writeUsbSerial(std::to_string(frame.dlc));
        for (int i = 0; i < frame.dlc; i++) {
            writeUsbSerial(fmt::sprintf("%02x", frame.data[i]));
        }
        if (time_stamping) {
            uint16_t timestamp = (uint16_t)millis;
            writeUsbSerial(fmt::sprintf("%04x", timestamp));
        }
    }
    writeUsbSerial('\r'); // OK
}


void Handler::printBusName(int bus) {
    switch (bus) {
        case 0:
            writeUsbSerial("CAN0");
            break;
        case 1:
            writeUsbSerial("CAN1");
            break;
        default:
            // Serial.print("Unknown bus");
            Log::Warning("Unknown bus: ", bus);
            break;
    }
}


} // namespace Lawicel