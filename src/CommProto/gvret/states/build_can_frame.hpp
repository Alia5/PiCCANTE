#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include "../../../CanBus/CanBus.hpp"
#include <cstdint>
#include <utility>

namespace gvret::state {

class build_can_frame : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit build_can_frame() : fsm::state<uint8_t, Protocol, bool>(SEND_CAN_FRAME) {}

    Protocol enter() override {
        step = 0;
        return SEND_CAN_FRAME;
    }
    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        switch (step) {
            case 0: // NOLINT
                frame.id = byte;
                break;
            case 1:
                frame.id |= byte << 8;
                break;
            case 2:
                frame.id |= byte << 16;
                break;
            case 3:
                frame.id |= byte << 24;
                // TODO: validate
                if (frame.id & 1 << 31) {
                    frame.id &= ~(1 << 31);
                    frame.id |= CAN2040_ID_EFF;
                }
                break;
            case 4:
                out_bus = byte & 3U;
                break;
            case 5: // NOLINT
                frame.dlc = byte & 0xF;
                if (frame.dlc > 8) {
                    frame.dlc = 8;
                }
                break;
            default:
                if (step < frame.dlc + 6) {
                    frame.data[step - 6] = byte;
                } else {
                    // TODO: out bus
                    send_can0(frame);
                    // state = IDLE;
                    // // this would be the checksum byte. Compute and compare.
                    // // temp8 = checksumCalc(buff, step);
                    // build_out_frame.rtr = 0;
                    // if (out_bus < NUM_BUSES)
                    //     canManager.sendFrame(canBuses[out_bus], build_out_frame);
                    return {IDLE, true};
                }
                break;
        }

        step++;
        return {SEND_CAN_FRAME, true};
    }

        protected:
    uint8_t step = 0;
    can2040_msg frame = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t out_bus = 0;
};
} // namespace gvret::state