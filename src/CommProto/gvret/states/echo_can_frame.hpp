#pragma once

#include "../gvret.hpp"
#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include "../../../CanBus/CanBus.hpp"
#include <cstdint>
#include <utility>
#include <ostream>

namespace gvret::state {
// Seems not to be implemented in savvyCan...
// It's the same as send_can_frame, but instead of sending the frame, it just echoes it...
class echo_can_frame : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit echo_can_frame(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(ECHO_CAN_FRAME), out(host_out) {}


    Protocol enter() override {
        step = 0;
        return ECHO_CAN_FRAME;
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
                    comm_can_frame(out_bus, frame, out);
                    return {IDLE, true};
                }
                break;
        }

        step++;
        return {ECHO_CAN_FRAME, true};
    }

        protected:
    uint8_t step = 0;
    can2040_msg frame = {0};
    uint8_t out_bus = 0;
    std::ostream& out;
};
} // namespace gvret::state