#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"

namespace gvret {
namespace state {
class set_d_out : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit set_d_out() : fsm::state<uint8_t, Protocol, bool>(SET_D_OUT) {}
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        // TODO: Set Dig Outputs once supported.
        // until now, just set the byte as processed.
        // for (int c = 0; c < 8; c++) {
        //     if (byte & (1 << c)) {
        //         // setOutput(c, true);
        //     } else {
        //         // setOutput(c, false);
        //     }
        // }
        return {IDLE, true};
    }
};
} // namespace state
} // namespace gvret