#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <utility>
#include <cstdint>

namespace gvret::state {
class set_sw_mode : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit set_sw_mode() : fsm::state<uint8_t, Protocol, bool>(SET_SW_MODE) {}
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        // Not supported, just go back to IDLE
        return {IDLE, false};
    }
};
} // namespace gvret