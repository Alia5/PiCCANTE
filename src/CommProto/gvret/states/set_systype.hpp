#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <utility>
#include <cstdint>
namespace gvret::state {
class set_systype : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit set_systype() : fsm::state<uint8_t, Protocol, bool>(SET_SYSTYPE) {}

    // NOLINTNEXTLINE
    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        // Not supported, just go back to IDLE

        return {IDLE, false};
    }
};
} // namespace gvret