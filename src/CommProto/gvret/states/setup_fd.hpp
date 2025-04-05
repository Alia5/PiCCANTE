#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>
namespace gvret::state {
class setup_fd : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit setup_fd() : fsm::state<uint8_t, Protocol, bool>(SETUP_FD) {}
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        // TODO: Implementation
        return {IDLE, false};
    }
};
} // namespace gvret::state
