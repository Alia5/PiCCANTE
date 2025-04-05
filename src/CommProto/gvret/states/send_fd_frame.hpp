#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>
namespace gvret::state {
class send_fd_frame : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit send_fd_frame() : fsm::state<uint8_t, Protocol, bool>(SEND_FD_FRAME) {}
    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        // TODO: Implementation
        return {IDLE, false};
    }
};
} // namespace gvret::state