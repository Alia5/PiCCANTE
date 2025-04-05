#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>

namespace gvret::state {
class get_fd_settings : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_fd_settings() : fsm::state<uint8_t, Protocol, bool>(GET_FD_SETTINGS) {}
    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        // TODO: Implementation
        return {IDLE, false};
    }
};
} 