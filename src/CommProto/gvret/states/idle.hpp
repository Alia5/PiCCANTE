#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>
namespace gvret::state {

class idle : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit idle() : fsm::state<uint8_t, Protocol, bool>(IDLE) {}

    std::pair<Protocol, bool> tick(uint8_t& byte) override {
        if (byte == GET_COMMAND) {
            return {GET_COMMAND, true};
        }
        if (byte == ENABLE_BINARY_MODE) {
            return {IDLE, true};
        }
        return {IDLE, false};
    }
};

}