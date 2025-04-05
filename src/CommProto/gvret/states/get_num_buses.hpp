#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <utility>
#include <cstdint>
#include <ostream>

namespace gvret::state {
class get_num_buses : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_num_buses(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_NUM_BUSES), out(host_out) {}

    constexpr static uint8_t NUM_BUSES = 1; // TODO: get from config/build-config
    Protocol enter() override {
        out << GET_COMMAND << GET_NUM_BUSES << NUM_BUSES << std::flush;
        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret