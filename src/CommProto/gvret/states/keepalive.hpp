#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>
#include <ostream>
#include "../../../util/bin.hpp"

namespace gvret::state {
class keepalive : public fsm::state<uint8_t, Protocol, bool> {
    static constexpr uint16_t KEEPALIVE_PAYLOAD = 0xDEAD;

        public:
    explicit keepalive(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(KEEPALIVE), out(host_out) {}

    Protocol enter() override {
        out << GET_COMMAND << KEEPALIVE << piccante::bin_be(KEEPALIVE_PAYLOAD)
            << std::flush; // NOLINT
        return IDLE;
    }
    std::pair<Protocol, bool> tick(uint8_t& byte) override { return {IDLE, false}; }

        private:
    std::ostream& out;
};
} // namespace gvret