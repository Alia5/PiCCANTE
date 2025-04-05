#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <ostream>
#include <utility>
#include "../../../util/bin.hpp"

namespace gvret::state {
class get_canbus_params_3_4_5 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_canbus_params_3_4_5(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_CANBUS_PARAMS_3_4_5), out(host_out) {}

    // TODO: doesn't seem to be implemented anywhere, just guessing response based on
    // other
    Protocol enter() override {
        // TODO: implement, for now just tx 500000kbits, enabled, NOT listen only
        const uint32_t speed = 500000;
        const uint8_t flags = 0x01 + (0x00 << 4); // enabled + listen only

        out << GET_COMMAND << GET_CANBUS_PARAMS_3_4_5 << flags << piccante::bin(speed)
            << flags << piccante::bin(speed) << flags << piccante::bin(speed)
            << std::flush;

        // TODO: doesn't TX a checksum byte ¯\_(ツ)_/¯

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret