#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <ostream>
#include <utility>
#include "../../../util/bin.hpp"

#include "../../../Logger/Logger.hpp"
namespace gvret::state {
class get_canbus_params_1_2 : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_canbus_params_1_2(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_CANBUS_PARAMS_1_2), out(host_out) {}

    Protocol enter() override {
        Log::debug << "get_canbus_params_1_2\n" << std::flush;
        // TODO: implement, for now just tx 500000kbits, enabled, NOT listen only
        const uint32_t speed = 500000;
        const uint8_t flags = 0x01 + (0x00 << 4); // enabled + listen only
        out << GET_COMMAND << GET_CANBUS_PARAMS_1_2 << flags << piccante::bin(speed)
            << flags << piccante::bin(speed) << std::flush;

        // TODO: doesn't TX a checksum byte ¯\_(ツ)_/¯

        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};
} // namespace gvret::state