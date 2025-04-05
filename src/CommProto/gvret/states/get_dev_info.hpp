#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <cstdint>
#include <utility>
#include <ostream>
#include "../../../util/bin.hpp"

namespace gvret::state {
class get_dev_info : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit get_dev_info(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(GET_DEV_INFO), out(host_out) {}

    Protocol enter() override {
        // TODO: build number
        uint16_t build_number = 0x0001;
        out << GET_COMMAND << GET_DEV_INFO
            << piccante::bin(build_number)
            // TODO: Wtf is 0x20?!
            << (uint8_t(0x20))
            // TODO: WTF Is 0?
            << (uint8_t(0))
            // single wire mode?
            << (uint8_t(0)) << std::flush;

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