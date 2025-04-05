#pragma once

#include "../../../StateMachine/state.hpp"
#include "../proto.hpp"
#include <FreeRTOS.h>
#include <ostream>
#include "../../../util/bin.hpp"

namespace gvret::state {
class time_sync : public fsm::state<uint8_t, Protocol, bool> {
        public:
    explicit time_sync(std::ostream& host_out)
        : fsm::state<uint8_t, Protocol, bool>(TIME_SYNC), out(host_out) {}
    Protocol enter() override {
        Log::info << "Time Sync\n" << std::flush;
        // no state required, send response and return to IDLE
        // TODO: can only cause trouble to output MICROS as 4 byte value, but oh well...
        // I don't think it is implemented anyway anywhere, just use millis for now...
        uint32_t time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        out << GET_COMMAND
            << TIME_SYNC
            // IS this really output in binary?
            << piccante::bin(time) << std::flush;
        return IDLE;
    }
    std::pair<Protocol, bool> tick([[maybe_unused]] uint8_t& byte) override {
        return {IDLE, false};
    }

        private:
    std::ostream& out;
};


} // namespace gvret