
#include "gvret.hpp"
#include <cstdint>
#include <span>
#include <numeric>
#include <ostream>
#include "../../CanBus/CanBus.hpp"
#include "proto.hpp"
#include <FreeRTOS.h>
#include "../../util/bin.hpp"
namespace gvret {

uint8_t check_sum(const std::span<uint8_t>& buff) {
    return std::accumulate(buff.begin(), buff.end(), 0);
}

void comm_can_frame(uint busnumber, const can2040_msg& frame, std::ostream& out) {
    // TODO: can only cause trouble to output MICROS as 4 byte value, but oh well...
    // I don't think it is implemented anyway anywhere, just use millis for now...
    uint32_t time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    auto id = frame.id;
    // remove flags from id copy
    id &= ~(CAN2040_ID_EFF | CAN2040_ID_RTR);


    out << GET_COMMAND << SEND_CAN_FRAME << piccante::bin(time) << piccante::bin(id)
        << static_cast<uint8_t>(frame.dlc + uint8_t(busnumber << 4));
    for (uint8_t i = 0; i < frame.dlc; i++) {
        out << frame.data[i];
    }
    // TODO: Checksum, seems not to be implemented anyway...
    out << 0 << std::flush;
}

} // namespace gvret