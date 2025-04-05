#include "handler.hpp"
#include <cstdint>

namespace gvret {
bool handler::process_byte(uint8_t byte) { return protocol_fsm.tick(byte); }

} // namespace gvret