#include "usb_cdc_stream.hpp"
#include <cstddef>

namespace usb_cdc {
void USB_CDC_Sink::write(const char* v, std::size_t s) { tud_cdc_n_write(itf, v, s); }
void USB_CDC_Sink::flush() { tud_cdc_n_write_flush(itf); }
} // namespace usb_cdc
