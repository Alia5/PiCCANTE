/*
 * PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
 * Copyright (C) 2025 Peter Repukat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <bsp/board_api.h>
#include "common/tusb_types.h"
#include "config/tusb_config.h"
#include "device/usbd.h"


#define CDC_RASPBERRY_VID 0x2E8A
#if PICO_RP2350
// For RP2350 boards
#define CDC_RASPBERRY_PID 0x0009
#else
// For RP2040 boards
#define CDC_RASPBERRY_PID 0x000A
#endif
// set USB 2.0
#define CDC_BCD 0x0200

// defines a descriptor that will be communicated to the host
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = CDC_BCD,

    .bDeviceClass = TUSB_CLASS_MISC,         // CDC is a subclass of misc
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, // CDC uses common subclass
    .bDeviceProtocol = MISC_PROTOCOL_IAD,    // CDC uses IAD

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, // 64 bytes

    .idVendor = CDC_RASPBERRY_VID,
    .idProduct = CDC_RASPBERRY_PID,
    .bcdDevice = 0x0100, // Device release number

    .iManufacturer = 0x01, // Index of manufacturer string
    .iProduct = 0x02,      // Index of product string
    .iSerialNumber = 0x03, // Index of serial number string

    .bNumConfigurations = 0x01 // 1 configuration
};

// called when host requests to get device descriptor
uint8_t const* tud_descriptor_device_cb(void);

enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
#if CFG_TUD_CDC > 1
    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,
#endif
#if CFG_TUD_CDC > 2
    ITF_NUM_CDC_2,
    ITF_NUM_CDC_2_DATA,
#endif
#if CFG_TUD_CDC > 3
    ITF_NUM_CDC_3,
    ITF_NUM_CDC_3_DATA,
#endif
    ITF_NUM_TOTAL
};

// total length of configuration descriptor
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

// define endpoint numbers
#define EPNUM_CDC_0_NOTIF 0x81 // notification endpoint for CDC 0
#define EPNUM_CDC_0_OUT 0x02   // out endpoint for CDC 0
#define EPNUM_CDC_0_IN 0x82    // in endpoint for CDC 0

#define EPNUM_CDC_1_NOTIF 0x84 // notification endpoint for CDC 1
#define EPNUM_CDC_1_OUT 0x05   // out endpoint for CDC 1
#define EPNUM_CDC_1_IN 0x85    // in endpoint for CDC 1

#define EPNUM_CDC_2_NOTIF 0x88 // notification endpoint for CDC 2
#define EPNUM_CDC_2_OUT 0x08   // out endpoint for CDC 2
#define EPNUM_CDC_2_IN 0x89    // in endpoint for CDC 2

#define EPNUM_CDC_3_NOTIF 0x8C // notification endpoint for CDC 3
#define EPNUM_CDC_3_OUT 0x0B   // out endpoint for CDC 3
#define EPNUM_CDC_3_IN 0x8D    // in endpoint for CDC 3

// configure descriptor (for 2 CDC interfaces)
uint8_t const desc_fs_configuration[] = {
    // config descriptor | how much power in mA, count of interfaces, ...
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT,
                       EPNUM_CDC_0_IN, CFG_TUD_ENDPOINT0_SIZE),

#if CFG_TUD_CDC > 1
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 5, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT,
                       EPNUM_CDC_1_IN, CFG_TUD_ENDPOINT0_SIZE),
#endif

#if CFG_TUD_CDC > 2
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, 6, EPNUM_CDC_2_NOTIF, 8, EPNUM_CDC_2_OUT,
                       EPNUM_CDC_2_IN, CFG_TUD_ENDPOINT0_SIZE),
#endif

#if CFG_TUD_CDC > 3
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_3, 7, EPNUM_CDC_3_NOTIF, 8, EPNUM_CDC_3_OUT,
                       EPNUM_CDC_3_IN, CFG_TUD_ENDPOINT0_SIZE),
#endif

};

// called when host requests to get configuration descriptor
uint8_t const* tud_descriptor_configuration_cb(uint8_t index);

// more device descriptor this time the qualifier
tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = CDC_BCD,

    .bDeviceClass = TUSB_CLASS_CDC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00};

// called when host requests to get device qualifier descriptor
uint8_t const* tud_descriptor_device_qualifier_cb(void);

// String descriptors referenced with .i... in the descriptor tables

enum {
    STRID_LANGID = 0,   // 0: supported language ID
    STRID_MANUFACTURER, // 1: Manufacturer
    STRID_PRODUCT,      // 2: Product
    STRID_SERIAL,       // 3: Serials
    STRID_CDC_0,        // 4: CDC Interface 0
    STRID_CDC_1,        // 5: CDC Interface 1
    STRID_CDC_2,        // 6: CDC Interface 2
    STRID_CDC_3,        // 7: CDC Interface 3
};

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for
// transfer to complete Configuration descriptor in the other speed e.g if high speed then
// this is for full speed and vice versa
uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index) {
    (void)index; // for multiple configurations

    // if link speed is high return fullspeed config, and vice versa
    return desc_fs_configuration;
}

// array of pointer to string descriptors
char const* string_desc_arr[] = {
    // switched because board is little endian
    (const char[]){0x09, 0x04}, // 0: supported language is English (0x0409)
    "Alia5",                    // 1: Manufacturer
    "PiCCANTE",                 // 2: Product
    NULL,                       // 3: Serials (null so it uses unique ID if available)
    "PiCCANTE + GVRET",         // 4: CDC Interface 0
#if CFG_TUD_CDC > 1
    "PiCCANTE SLCAN0", // 5: CDC Interface 1
#endif

#if CFG_TUD_CDC > 2
    "PiCCANTE SLCAN1", // 6: CDC Interface 2
#endif

#if CFG_TUD_CDC > 3
    "PiCCANTE SLCAN2", // 7: CDC Interface 3
#endif
};

// buffer to hold the string descriptor during the request | plus 1 for the null
// terminator
static uint16_t _desc_str[32 + 1];

// called when host request to get string descriptor
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);

// --------------------------------------------------------------------+
// IMPLEMENTATION
// --------------------------------------------------------------------+

uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&desc_device; }

uint8_t const* tud_descriptor_device_qualifier_cb(void) {
    return (uint8_t const*)&desc_device_qualifier;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    // avoid unused parameter warning and keep function signature consistent
    (void)index;

    return desc_fs_configuration;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    // TODO: check lang id
    (void)langid;
    size_t char_count;

    // Determine which string descriptor to return
    switch (index) {
        case STRID_LANGID:
            memcpy(&_desc_str[1], string_desc_arr[STRID_LANGID], 2);
            char_count = 1;
            break;

        case STRID_SERIAL:
            // try to read the serial from the board
            char_count = board_usb_get_serial(_desc_str + 1, 32);
            break;

        default:
            // COPYRIGHT NOTE: Based on TinyUSB example
            // Windows wants utf16le

            // Determine which string descriptor to return
            if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
                return NULL;
            }

            // Copy string descriptor into _desc_str
            const char* str = string_desc_arr[index];

            char_count = strlen(str);
            size_t const max_count =
                sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
            // Cap at max char
            if (char_count > max_count) {
                char_count = max_count;
            }

            // Convert ASCII string into UTF-16
            for (size_t i = 0; i < char_count; i++) {
                _desc_str[1 + i] = str[i];
            }
            break;
    }

    // First byte is the length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (char_count * 2 + 2));

    return _desc_str;
}