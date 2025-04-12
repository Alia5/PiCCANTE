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
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif
// #ifndef CFG_TUSB_DEBUG
// #define CFG_TUSB_DEBUG (2)
// #endif
#define CFG_TUSB_OS OPT_OS_FREERTOS


#define CFG_TUD_ENABLED (1)

  // Legacy RHPORT configuration
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT (0)
#endif
  // end legacy RHPORT

  //------------------------
  // DEVICE CONFIGURATION //
  //------------------------

  // Enable 2 CDC classes
#define CFG_TUD_CDC ((piccanteNUM_CAN_BUSSES + 1))
  // Set CDC FIFO buffer sizes
#define CFG_TUD_CDC_RX_BUFSIZE (64)
#define CFG_TUD_CDC_TX_BUFSIZE (64)
#define CFG_TUD_CDC_EP_BUFSIZE (64)

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE (64)
#endif

#ifdef __cplusplus
  }
 #endif
 
 #endif /* _TUSB_CONFIG_H_ */