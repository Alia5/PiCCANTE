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
// NOLINTBEGIN
#include "idle.hpp"
#include "build_can_frame.hpp"
#include "echo_can_frame.hpp"
#include "get_a_inputs.hpp"
#include "get_canbus_params_1_2.hpp"
#include "get_canbus_params_3_4_5.hpp"
#include "get_d_inputs.hpp"
#include "get_dev_info.hpp"
#include "get_num_buses.hpp"
#include "get_command.hpp"
#include "set_d_out.hpp"
#include "set_sw_mode.hpp"
#include "set_systype.hpp"
#include "set_canbus_params_3_4_5.hpp"
#include "time_sync.hpp"
#include "get_fd_settings.hpp"
#include "setup_fd.hpp"
#include "setup_canbus_1_2.hpp"
#include "keepalive.hpp"
#include "send_fd_frame.hpp"
// NOLINTEND