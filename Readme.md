<img src="logo.svg" align="right" width="200"/>
<br />

[![Build Status](https://github.com/alia5/PiCCANTE/actions/workflows/build.yml/badge.svg)](https://github.com/alia5/PiCCANTE/actions/workflows/build.yml)
[![License: GPL-3.0](https://img.shields.io/github/license/alia5/PiCCANTE)](https://github.com/alia5/PiCCANTE/blob/main/LICENSE)
[![Release](https://img.shields.io/github/v/release/alia5/PiCCANTE?include_prereleases&sort=semver)](https://github.com/alia5/PiCCANTE/releases)
[![Issues](https://img.shields.io/github/issues/alia5/PiCCANTE)](https://github.com/alia5/PiCCANTE/issues)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/alia5/PiCCANTE/pulls)

# PiCCANTE 🌶️

**Pi**CCANTE **C**ar **C**ontroller **A**rea **N**etwork **T**ool for **E**xploration

PiCCANTE is a powerful hacking tool for accessing, exploring, and manipulating vehicle CAN bus networks, built on the affordable Raspberry Pi Pico platform. Designed for researchers, security professionals, and automotive enthusiasts looking to dive deep into vehicle systems without expensive proprietary equipment.

## 🚧 WORK IN PROGRESS 🚧

## ℹ️ About PiCCANTE

PiCCANTE is designed to be a dead-**simple**, dirt-**cheap**, and readily-**available** hardware platform for vehicle research and exploration.  
As cars become more and more computerized, manufacturers keep locking them behind proprietary tools and restrictive systems, even connect vehicles to the internet, yet they still rely on communication standards from the 1980s.  
Car hacking and independent research grows increasingly important to discover security vulnerabilities, reverse engineer vehicle systems, and empower owners with custom tools for diagnostics and tuning that support the Right to Repair movement.

The project leverages Raspberry Pi Pico (2 [W]) development boards, selected for their exceptional affordability, global availability, impressive specifications, and well-documented open-source SDK.

⚠️ Always exercise caution when connecting to vehicle systems.

### 🎯 Core Goals

<sub>Keep it D.I.R.T.Y 😉</sub>  

- **D**ead-Simple 🔨  
   - Simple enough for perfboard soldering with minimal components.
- **I**nexpensive 💰  
   - Minimal hardware requirements to keep costs dirt-cheap.  
   - Based on affordable, off-the-shelf components.
- **R**eadily-Available 🌎
   - Fully open-source hardware and software.  
   - Uses widely available Raspberry Pi Pico boards.
- **T**ool-Compatible 🔌
   - Seamless integration with existing CAN tools.  
   - Pre-built firmware releases for immediate use.
- **Y**our-First Choice 💡
   - Driverless across all major operating systems.  
   - Easy to use for beginners and experts alike.
   - 🔜 Documentation

### ✨ Features

- ✅ Up to 3× CAN 2.0B interfaces (2× on RP2040 (Pico[W]), 3× on RP2350 (Pico 2[W]))
- ✅ 1× USB-CDC PiCCANTE command + GVRET (binary only) interface
  - Compatible with [SavvyCAN](https://github.com/collin80/SavvyCAN) and other automotive tools
- ✅ Up to 3× USB-CDC SLCAN interfaces (dedicated to each CAN channel)
  - SocketCAN compatible via [can-utils (Linux)](https://github.com/linux-can/can-utils)
- ✅ LED status indicators
- ✅ Command line interface for configuration and diagnostics
- ✅ WiFi support (on Pico W models)
  - ✅ Create WiFi access point (wifi ap <ssid> <password> <channel>)
  - ✅ Connect to existing WiFi networks (wifi connect <ssid> <password>)
  - ✅ WiFi status and configuration commands
  - ✅ [Configurable] PiCCANTE configuration + GVRET as Telnet server
- 🔜 Web-based configuration interface (on Pico W models)
- 🔜 Bluetooth support (on Pico W models)
- 🔜 ELM327 emulator
- 🔜 (Software) CAN filters
- 🔜 MITM mode for advanced analysis / vehicle tuning
- 🔜 Data logging to SD card (maybe)
- 🔜 3D printable case designs for making PiCCANTE based OBD-II dongles

## 📋 Quick Start

### 🛒 What You'll Need

- 1× Raspberry Pi Pico (any model, RP2040 or RP2350)
  - **Note:** The Pico 2 W (RP2350) is recommended for full feature support, including WiFi and Bluetooth.
- 1-3× CAN transceivers (readily available SN65HVD23x or any other 3.3V compatible transceiver)
  - **Note:** Most readily available transceiver breakout-boards have a 120Ohm terminating Resistor on them, for research in a vehicle, you may need to remove it.
- USB cable
- Perfboard
  - [**OPTIONAL**] or custom PCB 🔜
- Soldering iron
- Basic soldering skills
- [**Optional**] Low dropout Buck-Converter module and OBD Plug for directly connecting to the vehicle's OBD-II port
- [**Optional**] 3D printed case for the Pico and transceivers

### 🚀 Basic Usage Examples

#### 🐧 Linux with can-utils (slcan)

```bash
# Set up SLCAN interface
sudo slcand -o -s6 -t hw -S 3000000 /dev/ttyACM1 can0
sudo ip link set up can0

# Monitor CAN traffic
candump can0
```

#### 🖥️ Cross Platform with SavvyCAN

1. Connect PiCCANTE to USB
2. Open SavvyCAN
3. Go to Connection ➡️ Open Connection Window ➡️ Add device Connection
4. Select "Serial Connection (GVRET)" and choose the appropriate COM port / TTY
5. Click "Create New Connection"

## 🔧 Hardware

Comprehensive documentation for self-built perfboard hardware is coming soon.  
Custom PCB designs may follow at a later date.

## 💾 Software

Pre-compiled binaries for all official Raspberry Pi Pico boards are available as CI action artifacts in the GitHub repository.

When connected via USB, PiCCANTE exposes **up to** 4× USB-CDC interfaces:

- 1× Combined PiCCANTE command + GVRET (binary) interface
  - Always the _first_ exposed CDC device.
  - Also exposed via telnet if enabled on WiFi enabled boards.
- **Up to** 3× SLCAN interfaces (dedicated to each CAN channel)

### ⌨️ PiCCANTE Commands

```
binary          - Toggle GVRET binary mode (binary <on|off>)
can_bitrate     - Change CAN bus bitrate (can_bitrate <bus> <bitrate>)
can_disable     - Disable CAN bus (can_disable <bus>)
can_enable      - Enable CAN bus with specified bitrate (can_enable <bus> <bitrate>)
can_status      - Show status of CAN buses
echo            - Toggle command echo (echo <on|off>)
help            - Display this help message
led_mode        - Set LED mode (led_mode <0-3>) 0=OFF, 1=Power, 2=CAN Activity
log_level       - Set log level (log_level <0-3>). 0=DEBUG, 1=INFO, 2=WARNING, 3=ERROR
reset           - Reset the system (reset)
save            - Save current settings to flash
set_num_busses  - Set number of CAN buses (can_num_busses [number])
settings        - Show current system settings
sys_stats       - Display system information and resource usage (sys_stats [cpu|heap|fs|tasks|uptime|adc|wifi])
telnet          - Enable or disable Telnet and set port (telnet enable|disable | telnet <port>)
wifi            - Manage WiFi settings (wifi info | wifi connect <ssid> <password> | wifi ap <ssid> <password> <channel> | wifi disable)
```

## ❓ Troubleshooting

- **Issue**: CAN bus not receiving data  
  *Solutions*:  
  - Verify CAN status with `can_status` command
  - Verify wiring and bitrate settings
  - Check for proper bus termination resistance (remove resistor on the transceiver if connected to an existing CAN-bus)

- **Issue**: LED not lighting up / blinking  
  *Solutions*:
  - Check USB connection and power
  - Check led_mode command on the first USB CDC device

## 🛠️ Development

PiCCANTE is built using the Raspberry Pi Pico SDK and follows standard Pico development practices. This section covers how to set up your development environment, build the project, and extend its functionality.

### 🧰 Prerequisites

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [CMake](https://cmake.org/) (3.13 or newer) [Windows: bundled with PicoSDK]
- C/C++ compiler (GCC ARM) [Windows: bundled with PicoSDK]
- [Visual Studio Code](https://code.visualstudio.com/) with the [Pico extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) (recommended)

### 🔄 Environment Setup

1. **Install the Pico SDK**:

    Follow the instructions in the [Pico SDK documentation](https://github.com/raspberrypi/pico-sdk) to install the SDK

2. **Clone PiCCANTE**:

    ```bash
    git clone <https://github.com/username/PiCCANTE.git>
    cd PiCCANTE
    git submodule update --init --recursive
    ```

### 🏗️ Building

#### 💻 Command Line

1. **Configure with CMake**:

   ```bash
   cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   ```

2. **Build the project**:

   ```bash
   cmake --build build --config Release
   ```

#### 🖥️ Using VS Code

1. Open the PiCCANTE folder in VS Code
2. Press F1 and select "CMake: Configure"
3. Press F7 to build the project
4. To flash, connect a Pico in BOOTSEL mode and press F5

## 👥 Contributing

Contributions to PiCCANTE are welcome! Whether it's bug reports, feature requests, documentation improvements, or code contributions, your help makes this project better.

- **Bug Reports**: Open an issue describing the problem and steps to reproduce it
- **Feature Requests**: Open an issue describing the desired functionality
- **Code Contributions**: Fork the repository, make your changes, and submit a pull request

### Extending PiCCANTE

PiCCANTE is designed to be modular and extensible. Here are some key areas where you might want to extend functionality:

- **Adding new commands**: Extend the command interpreter to add custom commands.
- **Implementing new protocols**: Add new protocol handlers in their respective directories.
- **Car-specific tools**: Fork PiCCANTE to create a car-specific tool and diagnostics dongle.
- **Custom data logging**: Implement custom data logging to SD card to monitor your vehicle on a track day.

## 📄 License

```
PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
Copyright (C) 2025 Peter Repukat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```
