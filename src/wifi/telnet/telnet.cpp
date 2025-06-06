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
#include "telnet.hpp"
#include "outstream/stream.hpp"
#include "telnet_server.hpp"
#include "SysShell/settings.hpp"
#include "Logger/Logger.hpp"

namespace piccante::wifi::telnet {

namespace {
std::unique_ptr<server> telnet_server = nullptr;
out::sink_mux telnet_sink;
}
bool initialize() {
    if (!sys::settings::telnet_enabled()) {
        Log::debug << "Telnet server is disabled in settings\n";
        return true;
    }

    if (!telnet_server) {
        telnet_server = std::make_unique<server>("Telnet PiCCANTE",
                                                 sys::settings::get_telnet_port(),
                                                 "PiCCANTE + GVRET Telnet Server\r\n");
        telnet_sink.add_sink(&telnet_server->get_all_sink());
    }

    bool success = telnet_server->start();
    if (!success) {
        Log::error << "Failed to start telnet server\n";
    }

    return success;
}

void stop() {
    if (telnet_server) {
        telnet_server->stop();
        telnet_server.reset();
        Log::info << "Telnet server stopped\n";
    }
}

bool reconfigure() {
    if (!telnet_server) {
        if (sys::settings::telnet_enabled()) {
            return initialize();
        }
        return true;
    }

    if (!sys::settings::telnet_enabled()) {
        stop();
        return true;
    }

    if (telnet_server->is_running()) {
        telnet_server->stop();
    }
    return telnet_server->reconfigure(sys::settings::get_telnet_port());
}

bool is_running() { return telnet_server && telnet_server->is_running(); }

QueueHandle_t get_rx_queue() {
    if (telnet_server) {
        return telnet_server->get_rx_queue();
    }
    return nullptr;
}

out::base_sink& get_sink() { return telnet_sink; }

bool enable() {
    sys::settings::set_telnet_enabled(true);
    return reconfigure();
}

void disable() {
    sys::settings::set_telnet_enabled(false);
    stop();
}

bool set_port(uint16_t port) {
    if (port < 1 || port > 65535) {
        return false;
    }

    sys::settings::set_telnet_port(port);
    return reconfigure();
}

} // namespace piccante::wifi::telnet