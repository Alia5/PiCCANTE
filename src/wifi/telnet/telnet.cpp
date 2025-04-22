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
#include "telnet_server.hpp"
#include "SysShell/settings.hpp"
#include "Logger/Logger.hpp"

namespace piccante::wifi::telnet {

namespace {
std::unique_ptr<server> telnet_server = nullptr;
} // namespace

class telnet_sink : public out::base_sink {
        public:
    void write(const char* data, std::size_t len) override {
        if (!telnet_server || !telnet_server->is_running()) {
            return;
        }

        if (xSemaphoreTake(telnet_server->get_clients_mutex(), pdMS_TO_TICKS(100)) !=
            pdTRUE) {
            return;
        }

        for (const auto& client : telnet_server->get_clients()) {
            if (client.sink) {
                client.sink->write(data, len);
            }
        }

        xSemaphoreGive(telnet_server->get_clients_mutex());
    }

    void flush() override {
        if (!telnet_server || !telnet_server->is_running()) {
            return;
        }

        if (xSemaphoreTake(telnet_server->get_clients_mutex(), pdMS_TO_TICKS(100)) !=
            pdTRUE) {
            return;
        }

        for (const auto& client : telnet_server->get_clients()) {
            if (client.sink) {
                client.sink->flush();
            }
        }

        xSemaphoreGive(telnet_server->get_clients_mutex());
    }
};

namespace {
telnet_sink all_clients_sink;
bool telnet_sink_added = false;
out::sink_mux sink_muxxer{};
} // namespace

bool initialize() {
    if (!sys::settings::telnet_enabled()) {
        Log::debug << "Telnet server is disabled in settings\n";
        return true;
    }

    if (!telnet_server) {
        telnet_server = std::make_unique<server>("Telnet PiCCANTE",
                                                 sys::settings::get_telnet_port(),
                                                 "PiCCANTE + GVRET Telnet Server\r\n");
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

out::base_sink& get_sink() { return all_clients_sink; }


out::sink_mux& mux_sink(std::initializer_list<out::base_sink*> sinks) {
    if (!telnet_sink_added) {
        sink_muxxer.add_sink(&all_clients_sink);
        telnet_sink_added = true;
    }

    for (auto* sink : sinks) {
        sink_muxxer.add_sink(sink);
    }

    return sink_muxxer;
}

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