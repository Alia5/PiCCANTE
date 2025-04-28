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
#include "telnet_server.hpp"
#include "FreeRTOSConfig.h"
#include "Logger/Logger.hpp"

#include <cstdint>
#include <projdefs.h>
#include <lwip/inet.h>
#include <lwip/def.h>
#include <cerrno>
#include <memory>
#include <array>
#include <algorithm>
#include <cstring>
#include <sys/select.h>
#include <sys/_timeval.h>
#include <utility>

#include "lwip/sockets.h"

namespace piccante::wifi::telnet {

// Telnet protocol constants
constexpr uint8_t IAC = 255; // Interpret As Command
constexpr uint8_t WILL = 251;
constexpr uint8_t WONT = 252;
constexpr uint8_t DO = 253;
constexpr uint8_t DONT = 254;
constexpr uint8_t ECHO = 1;
constexpr uint8_t SGA = 3; // Suppress Go Ahead

server::server(std::string_view name, uint16_t port, std::string_view welcome_message)
    : name(name), port(port), welcome_message(welcome_message), running(false),
      server_socket(-1), server_task_handle(nullptr), clients_mutex(nullptr) {
    clients_mutex = xSemaphoreCreateMutex();
    rx_byte_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(uint8_t));
}

server::~server() {
    stop();
    if (clients_mutex) {
        vSemaphoreDelete(clients_mutex);
        clients_mutex = nullptr;
    }
    if (rx_byte_queue) {
        vQueueDelete(rx_byte_queue);
        rx_byte_queue = nullptr;
    }
}

bool server::start() {
    if (running) {
        return true; // Already running
    }

    BaseType_t task_created = xTaskCreate(server_task,
                                          name.c_str(),
                                          configMINIMAL_STACK_SIZE * 2,
                                          this,
                                          configMAX_PRIORITIES - 8,
                                          &server_task_handle);

    if (task_created != pdPASS) {
        Log::error << "Failed to create telnet server task\n";
        return false;
    }

    running = true;
    Log::info << "Telnet server started on port " << port << "\n";
    return true;
}

void server::stop() {
    if (!running) {
        return;
    }

    running = false;

    close_socket();
    if (server_task_handle) {
        for (int i = 0; i < 10 && eTaskGetState(server_task_handle) != eDeleted; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (eTaskGetState(server_task_handle) != eDeleted) {
            vTaskDelete(server_task_handle);
        }

        server_task_handle = nullptr;
    }

    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (auto& client : clients) {
            all_clients_sink.remove_sink(client.sink.get());
            if (client.socket >= 0) {
                lwip_close(client.socket);
                client.socket = -1;
            }

            if (client.task_handle) {
                vTaskDelete(client.task_handle);
                client.task_handle = nullptr;
            }
        }


        clients.clear();
        xSemaphoreGive(clients_mutex);
    }

    Log::info << "Telnet server stopped\n";
}

bool server::is_running() const { return running; }

bool server::reconfigure(uint16_t new_port) {
    bool was_running = running;

    if (was_running) {
        stop();
    }

    port = new_port;

    return start();
}

QueueHandle_t server::get_rx_queue() const { return rx_byte_queue; }

bool server::create_socket() {
    close_socket();

    server_socket = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0) {
        Log::error << "Failed to create telnet server socket\n";
        return false;
    }

    int opt = 1;
    if (lwip_setsockopt(server_socket, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
        Log::error << "Failed to set TCP_NODELAY on telnet server socket\n";
        close_socket();
        return false;
    }

    opt = 1;
    if (lwip_ioctl(server_socket, FIONBIO, &opt) < 0) {
        Log::error << "Failed to set telnet server socket to non-blocking\n";
        close_socket();
        return false;
    }

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (lwip_bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) <
        0) {
        Log::error << "Failed to bind telnet server socket\n";
        close_socket();
        return false;
    }

    if (lwip_listen(server_socket, 5) < 0) {
        Log::error << "Failed to listen on telnet server socket\n";
        close_socket();
        return false;
    }

    return true;
}

void server::close_socket() {
    if (server_socket >= 0) {
        lwip_close(server_socket);
        server_socket = -1;
    }
}

void server::server_task(void* params) {
    auto* self = static_cast<server*>(params);

    if (!self->create_socket()) {
        self->running = false;
        vTaskDelete(nullptr);
        return;
    }

    self->accept_connections();

    self->close_socket();
    self->running = false;
    vTaskDelete(nullptr);
}

void server::accept_connections() {
    fd_set read_fds;
    struct timeval tv;

    while (running) {
        FD_ZERO(&read_fds);
        FD_SET(server_socket, &read_fds);

        tv.tv_sec = 0;
        tv.tv_usec = 500000;

        int res = lwip_select(server_socket + 1, &read_fds, nullptr, nullptr, &tv);

        if (res < 0) {
            Log::error << "Select error in telnet server: " << errno << "\n";
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        } else if (res == 0) {
            continue;
        }

        if (!FD_ISSET(server_socket, &read_fds)) {
            continue;
        }

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int client_socket =
            lwip_accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_len);

        if (client_socket < 0) {
            if (errno != EWOULDBLOCK) {
                Log::error << "Failed to accept telnet connection: " << errno << "\n";
            }
            continue;
        }

        Log::info << "Telnet client connected from "
                  << (client_addr.sin_addr.s_addr & 0xFF) << "."
                  << ((client_addr.sin_addr.s_addr >> 8) & 0xFF) << "."
                  << ((client_addr.sin_addr.s_addr >> 16) & 0xFF) << "."
                  << ((client_addr.sin_addr.s_addr >> 24) & 0xFF) << ":"
                  << ntohs(client_addr.sin_port) << "\n";

        int flag = 1;
        lwip_setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        int sendbuf = 4096;
        lwip_setsockopt(client_socket, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf));

        int opt = 1;
        if (lwip_ioctl(client_socket, FIONBIO, &opt) < 0) {
            Log::error << "Failed to set telnet client socket to non-blocking\n";
            lwip_close(client_socket);
            continue;
        }

        auto client_sink = std::make_shared<sink>(client_socket);

        if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            Log::error << "Failed to acquire clients mutex\n";
            lwip_close(client_socket);
            continue;
        }
        all_clients_sink.add_sink(client_sink.get());
        client_connection new_client;
        new_client.socket = client_socket;
        new_client.sink = client_sink;
        clients.push_back(new_client);
        xSemaphoreGive(clients_mutex);


        TaskHandle_t client_task_handle = nullptr;
        BaseType_t task_created =
            xTaskCreate(client_handler_task,
                        "TelnetClient",
                        configMINIMAL_STACK_SIZE * 2,
                        new std::pair<server*, int>(this, client_socket),
                        configMAX_PRIORITIES - 9,
                        &client_task_handle);

        if (task_created != pdPASS) {
            Log::error << "Failed to create telnet client handler task\n";
            if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                all_clients_sink.remove_sink(client_sink.get());
                clients.erase(std::remove_if(clients.begin(), clients.end(),
                                             [client_socket](const auto& client) {
                                                 return client.socket == client_socket;
                                             }),
                              clients.end());
                xSemaphoreGive(clients_mutex);
            }
            lwip_close(client_socket);
            continue;
        }

        if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            for (auto& client : clients) {
                if (client.socket == client_socket) {
                    client.task_handle = client_task_handle;
                    break;
                }
            }
            xSemaphoreGive(clients_mutex);
        }

        send_telnet_will_echo(client_socket);
    }
}

void server::send_telnet_will_echo(int client_socket) {
    uint8_t will_echo[] = {IAC, WILL, ECHO};
    lwip_send(client_socket, will_echo, sizeof(will_echo), 0);

    uint8_t will_sga[] = {IAC, WILL, SGA};
    lwip_send(client_socket, will_sga, sizeof(will_sga), 0);

    uint8_t do_echo[] = {IAC, DONT, ECHO};
    lwip_send(client_socket, do_echo, sizeof(do_echo), 0);
}


void server::client_handler_task(void* params) {
    auto* pair = static_cast<std::pair<server*, int>*>(params);
    server* server_instance = pair->first;
    int client_socket = pair->second;
    delete pair; // Free the parameters


    lwip_send(client_socket, server_instance->welcome_message.c_str(),
              server_instance->welcome_message.length(), 0);

    std::shared_ptr<sink> client_sink;
    if (xSemaphoreTake(server_instance->clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (const auto& client : server_instance->clients) {
            if (client.socket == client_socket) {
                client_sink = client.sink;
                break;
            }
        }
        xSemaphoreGive(server_instance->clients_mutex);
    }

    if (!client_sink) {
        Log::error << "Failed to find client sink for socket " << client_socket << "\n";
        lwip_close(client_socket);
        vTaskDelete(nullptr);
        return;
    }

    std::array<uint8_t, server::RECV_BUFFER_SIZE> buffer;
    while (server_instance->running) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(client_socket, &read_fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        int res = lwip_select(client_socket + 1, &read_fds, nullptr, nullptr, &tv);

        if (res < 0) {
            Log::error << "Select error in telnet client handler: " << errno << "\n";
            break;
        } else if (res == 0) {
            // Timeout, just loop
            continue;
        }

        if (!FD_ISSET(client_socket, &read_fds)) {
            continue;
        }

        int bytes_read = lwip_recv(client_socket, buffer.data(), buffer.size(), 0);

        if (bytes_read <= 0) {
            if (bytes_read < 0 && errno == EWOULDBLOCK) {
                continue;
            }

            Log::info << "Telnet client disconnected (socket " << client_socket << ")\n";
            break;
        }

        bool has_command = false;
        for (int i = 0; i < bytes_read - 1; i++) {
            if (buffer[i] == IAC) {
                server_instance->process_telnet_command(client_socket, buffer.data() + i,
                                                        bytes_read - i);
                has_command = true;
                break;
            }
        }

        if (!has_command) {
            for (int i = 0; i < bytes_read; i++) {
                uint8_t byte = buffer[i];
                if (xQueueSendToBack(server_instance->rx_byte_queue,
                                     &byte,
                                     pdMS_TO_TICKS(10)) != pdTRUE) {
                    break;
                }
            }
        }
    }

    lwip_close(client_socket);

    if (xSemaphoreTake(server_instance->clients_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        server_instance->all_clients_sink.remove_sink(client_sink.get());
        server_instance->clients.erase(
            std::remove_if(server_instance->clients.begin(),
                           server_instance->clients.end(),
                           [client_socket](const auto& client) {
                               return client.socket == client_socket;
                           }),
            server_instance->clients.end());
        xSemaphoreGive(server_instance->clients_mutex);
    }

    Log::debug << "Telnet client handler task exiting\n";
    vTaskDelete(nullptr);
}


void server::process_telnet_command(int client_socket, const uint8_t* data, size_t len) {
    if (len < 3) {
        return;
    }

    if (data[0] == IAC) {
        switch (data[1]) {
            case DO:
            case DONT: {
                uint8_t response[] = {IAC, WONT, data[2]};
                lwip_send(client_socket, response, sizeof(response), 0);
            } break;

            case WILL:
            case WONT: {
                uint8_t response[] = {IAC, DONT, data[2]};
                lwip_send(client_socket, response, sizeof(response), 0);
            } break;
            default:
                // Ignore other commands
                break;
        }
    }
}

sink::sink(int socket) : client_socket(socket), write_mutex(nullptr) {
    write_mutex = xSemaphoreCreateMutex();
    buffer.reserve(1024);
}

sink::~sink() {
    if (write_mutex) {
        vSemaphoreDelete(write_mutex);
        write_mutex = nullptr;
    }
}

void sink::write(const char* data, size_t len) {
    if (xSemaphoreTake(write_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (buffer.size() + len > 1024) {
            flush();
        }
        buffer.insert(buffer.end(), data, data + len);
        xSemaphoreGive(write_mutex);
    }
}

int sink::get_socket() const { return client_socket; }

void sink::flush() {
    if (client_socket >= 0) {
        lwip_send(client_socket, buffer.data(), buffer.size(), 0);
        buffer.clear();
    }
}

} // namespace piccante::wifi::telnet