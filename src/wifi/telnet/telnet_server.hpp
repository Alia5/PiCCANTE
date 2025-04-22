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
#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <array>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "outstream/stream.hpp"

#ifdef write
#undef write
#endif

namespace piccante::wifi::telnet {

class sink;

struct client_connection {
    int socket;
    TaskHandle_t task_handle;
    std::shared_ptr<telnet::sink> sink;
};

class server {
        public:
    static constexpr size_t RX_QUEUE_LENGTH = 1024;
    static constexpr size_t RECV_BUFFER_SIZE = 1024;

    explicit server(std::string_view name, uint16_t port,
                    std::string_view welcome_message);
    virtual ~server();

    server(const server&) = delete;
    server& operator=(const server&) = delete;

    bool start();
    void stop();
    bool is_running() const;
    bool reconfigure(uint16_t new_port);

    QueueHandle_t get_rx_queue() const;
    SemaphoreHandle_t get_clients_mutex() const { return clients_mutex; }
    const std::vector<client_connection>& get_clients() const { return clients; }

        private:
    static void server_task(void* params);
    static void client_handler_task(void* params);

    bool create_socket();
    void close_socket();
    void accept_connections();

    void process_telnet_command(int client_socket, const uint8_t* data, size_t len);
    void send_telnet_will_echo(int client_socket);

    QueueHandle_t rx_byte_queue;

    std::string name;
    uint16_t port;
    std::string welcome_message;

    bool running;
    int server_socket;
    TaskHandle_t server_task_handle;

    SemaphoreHandle_t clients_mutex;


    std::vector<client_connection> clients;
};

class sink : public out::base_sink {
        public:
    explicit sink(int socket);
    ~sink() override;
    void write(const char* data, std::size_t len) override;
    int get_socket() const;
    void flush() override;

        private:
    int client_socket;
    SemaphoreHandle_t write_mutex;
};

} // namespace piccante::wifi::telnet