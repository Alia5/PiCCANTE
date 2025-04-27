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
#include "bt_spp.hpp"
#include "Logger/Logger.hpp"
#include <array>
#include <vector>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern "C" {
#include "btstack.h"
}

namespace piccante::bluetooth {

namespace {
constexpr uint8_t RFCOMM_SERVER_CHANNEL = 1;
constexpr uint16_t SPP_RX_QUEUE_SIZE = 256;
constexpr uint16_t SPP_TX_BUFFER_SIZE = 256;
constexpr uint32_t THROUGHPUT_REPORT_INTERVAL_MS = 3000;
constexpr uint8_t BT_REQUEST_QUEUE_SIZE = 10;

enum class BtRequestType : uint8_t { REQUEST_CAN_SEND_NOW, DISCONNECT };

struct BtRequest {
    BtRequestType type;
    uint16_t rfcomm_cid;
};

SemaphoreHandle_t tx_buffer_mutex = nullptr;
SemaphoreHandle_t state_mutex = nullptr;

QueueHandle_t rx_queue = nullptr;
QueueHandle_t bt_request_queue = nullptr;
std::array<uint8_t, SPP_TX_BUFFER_SIZE> tx_buffer;
uint16_t tx_buffer_size = 0;

uint16_t rfcomm_channel_id = 0;
bool running = false;
bool send_pending = false;
std::array<uint8_t, 150> spp_service_buffer;
btstack_packet_callback_registration_t hci_event_callback_registration;
TaskHandle_t bt_task_handle = nullptr;

uint32_t data_sent = 0;
uint32_t data_received = 0;
uint32_t last_throughput_time = 0;
uint16_t max_frame_size = 0;

btstack_context_callback_registration_t can_send_now_callback_registration;

// Forward declarations
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet,
                    uint16_t size);
void process_can_send_now();
void report_throughput(bool force = false);
void bt_task(void* params);
bool enqueue_bt_request(BtRequestType type, uint16_t rfcomm_cid);

void request_can_send_now() {
    send_pending = true;
    if (rfcomm_channel_id != 0) {
        if (!enqueue_bt_request(BtRequestType::REQUEST_CAN_SEND_NOW, rfcomm_channel_id)) {
            Log::error << "BT: Failed to queue can-send-now request\n";
            send_pending = false;
        }
    } else {
        send_pending = false;
    }
}

// Enqueue a BT request to be processed on the main BT thread
bool enqueue_bt_request(BtRequestType type, uint16_t rfcomm_cid) {
    if (!bt_request_queue) {
        return false;
    }

    BtRequest request = {.type = type, .rfcomm_cid = rfcomm_cid};

    return xQueueSend(bt_request_queue, &request, pdMS_TO_TICKS(5)) == pdTRUE;
}

// Callback for can-send-now event to be executed in BTstack thread context
void can_send_now_callback(void* context) {
    uint16_t rfcomm_cid = *((uint16_t*)context);
    rfcomm_request_can_send_now_event(rfcomm_cid);
}

void cleanup_resources() {
    if (rx_queue) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
    }

    if (bt_request_queue) {
        vQueueDelete(bt_request_queue);
        bt_request_queue = nullptr;
    }

    if (tx_buffer_mutex) {
        vSemaphoreDelete(tx_buffer_mutex);
        tx_buffer_mutex = nullptr;
    }

    if (state_mutex) {
        vSemaphoreDelete(state_mutex);
        state_mutex = nullptr;
    }
}

bool init_resources() {
    // Create mutexes
    tx_buffer_mutex = xSemaphoreCreateMutex();
    if (!tx_buffer_mutex) {
        Log::error << "Failed to create BT TX buffer mutex\n";
        return false;
    }

    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex) {
        cleanup_resources();
        Log::error << "Failed to create BT state mutex\n";
        return false;
    }

    // Create queues
    rx_queue = xQueueCreate(SPP_RX_QUEUE_SIZE, sizeof(uint8_t));
    if (!rx_queue) {
        cleanup_resources();

        Log::error << "Failed to create BT RX queue\n";
        return false;
    }

    bt_request_queue = xQueueCreate(BT_REQUEST_QUEUE_SIZE, sizeof(BtRequest));
    if (!bt_request_queue) {
        cleanup_resources();
        Log::error << "Failed to create BT request queue\n";
        return false;
    }

    tx_buffer_size = 0;
    send_pending = false;
    rfcomm_channel_id = 0;
    data_sent = 0;
    data_received = 0;

    running = true;
    return true;
}

void init() {
    l2cap_init();

#ifdef ENABLE_BLE
    sm_init();
#endif

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);

    sdp_init();
    spp_create_sdp_record(spp_service_buffer.data(), sdp_create_service_record_handle(),
                          RFCOMM_SERVER_CHANNEL, "PiCCANTE SPP");
    sdp_register_service(spp_service_buffer.data());

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("PiCCANTE SPP");

    hci_power_control(HCI_POWER_ON);

    last_throughput_time = btstack_run_loop_get_time_ms();

    Log::info << "Bluetooth SPP stack initialized\n";
}

class spp_sink : public out::base_sink {
        public:
    void write(const char* data, std::size_t len) override {
        if (xSemaphoreTake(tx_buffer_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            Log::debug << "BT: Failed to take TX mutex\n";
            return;
        }

        std::memcpy(tx_buffer.data() + tx_buffer_size, data, len);
        tx_buffer_size += len;

        xSemaphoreGive(tx_buffer_mutex);
    }

    void flush() override {
        if (!send_pending) {
            request_can_send_now();
        }
    }
};
spp_sink bluetooth_spp_sink;


void process_can_send_now() {
    if (!running || rfcomm_channel_id == 0 || !send_pending) {
        send_pending = false;
        return;
    }

    if (xSemaphoreTake(tx_buffer_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        Log::error << "BT: Failed to take TX mutex\n";
        return;
    }

    if (tx_buffer_size > 0) {
        uint16_t bytes_to_send =
            tx_buffer_size > max_frame_size ? max_frame_size : tx_buffer_size;

        uint8_t status = rfcomm_send(rfcomm_channel_id, tx_buffer.data(), bytes_to_send);

        if (status == ERROR_CODE_SUCCESS) {
            data_sent += bytes_to_send;
            std::memmove(tx_buffer.data(), tx_buffer.data() + bytes_to_send,
                         tx_buffer_size - bytes_to_send);
            tx_buffer_size -= bytes_to_send;
            report_throughput();
        } else {
            Log::error << "BT send error: " << status << "\n";
        }
    }

    if (tx_buffer_size > 0) {
        request_can_send_now();
    } else {
        send_pending = false;
    }

    xSemaphoreGive(tx_buffer_mutex);
}

void report_throughput(bool force) {
    const uint32_t now = btstack_run_loop_get_time_ms();
    const uint32_t time_passed = now - last_throughput_time;

    if (force || time_passed >= THROUGHPUT_REPORT_INTERVAL_MS) {
        if (data_sent > 0 || data_received > 0) {
            const int tx_bytes_per_second = data_sent * 1000 / time_passed;
            const int rx_bytes_per_second = data_received * 1000 / time_passed;

            Log::debug << "BT Throughput - TX: " << (tx_bytes_per_second / 1000) << "."
                       << (tx_bytes_per_second % 1000)
                       << " kB/s, RX: " << (rx_bytes_per_second / 1000) << "."
                       << (rx_bytes_per_second % 1000) << " kB/s\n";
        }

        last_throughput_time = now;
        data_sent = 0;
        data_received = 0;
    }
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet,
                    uint16_t size) {
    (void)channel;

    bd_addr_t event_addr;
    uint8_t rfcomm_channel_nr;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        Log::info << "Bluetooth SPP stack is up and running\n";
                    }
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // Using fixed PIN "0000"
                    Log::debug << "Bluetooth PIN code request - using '0000'\n";
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // Auto-accept SSP confirmation
                    Log::debug << "Bluetooth SSP User Confirmation Auto accept\n";
                    // Note: Not calling gap_user_confirmation_response since it doesn't
                    // exist
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    Log::debug << "Bluetooth RFCOMM incoming connection\n";
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr =
                        rfcomm_event_incoming_connection_get_server_channel(packet);

                    rfcomm_channel_id =
                        rfcomm_event_incoming_connection_get_rfcomm_cid(packet);

                    Log::debug << "Bluetooth RFCOMM channel "
                               << static_cast<int>(rfcomm_channel_nr) << " requested\n";

                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;

                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        Log::error << "Bluetooth RFCOMM channel open failed, status 0x"
                                   << static_cast<int>(
                                          rfcomm_event_channel_opened_get_status(packet))
                                   << "\n";
                        rfcomm_channel_id = 0;
                    } else {
                        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            rfcomm_channel_id =
                                rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                            max_frame_size =
                                rfcomm_event_channel_opened_get_max_frame_size(packet);

                            last_throughput_time = btstack_run_loop_get_time_ms();
                            data_sent = 0;
                            data_received = 0;

                            xSemaphoreGive(state_mutex);
                        }

                        Log::info
                            << "Bluetooth RFCOMM channel opened. Channel ID "
                            << rfcomm_event_channel_opened_get_rfcomm_cid(packet)
                            << ", max frame size "
                            << rfcomm_event_channel_opened_get_max_frame_size(packet)
                            << "\n";
                        gap_discoverable_control(0);
                    }
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    process_can_send_now();
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    Log::info << "Bluetooth RFCOMM channel closed\n";
                    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        rfcomm_channel_id = 0;
                        send_pending = false;
                        xSemaphoreGive(state_mutex);
                    }

                    if (xSemaphoreTake(tx_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        tx_buffer_size = 0;
                        xSemaphoreGive(tx_buffer_mutex);
                    }

                    if (rx_queue) {
                        xQueueReset(rx_queue);
                    }

                    report_throughput(true);
                    gap_discoverable_control(1);
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            data_received += size;

            if (rx_queue) {
                for (uint16_t i = 0; i < size; i++) {
                    // Add to rx queue
                    uint8_t byte = packet[i];
                    BaseType_t result = xQueueSend(rx_queue, &byte, 0);
                    if (result != pdTRUE) {
                        // Queue full - discard oldest byte and try again
                        uint8_t dummy;
                        xQueueReceive(rx_queue, &dummy, 0);
                        xQueueSend(rx_queue, &byte, 0);
                    }
                }
            }

            report_throughput();
            break;

        default:
            break;
    }
}

void bt_task(void* params) {
    (void)params;

    Log::info << "Starting Bluetooth SPP task\n";

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!init_resources()) {
        Log::error << "Failed to initialize Bluetooth resources\n";
        vTaskDelete(NULL);
        return;
    }

    init();

    // Main task loop - process BT requests
    // The CYW43 driver handles the BTstack run loop with CYW43_ENABLE_BLUETOOTH=1
    BtRequest request;
    uint16_t rfcomm_cid_context;
    while (running) {
        while (xQueueReceive(bt_request_queue, &request, pdMS_TO_TICKS(5)) == pdTRUE) {
            switch (request.type) {
                case BtRequestType::REQUEST_CAN_SEND_NOW:
                    rfcomm_cid_context = request.rfcomm_cid;
                    can_send_now_callback_registration.callback = &can_send_now_callback;
                    can_send_now_callback_registration.context = &rfcomm_cid_context;
                    btstack_run_loop_execute_on_main_thread(
                        &can_send_now_callback_registration);
                    // rfcomm_request_can_send_now_event(request.rfcomm_cid);
                    break;
                case BtRequestType::DISCONNECT:
                    if (request.rfcomm_cid != 0) {
                        rfcomm_disconnect(request.rfcomm_cid);
                    }
                    break;
            }
        }
    }

    if (rfcomm_channel_id != 0) {
        rfcomm_disconnect(rfcomm_channel_id);
    }

    hci_power_control(HCI_POWER_OFF);

    Log::info << "Bluetooth SPP task exiting\n";
    cleanup_resources();
    vTaskDelete(NULL);
}

} // namespace

TaskHandle_t create_task() {
    if (bt_task_handle != nullptr) {
        return bt_task_handle;
    }

    if (xTaskCreate(bt_task, "BT Task", configMINIMAL_STACK_SIZE, nullptr,
                    configMAX_PRIORITIES - 8, &bt_task_handle) != pdPASS) {
        Log::error << "Failed to create Bluetooth task\n";
        return nullptr;
    }

    // Set core affinity to match cyw43 driver // TODO: needed?
    vTaskCoreAffinitySet(bt_task_handle, 0x01);

    return bt_task_handle;
}

void stop() {
    if (!running) {
        return;
    }

    running = false;

    vTaskDelay(pdMS_TO_TICKS(300));

    if (bt_task_handle != nullptr) {
        vTaskDelete(bt_task_handle);
        bt_task_handle = nullptr;
    }

    Log::info << "Bluetooth SPP server stopped\n";
}

bool is_running() { return running; }

QueueHandle_t get_rx_queue() { return rx_queue; }

out::base_sink& get_sink() { return bluetooth_spp_sink; }

} // namespace piccante::bluetooth