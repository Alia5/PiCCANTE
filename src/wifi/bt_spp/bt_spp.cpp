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
constexpr size_t TX_BUFFER_SIZE = 64;

static QueueHandle_t rx_queue = nullptr;
static uint16_t rfcomm_channel_id = 0;
static bool running = false;
static uint8_t spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;
static TaskHandle_t bt_task_handle = nullptr;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet,
                           uint16_t size);
static void spp_bt_task(void* params);
static constexpr int SPP_TX_QUEUE_SIZE = 256;
static QueueHandle_t tx_queue = nullptr;

} // namespace

// Implementation of sink class for SPP output
class spp_sink : public out::base_sink {
        public:
    void write(const char* data, std::size_t len) override {
        if (!running || rfcomm_channel_id == 0) {
            return;
        }

        for (size_t i = 0; i < len; i++) {
            // Try to send with a timeout
            if (xQueueSend(tx_queue, &data[i], pdMS_TO_TICKS(1)) != pdTRUE) {
                flush();
                vTaskDelay(pdMS_TO_TICKS(1));
                // Try again
                if (xQueueSend(tx_queue, &data[i], pdMS_TO_TICKS(5)) != pdPASS) {
                    // Still failed, drop byte
                    Log::debug << "BT TX queue overflow, dropped byte\n";
                }
            }
        }
    }

    void flush() override {
        if (!running || rfcomm_channel_id == 0) {
            return;
        }
        rfcomm_request_can_send_now_event(rfcomm_channel_id);
    }
};

// Static instance of the sink
static spp_sink bluetooth_spp_sink;
static out::sink_mux sink_muxxer;
static bool spp_sink_added = false;
static uint16_t max_tx_size = 0;

// Public API implementations
bool initialize() {
    if (running) {
        return true;
    }

    // Create queue for received bytes
    rx_queue = xQueueCreate(SPP_RX_QUEUE_SIZE, sizeof(uint8_t));
    if (!rx_queue) {
        Log::error << "Failed to create Bluetooth SPP RX queue\n";
        return false;
    }


    tx_queue = xQueueCreate(SPP_TX_QUEUE_SIZE, sizeof(uint8_t));
    if (!tx_queue) {
        Log::error << "Failed to create Bluetooth SPP TX queue\n";
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
        return false;
    }


    hci_event_callback_registration.callback = &packet_handler;

    // Don't call BTstack functions directly here - they'll be called from the task
    running = true;
    Log::info << "Bluetooth SPP initialized, starting task\n";

    return true;
}

TaskHandle_t create_task() {
    if (bt_task_handle != nullptr) {
        return bt_task_handle;
    }

    // Create a task for Bluetooth operations
    if (xTaskCreate(spp_bt_task, "BT SPP", configMINIMAL_STACK_SIZE, nullptr,
                    configMAX_PRIORITIES - 8, &bt_task_handle) != pdPASS) {
        Log::error << "Failed to create Bluetooth SPP task\n";
        return nullptr;
    }

    return bt_task_handle;
}

void stop() {
    if (!running) {
        return;
    }

    // Stop the Bluetooth task
    if (bt_task_handle != nullptr) {
        vTaskDelete(bt_task_handle);
        bt_task_handle = nullptr;
    }

    // Clean up resources
    if (rx_queue) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
    }

    if (tx_queue) {
        vQueueDelete(tx_queue);
        tx_queue = nullptr;
    }

    running = false;
    Log::info << "Bluetooth SPP server stopped\n";
}

bool is_running() { return running; }

QueueHandle_t get_rx_queue() { return rx_queue; }

out::base_sink& get_sink() { return bluetooth_spp_sink; }

out::sink_mux& mux_sink(std::initializer_list<out::base_sink*> sinks) {
    if (!spp_sink_added) {
        sink_muxxer.add_sink(&bluetooth_spp_sink);
        spp_sink_added = true;
    }

    for (auto* sink : sinks) {
        sink_muxxer.add_sink(sink);
    }

    return sink_muxxer;
}

bool reconfigure() {
    if (!running) {
        return initialize();
    }
    return true;
}

namespace {

// Bluetooth task - handles initialization and events
static void spp_bt_task(void* params) {
    (void)params;

    Log::info << "Starting Bluetooth SPP task\n";

    // Wait a bit for the system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    initialize();

    // Register for HCI events - AFTER hci_init
    hci_add_event_handler(&hci_event_callback_registration);

    // Initialize L2CAP
    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager (needed for cross-transport key derivation)
    sm_init();
#endif

    // Initialize RFCOMM
    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);

    // Initialize SDP and create SPP service record
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, sdp_create_service_record_handle(),
                          RFCOMM_SERVER_CHANNEL, "PiCCANTE SPP");
    sdp_register_service(spp_service_buffer);

    // Set device as discoverable and enable SSP
    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("PiCCANTE SPP");

    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);

    Log::info << "Bluetooth SPP initialization complete\n";

    // Enter the BTstack run loop - this never returns
    btstack_run_loop_execute();

    // If we somehow exit the run loop, clean up
    Log::error << "Bluetooth SPP run loop exited unexpectedly\n";
    running = false;
    vTaskDelete(NULL);
}

// Packet handler for Bluetooth events and data
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet,
                           uint16_t size) {
    (void)channel;

    bd_addr_t event_addr;
    uint8_t rfcomm_channel_nr;
    uint16_t mtu;

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
                        rfcomm_channel_id =
                            rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        max_tx_size = mtu;
                        Log::info << "Bluetooth RFCOMM channel opened. Channel ID "
                                  << rfcomm_channel_id << ", max frame size " << mtu
                                  << "\n";
                    }
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    if (tx_queue != nullptr) {
                        UBaseType_t queued_items = uxQueueMessagesWaiting(tx_queue);
                        if (queued_items > 0) {
                            uint16_t bytes_to_send =
                                (queued_items > max_tx_size) ? max_tx_size : queued_items;

                            std::vector<uint8_t> tx_packet_buffer(
                                std::min(uint16_t(queued_items), max_tx_size));

                            for (uint16_t i = 0; i < bytes_to_send; i++) {
                                if (xQueueReceive(tx_queue, &tx_packet_buffer[i], 0) !=
                                    pdTRUE) {
                                    break;
                                }
                            }

                            rfcomm_send(rfcomm_channel_id, tx_packet_buffer.data(),
                                        bytes_to_send);

                            if (uxQueueMessagesWaiting(tx_queue) > 0) {
                                rfcomm_request_can_send_now_event(rfcomm_channel_id);
                            }
                        }
                    }
                    break;
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    Log::info << "Bluetooth RFCOMM channel closed\n";
                    rfcomm_channel_id = 0;
                    // TODO: clean queue;
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            // Process received data
            for (uint16_t i = 0; i < size; i++) {
                // Add to rx queue
                if (rx_queue) {
                    uint8_t byte = packet[i];
                    xQueueSend(rx_queue, &byte, 0);
                }
            }
            break;

        default:
            break;
    }
}

} // anonymous namespace

} // namespace piccante::bluetooth