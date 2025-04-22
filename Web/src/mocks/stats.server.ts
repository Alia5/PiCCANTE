/* eslint-disable */
export const mocks = {
    '/api/stats': {
        GET: {
            status: 200,
            body: {
                "is_pico2": true,
                "uptime": {
                    "days": 0,
                    "hours": 0,
                    "minutes": 3,
                    "seconds": 4,
                    "ticks": 184807
                },
                "memory": {
                    "free_heap": 66768,
                    "min_free_heap": 66768,
                    "total_heap": 131072,
                    "heap_used": 64304
                },
                "cpu": {
                    "total_runtime": 184863237,
                    "load0": 3.068493,
                    "load1": 2.095370,
                    "tasks": [
                        {
                            "name": "HTTP Server",
                            "state": 2,
                            "priority": 2,
                            "stack_high_water": 424,
                            "core_id": 0,
                            "task_number": 16,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.04,
                            "cpu_usage_1": 0.06
                        },
                        {
                            "name": "CAN",
                            "state": 2,
                            "priority": 27,
                            "stack_high_water": 468,
                            "core_id": 1,
                            "task_number": 9,
                            "core_affinity": 2,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.84
                        },
                        {
                            "name": "StatsTask",
                            "state": 0,
                            "priority": 1,
                            "stack_high_water": 386,
                            "core_id": 1,
                            "task_number": 1,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.22
                        },
                        {
                            "name": "IdleDetect",
                            "state": 2,
                            "priority": 1,
                            "stack_high_water": 409,
                            "core_id": 0,
                            "task_number": 8,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "WiFi",
                            "state": 2,
                            "priority": 1,
                            "stack_high_water": 346,
                            "core_id": 1,
                            "task_number": 10,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "IDLE1",
                            "state": 1,
                            "priority": 0,
                            "stack_high_water": 484,
                            "core_id": 1,
                            "task_number": 12,
                            "core_affinity": -1,
                            "cpu_usage_0": 44.00,
                            "cpu_usage_1": 53.71
                        },
                        {
                            "name": "IDLE0",
                            "state": 1,
                            "priority": 0,
                            "stack_high_water": 466,
                            "core_id": 0,
                            "task_number": 11,
                            "core_affinity": -1,
                            "cpu_usage_0": 52.93,
                            "cpu_usage_1": 44.20
                        },
                        {
                            "name": "async_context_t",
                            "state": 2,
                            "priority": 4,
                            "stack_high_water": 816,
                            "core_id": 0,
                            "task_number": 14,
                            "core_affinity": 1,
                            "cpu_usage_0": 1.47,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "SLCAN0",
                            "state": 2,
                            "priority": 22,
                            "stack_high_water": 462,
                            "core_id": 0,
                            "task_number": 5,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.17,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "SLCAN1",
                            "state": 2,
                            "priority": 22,
                            "stack_high_water": 462,
                            "core_id": 0,
                            "task_number": 6,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.14,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "SLCAN2",
                            "state": 2,
                            "priority": 22,
                            "stack_high_water": 462,
                            "core_id": 0,
                            "task_number": 7,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.13,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "TelnetServer",
                            "state": 2,
                            "priority": 5,
                            "stack_high_water": 918,
                            "core_id": 0,
                            "task_number": 17,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.02,
                            "cpu_usage_1": 0.07
                        },
                        {
                            "name": "PiCCANTE+GVRET",
                            "state": 2,
                            "priority": 6,
                            "stack_high_water": 298,
                            "core_id": 0,
                            "task_number": 4,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.35,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "tcpip_thread",
                            "state": 2,
                            "priority": 1,
                            "stack_high_water": 332,
                            "core_id": 1,
                            "task_number": 15,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.18
                        },
                        {
                            "name": "USB",
                            "state": 2,
                            "priority": 26,
                            "stack_high_water": 438,
                            "core_id": 0,
                            "task_number": 2,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.00,
                            "cpu_usage_1": 0.00
                        },
                        {
                            "name": "Tmr Svc",
                            "state": 2,
                            "priority": 31,
                            "stack_high_water": 978,
                            "core_id": 1,
                            "task_number": 13,
                            "core_affinity": -1,
                            "cpu_usage_0": 0.02,
                            "cpu_usage_1": 0.73
                        },
                        {
                            "name": "CAN RX",
                            "state": 0,
                            "priority": 5,
                            "stack_high_water": 452,
                            "core_id": 0,
                            "task_number": 3,
                            "core_affinity": 1,
                            "cpu_usage_0": 0.73,
                            "cpu_usage_1": 0.00
                        }
                    ]
                },
                "fs": {
                    "block_size": 4096,
                    "block_count": 128,
                    "total_size": 524288,
                    "used_size": 212992,
                    "free_size": 311296,
                },
                "adc": [
                    {
                        "channel": 0,
                        "value": 0.755092,
                        "raw": 937,
                        "name": "ADC0",
                        "unit": "V"
                    },
                    {
                        "channel": 1,
                        "value": 0.568938,
                        "raw": 706,
                        "name": "ADC1",
                        "unit": "V"
                    },
                    {
                        "channel": 2,
                        "value": 0.419853,
                        "raw": 521,
                        "name": "ADC2",
                        "unit": "V"
                    },
                    {
                        "channel": 3,
                        "value": 4.310550,
                        "raw": 1783,
                        "name": "System Voltage",
                        "unit": "V"
                    },
                    {
                        "channel": 4,
                        "value": 28.911331,
                        "raw": 872,
                        "name": "CPU Temperature",
                        "unit": "°C"
                    }
                ],
                "can": [
                    {
                        "bus": 0,
                        "enabled": true,
                        "bitrate": 500000,
                        "rx_buffered": 0,
                        "tx_buffered": 0,
                        "rx_overflow": 0,
                        "rx_total": 0,
                        "tx_total": 0,
                        "tx_attempt": 0,
                        "parse_error": 0
                    },
                    {
                        "bus": 1,
                        "enabled": false,
                        "bitrate": 500000,
                        "rx_buffered": 0,
                        "tx_buffered": 0,
                        "rx_overflow": 0,
                        "rx_total": 0,
                        "tx_total": 0,
                        "tx_attempt": 0,
                        "parse_error": 0
                    },
                    {
                        "bus": 2,
                        "enabled": false,
                        "bitrate": 500000,
                        "rx_buffered": 0,
                        "tx_buffered": 0,
                        "rx_overflow": 0,
                        "rx_total": 0,
                        "tx_total": 0,
                        "tx_attempt": 0,
                        "parse_error": 0
                    }
                ],
                "wifi": {
                    "mode": 1,
                    "connected": true,
                    "ssid": "REDACTED",
                    "channel": 1,
                    "rssi": -57,
                    "ip_address": "192.168.13.37",
                    "mac_address": "ff:af:ff:af:ff:af"
                }
            },
            func: () => {
                mocks['/api/stats'].GET.body.uptime.ticks += 1000;
                mocks['/api/stats'].GET.body.uptime.seconds += 1;
                if (mocks['/api/stats'].GET.body.uptime.seconds >= 60) {
                    mocks['/api/stats'].GET.body.uptime.seconds = 0;
                    mocks['/api/stats'].GET.body.uptime.minutes += 1;
                }
                if (mocks['/api/stats'].GET.body.uptime.minutes >= 60) {
                    mocks['/api/stats'].GET.body.uptime.minutes = 0;
                    mocks['/api/stats'].GET.body.uptime.hours += 1;
                }
                mocks['/api/stats'].GET.body.adc[3].value = 1 + Math.random() * 4.5
                mocks['/api/stats'].GET.body.adc[4].value = 20 + Math.random() * 60
            }
        }
    }
};
