/* eslint-disable */
export const mocks = {
    '/api/settings': {
        GET: {
            status: 200,
            body: {
                echo: true,
                log_level: 0,
                led_mode: 2,
                wifi_mode: 1,
                idle_sleep_minutes: 5,
                wifi_settings: {
                    ssid: 'REDACTED',
                    password: 'REDACTED',
                    channel: 1,
                    telnet_port: 23,
                    telnet_enabled: true
                },
                can_settings: {
                    max_supported: 3,
                    enabled: 3,
                    can0: {
                        enabled: true,
                        listen_only: false,
                        bitrate: 500000
                    },
                    can1: {
                        enabled: false,
                        listen_only: false,

                        bitrate: 500000
                    },
                    can2: {
                        enabled: false,
                        listen_only: false,

                        bitrate: 125000
                    }
                },
                "elm_settings": {
                    "interface": 1,
                    "bus": 0,
                    "bt_pin": 1337,
                }
            }
        },
        POST: {
            status: 200,
            body: {
                status: 'ok'
            },
            func: (data: any) => {
                // mocks.. don't need to care.
                Object.keys(data).forEach((key) => {
                    // @ts-ignore
                    if (typeof mocks['/api/settings'].GET.body[key] === 'object') {
                        Object.keys(data[key]).forEach((subKey) => {
                            // @ts-ignore
                            if (typeof mocks['/api/settings'].GET.body[key][subKey] === 'object') {
                                Object.keys(data[key][subKey]).forEach((subSubKey) => {
                                    // @ts-ignore
                                    mocks['/api/settings'].GET.body[key][subKey][subSubKey] = data[key][subKey][subSubKey];
                                });
                            } else {
                                // @ts-ignore
                                mocks['/api/settings'].GET.body[key][subKey] = data[key][subKey];
                            }
                        });
                    } else {
                        // @ts-ignore
                        mocks['/api/settings'].GET.body[key] = data[key];
                    }
                });
            }
        }
    }
};
