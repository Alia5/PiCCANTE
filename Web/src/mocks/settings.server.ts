
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
                }
            }
        },
        POST: {
            status: 200,
            body: {
                status: 'ok'
            }
        }
    }
};
