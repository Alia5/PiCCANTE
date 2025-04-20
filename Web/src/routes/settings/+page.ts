export const prerender = false;
export const csr = true;

export const load = async ({ fetch }) => {
    const res = await fetch('/api/settings');
    if (res.status === 200) {
        const data = await res.json();
        return { settings: data as PiCCANTE_Settings };
    }
    return { settings: undefined };
};

export interface PiCCANTE_Settings {
    'echo': boolean;
    'log_level': 0 | 1 | 2 | 3 | 4;
    'led_mode': 0 | 1 | 2;
    'wifi_mode': 0 | 1 | 2;
    'idle_sleep_minutes': number;
    'wifi_settings'?: {
        'ssid': string;
        'password': string;
        'channel': number;
        'telnet_port': number;
        'telnet_enabled': boolean;
    };
    'can_settings': {
        'max_supported': 1 | 2 | 3;
        'enabled': 1 | 2 | 3;
        'can0'?: {
            'enabled': true;
            'bitrate': number;
        };
        'can1'?: {
            'enabled': false;
            'bitrate': number;
        };
        'can2'?: {
            'enabled': false;
            'bitrate': number;
        };
    };
}
