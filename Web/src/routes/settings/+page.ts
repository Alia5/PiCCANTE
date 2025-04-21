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
