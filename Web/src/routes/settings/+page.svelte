<!--
 PiCCANTE - PiCCANTE Car Controller Area Network Tool for Exploration
 Copyright (C) 2025 Peter Repukat

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 -->
<script lang="ts">
import Dropdown from '$/lib/components/Dropdown.svelte';
import Masonry from '$/lib/components/Masonry.svelte';
import { page } from '$app/state';
import { fade, fly, slide } from 'svelte/transition';
import type { PiCCANTE_Settings } from './+page';
import Loadingspinner from '$/lib/components/Loadingspinner.svelte';
import { cubicOut } from 'svelte/easing';

const can_bitrates = [
    { name: '33.3 kbit/s', value: 33333 },
    { name: '50 kbit/s', value: 50000 },
    { name: '100 kbit/s', value: 100000 },
    { name: '125 kbit/s', value: 125000 },
    { name: '250 kbit/s', value: 250000 },
    { name: '500 kbit/s', value: 500000 },
    { name: '1 Mbit/s', value: 1000000 }
];

const led_modes = [
    { name: 'Off', value: 0 },
    { name: 'Power', value: 1 },
    { name: 'CAN Activity', value: 2 }
];

const elm_interfaces = [
    { name: 'USB', value: 0 },
    { name: 'Bluetooth', value: 1 },
    { name: 'WiFi', value: 2 }
];

const idle_timeout = [
    { name: 'Off', value: 0 },
    { name: '1 min', value: 1 },
    { name: '5 min', value: 5 },
    { name: '15 min', value: 15 },
    { name: '30 min', value: 30 }
];

let settings = $state(page.data.settings);
let wifi_enabled = $state(settings.wifi_mode > 0);
let wifi_mode = $state(settings.wifi_mode - 1);

let loading = $state(false);

$effect(() => {
    if (wifi_enabled) {
        settings.wifi_mode = wifi_mode + 1;
    } else {
        settings.wifi_mode = 0;
    }
});
$effect(() => {
    if (wifi_enabled) {
        settings.wifi_mode = wifi_mode + 1;
    }
});

type PartialNested<T> = {
    [K in keyof T]?: T[K] extends object | undefined ? PartialNested<T[K]> : T[K];
};
const applySetting = async (setting: PartialNested<PiCCANTE_Settings>) => {
    loading = true;
    try {
        await fetch('/api/settings', {
            method: 'POST',
            body: JSON.stringify(setting)
        });
        const res = await fetch('/api/settings');
        if (res.ok) {
            const data = await res.json();
            settings = data;
        } else {
            console.error('Failed to apply settings');
        }
    } finally {
        loading = false;
    }
};

const saveSettings = async (reset?: boolean) => {
    loading = true;
    try {
        const settingsCopy = JSON.parse(JSON.stringify(settings));
        delete settingsCopy.can_settings.enabled;
        delete settingsCopy.wifi_settings;
        settingsCopy.reboot = true;
        await applySetting(settingsCopy);

        await fetch('/api/save', {
            method: 'POST',
            body: reset ? JSON.stringify({ reset: true }) : ''
        });
    } finally {
        loading = false;
    }
};
</script>

<div class="wrapper" in:fly={{ y: window.innerHeight, easing: cubicOut }} out:fade={{}}>
    <Masonry gridGap="2em" colWidth="minmax(280px, 1fr)">
        <Masonry gridGap="2em" colWidth="minmax(280px, 1fr)">
            <div class="card d-grid">
                <h2>General</h2>
                <b>Echo</b>
                <div>
                    <input type="checkbox" class="toggle" bind:checked={settings.echo} />
                </div>
                <b>LED Mode</b>
                <Dropdown options={led_modes} bind:selected={settings.led_mode} />
                <b>Idle Timeout</b>
                <Dropdown options={idle_timeout} bind:selected={settings.idle_sleep_minutes} />
            </div>
            <div class="card d-grid child-gap">
                <h2>ELM327 Emulator</h2>
                <b>Interface</b>
                <Dropdown options={elm_interfaces} bind:selected={settings.elm_settings.interface} />
                <b>CAN-Bus</b>
                <Dropdown
                    options={Object.entries(settings.can_settings)
                        // eslint-disable-next-line @typescript-eslint/no-explicit-any
                        .filter(([k, v]) => k.startsWith('can') && (v as any).enabled)
                        .map((b, idx) => ({
                            name: `CAN${idx}`,
                            value: idx
                        }))}
                    bind:selected={settings.elm_settings.bus} />
                {#if settings.elm_settings.interface === 1}
                    <b transition:slide> Bluetooth PIN</b>
                    <input transition:slide type="number" bind:value={settings.elm_settings.bt_pin} />
                    <button
                        transition:slide
                        style="grid-column: 2; width: 100%; padding: 0.4em;"
                        onclick={() =>
                            applySetting({
                                elm_settings: {
                                    bt_pin: settings.elm_settings.bt_pin
                                }
                            })}>
                        Set PIN
                    </button>
                {/if}
            </div>
        </Masonry>
        <div class="card can">
            <div class="d-grid">
                <h2>CAN</h2>
                <b>Max. Supported</b>
                <p>{settings.can_settings.max_supported}</p>
                <b>Available</b>
                <Dropdown
                    options={Array(settings.can_settings.max_supported)
                        .fill(0)
                        .map((_, i) => ({ name: `${i + 1}`, value: i + 1 }))}
                    bind:selected={settings.can_settings.enabled}
                    onchange={() =>
                        applySetting({
                            can_settings: {
                                enabled: settings.can_settings.enabled
                            }
                        })} />
                <b></b>
                <span>(Immediately resets board!)</span>
                <b>Bus speed lock</b>
                <div>
                    <input type="checkbox" class="toggle" bind:checked={settings.can_settings.baud_lockout} />
                </div>
                <b></b>
                <span>(Prevents GVRET/SLCAN from changing bus speed)</span>
            </div>
            {#each Array(settings.can_settings.enabled)
                .fill(0)
                .map((_, i) => i) as idx (idx)}
                <div class="card d-grid" transition:slide>
                    <h3>CAN{idx}</h3>
                    <b>Enabled</b>
                    <div>
                        <input
                            type="checkbox"
                            class="toggle"
                            bind:checked={settings.can_settings[`can${idx}`].enabled} />
                    </div>
                    <b>Listen Only</b>
                    <div>
                        <input
                            type="checkbox"
                            class="toggle"
                            disabled={!settings.can_settings[`can${idx}`].enabled}
                            bind:checked={settings.can_settings[`can${idx}`].listen_only} />
                    </div>
                    <b>Baudrate</b>
                    <Dropdown
                        options={can_bitrates}
                        disabled={!settings.can_settings[`can${idx}`].enabled}
                        bind:selected={settings.can_settings[`can${idx}`].bitrate} />
                </div>
            {/each}
        </div>
        <div class="card d-grid wifi">
            <h2>WiFi</h2>
            <b>Enabled</b>
            <div>
                <input type="checkbox" class="toggle" bind:checked={wifi_enabled} />
            </div>
            <b>Mode</b>
            <Dropdown
                options={[
                    { name: 'Connect', value: 0 },
                    { name: 'Access Point', value: 1 }
                ]}
                disabled={!wifi_enabled}
                bind:selected={wifi_mode} />
            <b>SSID</b><input bind:value={settings.wifi_settings.ssid} disabled={!wifi_enabled} />
            <b>Password</b><input
                type="password"
                bind:value={settings.wifi_settings.password}
                disabled={!wifi_enabled} />
            <b>Channel</b>
            <input
                type="number"
                step="1"
                min="1"
                max="12"
                bind:value={settings.wifi_settings.channel}
                disabled={!wifi_enabled} />
            <div></div>
            <button
                onclick={() => {
                    void applySetting({
                        wifi_mode: settings.wifi_mode,
                        wifi_settings: {
                            ssid: settings.wifi_settings.ssid,
                            password: settings.wifi_settings.password,
                            channel: settings.wifi_settings.channel
                        }
                    });
                }}>Apply WiFi Settings</button>
            <div class="card d-grid">
                <h3 style={!wifi_enabled ? 'opacity: 0.5;' : ''}>Telnet</h3>
                <b style={!wifi_enabled ? 'opacity: 0.5;' : ''}>Enabled</b>
                <div>
                    <input
                        type="checkbox"
                        class="toggle"
                        bind:checked={settings.wifi_settings.telnet_enabled}
                        disabled={!wifi_enabled} />
                </div>
                <b style={!wifi_enabled ? 'opacity: 0.5;' : ''}>Port</b>
                <input
                    type="number"
                    step="1"
                    min="1"
                    max="65535"
                    bind:value={settings.wifi_settings.telnet_port}
                    disabled={!wifi_enabled} />
            </div>
        </div>
    </Masonry>
    {#if loading}
        <div class="loader" transition:fade><Loadingspinner size="4em" ringWidth="6px" /></div>
    {/if}
    <div class="btns" in:fade={{ delay: 400 }} out:fade>
        <button onclick={() => void saveSettings(true)}> Save and Reboot </button>
    </div>
</div>

<style lang="postcss">
.wrapper {
    width: 100%;
    .loader {
        position: absolute;
        inset: 0;
        display: grid;
        place-items: center;
        background-color: rgba(0, 0, 0, 0.5);
        z-index: 100;
    }
}
.btns {
    width: fit-content;
    display: grid;
    grid-template-columns: auto auto;
    gap: 1em;
    padding: 2em 0 1.5em 0;
    place-self: end;
    @media (orientation: landscape) {
        position: absolute;
        top: -1em;
        right: 2em;
    }
}

.can {
    display: grid;
    span {
        font-size: 0.8em;
        margin-top: -0.75em;
        justify-self: end;
    }
    & > .card {
        margin-top: 1em;
    }
}

.d-grid {
    & > :first-child {
        grid-column: span 2;
    }
    & > div {
        justify-self: end;
    }
    display: grid;
    align-items: center;
    grid-template-columns: auto minmax(6em, 1fr);
    grid-row-gap: 1em;
    grid-column-gap: 1em;
    @media (orientation: portrait) {
        grid-column-gap: 2em;
    }
}

.child-gap {
    row-gap: 0em;
    & > :global(:nth-child(n + 2)) {
        margin-top: 1em;
    }
}

.wifi {
    & * {
        transition: all var(--transition-duration) ease;
    }
}

.wifi > .card {
    grid-column: span 2;
    width: 100%;
    margin-top: 1em;
}
</style>
