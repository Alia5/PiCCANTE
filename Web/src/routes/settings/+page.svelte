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
import { slide } from 'svelte/transition';
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

const idle_timeout = [
    { name: 'Off', value: 0 },
    { name: '1 min', value: 1 },
    { name: '5 min', value: 5 },
    { name: '15 min', value: 15 },
    { name: '30 min', value: 30 }
];

const settings = $state(page.data.settings);
let wifi_enabled = $state(settings.wifi_mode > 0);
let wifi_mode = $state(settings.wifi_mode - 1);

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
</script>

<Masonry gridGap="2em" colWidth="minmax(420px, 1fr)">
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
    <div class="card can">
        <div>
            <h2>CAN</h2>
            <ol>
                <li><b>Max. Supported</b>{settings.can_settings.max_supported}</li>
                <li>
                    <b>Enabled</b>
                    <Dropdown
                        options={[
                            { name: '1', value: 1 },
                            { name: '2', value: 2 },
                            { name: '3', value: 3 }
                        ]}
                        bind:selected={settings.can_settings.enabled} />
                </li>
            </ol>
        </div>
        {#each Array(settings.can_settings.enabled)
            .fill(0)
            .map((_, i) => i) as idx (idx)}
            <div class="card" transition:slide>
                <h3>CAN{idx}</h3>
                <ol>
                    <li>
                        <b>Enabled</b><input
                            type="checkbox"
                            class="toggle"
                            bind:checked={settings.can_settings[`can${idx}`].enabled} />
                    </li>
                    <li>
                        <b>Listen Only</b><input
                            type="checkbox"
                            class="toggle"
                            disabled={!settings.can_settings[`can${idx}`].enabled}
                            bind:checked={settings.can_settings[`can${idx}`].listen_only} />
                    </li>
                    <li>
                        <b>Baudrate</b>
                        <Dropdown
                            options={can_bitrates}
                            disabled={!settings.can_settings[`can${idx}`].enabled}
                            bind:selected={settings.can_settings[`can${idx}`].bitrate} />
                    </li>
                </ol>
            </div>
        {/each}
    </div>
    <div class="card d-grid">
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
    </div>
</Masonry>

<style lang="postcss">
ol {
    display: grid;
    gap: 1em;
}

li {
    display: grid;
    align-items: center;
    grid-template-columns: auto min-content;
    gap: 3em;
}

.can {
    display: grid;
    & > .card {
        margin-top: 2em;
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
    grid-template-columns: auto min-content;
    grid-row-gap: 1em;
    grid-column-gap: 3em;
}
</style>
