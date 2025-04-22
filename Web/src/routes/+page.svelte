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
import Bar from '$/lib/components/Bar.svelte';
import Masonry from '$/lib/components/Masonry.svelte';
import { page } from '$app/state';
import { onMount } from 'svelte';
import { fade, slide } from 'svelte/transition';

let stats = $state(page.data);
let timed_out = $state(false);
let timeout: number | undefined = $state(undefined);

onMount(() => {
    timeout = setTimeout(() => {
        timed_out = true;
    }, 1500) as unknown as number;

    return () => {
        clearTimeout(timeout);
    };
});
$effect(
    () =>
        void (async () => {
            if (timed_out) {
                timed_out = false;
                const start = Date.now();
                try {
                    const res = await fetch('/api/stats');
                    stats = await res.json();
                } finally {
                    const end = Date.now();
                    timeout = setTimeout(
                        () => {
                            timed_out = true;
                        },
                        Math.max(0, 1500 + (end - start))
                    ) as unknown as number;
                }
            }
        })()
);
let width = $state(0);
let height = $state(0);

/* eslint-disable prettier/prettier */
const adcMap = {
    'CPU Temperature': {
        min: 0,
        max: 80,
        color: (value: number) => {
            if (value < 50) return '#11520d';
            if (value < 50) return '#ffa500';
            if (value < 60) return '#ff8100';
            return '#db1804';
        }
    },
    'System Voltage': {
        min: 0,
        max: 5,
        color: (value: number) => {
            if (value < 3.0) return '#db1804';
            if (value < 3.3) return '#ff8100';
            if (value < 3.5) return '#ffa500';
            return '#11520d';
        }
    }
} as {
    [key: string]: {
        min: number;
        max: number;
        color: (value: number) => string;
    } | undefined;
    /* eslint-enable prettier/prettier */
};

const getCpuLoadColor = (value: number) => {
    if (value > 90) return '#db1804';
    if (value > 75) return '#ff8100';
    if (value > 50) return '#ffa500';
    return '#11520d';
};
</script>

<svelte:window bind:innerWidth={width} bind:innerHeight={height} />

<div class="wrapper">
    <div class="card cpu d-grid">
        <h2>CPU</h2>
        <b>Core0</b>
        <div class="overlap" style="width: 100%;">
            <div transition:fade style="z-index: 0;">
                <Bar
                    value={stats.cpu.load0}
                    min={0}
                    max={100}
                    backgroundColor="transparent"
                    color={getCpuLoadColor(stats.cpu.load0)}></Bar>
            </div>
            <div style="z-index: 1;" class="input no-inp t-c t-s b">
                {`${stats.cpu.load0.toFixed(2)} %`}
            </div>
        </div>
        <b>Core1</b>
        <div class="overlap" style="width: 100%;">
            <div transition:fade style="z-index: 0;">
                <Bar
                    value={stats.cpu.load1}
                    min={0}
                    max={100}
                    backgroundColor="transparent"
                    color={getCpuLoadColor(stats.cpu.load1)}></Bar>
            </div>
            <div style="z-index: 1;" class="input no-inp t-c t-s b">
                {`${stats.cpu.load1.toFixed(2)} %`}
            </div>
        </div>
        <div class="card tasks">
            <h3>Tasks</h3>
            <h4>Name</h4>
            <h4>Priority</h4>
            <h4>CPU</h4>
            {#each stats.cpu.tasks as task (task.task_number)}
                <b transition:slide style="font-size: 1em;">
                    {task.name}
                </b>
                <span transition:slide style="text-align: center;">{task.priority}</span>
                <div class="overlap" style="width: 100%;">
                    <div transition:fade style="z-index: 0;">
                        <Bar
                            value={task.cpu_usage_0 + task.cpu_usage_1}
                            min={0}
                            max={100}
                            backgroundColor="transparent"
                            color={getCpuLoadColor(task.cpu_usage_0 + task.cpu_usage_1)}></Bar>
                    </div>
                    <div style="z-index: 1;" class="input no-inp t-c t-s b">
                        {`${(task.cpu_usage_0 + task.cpu_usage_1).toFixed(2)} %`}
                    </div>
                </div>
            {/each}
        </div>
    </div>
    <Masonry gridGap="2em" colWidth={`minmax(${Math.min(Math.max(280, width - 32), 350)}px, 1fr)`}>
        <div class="card d-grid">
            <h2>Memory</h2>
            <b>Heap</b>
            <div style="display: grid; gap: 0.5em; place-items: center; width: 100%;">
                <span
                    >{(stats.memory.heap_used / 1024).toFixed(0)} kB / {(
                        stats.memory.total_heap / 1024
                    ).toFixed(0)} kB</span>
                <div class="overlap" style="width: 100%;">
                    <div transition:fade style="z-index: 0;">
                        <Bar
                            value={stats.memory.heap_used}
                            min={0}
                            max={stats.memory.total_heap}
                            backgroundColor="transparent"
                            color={(() => {
                                if (stats.memory.heap_used === undefined) return '#3f3f4f';
                                if (stats.memory.heap_used > stats.memory.total_heap * 0.9) return '#db1804';
                                if (stats.memory.heap_used > stats.memory.total_heap * 0.75) return '#ff8100';
                                if (stats.memory.heap_used > stats.memory.total_heap * 0.5) return '#ffa500';
                                return '#11520d';
                            })()}></Bar>
                    </div>
                    <div style="z-index: 1;" class="input no-inp t-c t-s b">
                        {`${((stats.memory.heap_used / stats.memory.total_heap) * 100).toFixed(2)} %`}
                    </div>
                </div>

                <span
                    >Min free: {(stats.memory.min_free_heap / 1024).toFixed(0)} kB ({(
                        (stats.memory.min_free_heap / stats.memory.total_heap) *
                        100
                    ).toFixed(1)} %)</span>
            </div>
        </div>

        <div class="card d-grid">
            <h2>WiFi</h2>
            <b>Mode</b>
            <input
                disabled
                class="no-inp t-c t-s"
                value={stats.wifi?.mode == 0 ? 'Off' : stats.wifi.mode === 1 ? 'Client' : 'Access Point'} />
            <b>SSID</b>
            <input disabled class="no-inp t-c t-s" value={stats.wifi?.ssid} />
            <b>Channel</b>
            <input disabled class="no-inp t-c t-s" value={stats.wifi?.channel} />
            <b>RSSI</b>
            <div class="overlap" style="width: 100%;">
                <div transition:fade style="z-index: 0;">
                    {#if stats.wifi?.rssi}
                        <Bar
                            value={100 + (stats.wifi?.rssi || 0)}
                            min={0}
                            max={70}
                            backgroundColor="transparent"
                            color={(() => {
                                if (stats.wifi?.rssi === undefined) return '#3f3f4f';
                                if (stats.wifi.rssi < -70) return '#db1804';
                                if (stats.wifi.rssi < -60) return '#ff8100';
                                if (stats.wifi.rssi < -50) return '#ffa500';
                                return '#11520d';
                            })()}></Bar>
                    {/if}
                </div>
                <div style="z-index: 1;" class="input no-inp t-c t-s b">
                    {`${stats.wifi?.rssi} dBm`}
                </div>
            </div>
            <b>IP</b>
            <input disabled class="no-inp t-c t-s" value={stats.wifi?.ip_address} />
            <b>MAC</b>
            <input disabled class="no-inp t-c t-s" value={stats.wifi?.mac_address} />
        </div>

        <div class="card d-grid">
            <h2>ADC</h2>
            {#each stats.adc as adc, idx (idx)}
                <b>{adc.name}</b>
                <div style="display: grid; gap: 0.5em; place-items: center; width: 100%;">
                    <div class="overlap">
                        {#if adc.value >= 1}
                            <div transition:fade style="z-index: 0;">
                                <Bar
                                    value={adc.value}
                                    min={adcMap[adc.name]?.min ?? 0}
                                    max={adcMap[adc.name]?.max ?? 3.33}
                                    backgroundColor="transparent"
                                    color={adcMap[adc.name]?.color(adc.value) || '#3f3f4f'}></Bar>
                            </div>
                        {/if}
                        <div style="z-index: 1;" class="input no-inp t-c t-s b">
                            {adc.value.toFixed(2) + ' ' + adc.unit}
                        </div>
                    </div>
                    {#if adc.name === 'System Voltage'}
                        <span style="font-size: 0.9em">(May read low on W)</span>
                    {/if}
                </div>
            {/each}
        </div>
        <div class="card uptime">
            <h2>System Uptime</h2>
            <span
                >{stats.uptime.days > 0 ? `${stats.uptime.days} days ` : ''}{stats.uptime.hours} hours, {stats
                    .uptime.minutes} minutes, {stats.uptime.seconds} seconds</span>
        </div>
    </Masonry>
</div>

<style lang="postcss">
.wrapper {
    width: 100%;
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(450px, 1fr));
    gap: 2em;
    & > :global(*) {
        align-self: start;
    }
    @media (orientation: portrait) {
        grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    }
}

.card {
    width: 100%;
}

.tasks {
    grid-column: span 2;
    display: grid;
    grid-template-columns: min-content min-content auto;
    @media (orientation: portrait) {
        grid-template-columns: min-content auto;

        & > :nth-child(2) {
            padding-left: 0.5em;
        }

        & > :nth-child(4) {
            display: none;
        }
        & > :nth-child(3n) {
            place-self: end;
            padding-right: 1em;
        }
        & > :nth-child(3) {
            place-self: end;
            padding-right: 0.5em;
        }
        & > :nth-child(3n + 1) {
            grid-column: 1 / -1;
            place-self: center;
        }
        & > :first-child {
            grid-column: 1 / -1;
            place-self: start;
        }
    }

    width: 100%;
    gap: 1em;
    position: relative;
    & h4 {
        font-size: 1.07em;
    }
    & > :first-child {
        grid-column: 1 / -1;
    }
    &::after {
        content: '';
        height: 1px;
        background: var(--text-color);
        width: calc(100% - 2em);
        top: -1em;
        justify-self: center;
        opacity: 0.5;
        position: absolute;
        grid-row: 3;
    }
}

.overlap {
    display: grid;
    & > * {
        grid-column: 1;
        grid-row: 1;
    }
    width: 100%;
}

.cpu {
}

.no-inp {
    opacity: 1 !important;
    text-align: end;
}
.b {
    font-weight: bold;
}
.t-c {
    text-align: center;
}
.t-s {
    text-shadow: 1px 1px 2px #000;
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
    grid-column-gap: 2em;
    @media (orientation: portrait) {
        grid-column-gap: 1.5em;
    }
}
</style>
