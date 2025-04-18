<script lang="ts">
import Masonry from '$/lib/components/Masonry.svelte';
import IcRoundArrowDropDown from '~icons/ic/round-arrow-drop-down';
let echo = $state(false);

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
</script>

<Masonry gridGap="2em" colWidth="minmax(420px, 1fr)">
    <div class="card">
        <h2>General</h2>
        <ol>
            <li><b>Echo</b><input type="checkbox" class="toggle" bind:checked={echo} /></li>
            <li>
                <b>LED Mode</b>
                <div class="dropdown" style="min-width: 8em;">
                    <select>
                        {#each led_modes as mode (mode.value)}
                            <option value={mode.value}>{mode.name}</option>
                        {/each}
                    </select>
                    <div class="icon">
                        <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                    </div>
                </div>
            </li>
            <li>
                <b>Idle Timeout</b>
                <div class="dropdown" style="min-width: 8em;">
                    <select>
                        {#each idle_timeout as timeout (timeout.value)}
                            <option value={timeout.value}>{timeout.name}</option>
                        {/each}
                    </select>
                    <div class="icon">
                        <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                    </div>
                </div>
            </li>
        </ol>
    </div>
    <div class="card can">
        <div>
            <h2>CAN</h2>
            <ol>
                <li><b>Max. Supported</b>3</li>
                <li>
                    <b>Enabled</b>
                    <div class="dropdown" style="min-width: 8em;">
                        <select>
                            <option>1</option>
                            <option>2</option>
                            <option>3</option>
                        </select>
                        <div class="icon">
                            <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                        </div>
                    </div>
                </li>
            </ol>
        </div>
        <div class="card">
            <h3>CAN0</h3>
            <ol>
                <li><b>Enabled</b><input type="checkbox" class="toggle" /></li>
                <li><b>Listen Only</b><input type="checkbox" class="toggle" /></li>
                <li>
                    <b>Baudrate</b>
                    <div class="dropdown" style="min-width: 8em;">
                        <select>
                            {#each can_bitrates as bitrate (bitrate.value)}
                                <option value={bitrate.value}>{bitrate.name}</option>
                            {/each}
                        </select>
                        <div class="icon">
                            <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                        </div>
                    </div>
                </li>
            </ol>
        </div>
        <div class="card">
            <h3>CAN1</h3>
            <ol>
                <li><b>Enabled</b><input type="checkbox" class="toggle" /></li>
                <li><b>Listen Only</b><input type="checkbox" class="toggle" /></li>
                <li>
                    <b>Baudrate</b>
                    <div class="dropdown" style="min-width: 8em;">
                        <select>
                            {#each can_bitrates as bitrate (bitrate.value)}
                                <option value={bitrate.value}>{bitrate.name}</option>
                            {/each}
                        </select>
                        <div class="icon">
                            <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                        </div>
                    </div>
                </li>
            </ol>
        </div>
        <div class="card">
            <h3>CAN2</h3>
            <ol>
                <li><b>Enabled</b><input type="checkbox" class="toggle" /></li>
                <li><b>Listen Only</b><input type="checkbox" class="toggle" /></li>
                <li>
                    <b>Baudrate</b>
                    <div class="dropdown" style="min-width: 8em;">
                        <select>
                            {#each can_bitrates as bitrate (bitrate.value)}
                                <option value={bitrate.value}>{bitrate.name}</option>
                            {/each}
                        </select>
                        <div class="icon">
                            <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
                        </div>
                    </div>
                </li>
            </ol>
        </div>
    </div>
    <div class="card wifi">
        <h2>WiFi</h2>
        <b>Enabled</b>
        <div>
            <input type="checkbox" class="toggle" />
        </div>
        <b>Mode</b>
        <div class="dropdown" style="min-width: 100%;">
            <select>
                <option>Connect</option>
                <option>Access Point</option>
            </select>
            <div class="icon">
                <IcRoundArrowDropDown style="width: 2.4em; height: 2.4em;" />
            </div>
        </div>
        <b>SSID</b><input />
        <b>Password</b><input type="password" />
        <b>Channel</b> <input type="number" step="1" min="1" max="12" />
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
    gap: 2em;
}

.wifi {
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
