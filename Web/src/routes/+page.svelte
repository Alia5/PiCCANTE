<script lang="ts">
import Bar from '$/lib/components/Bar.svelte';
</script>

<div class="grid">
    <div class="card cpu">
        <h2>CPU</h2>
        <div></div>
    </div>
    <div class="card memory">
        <h2>Memory</h2>
        <div class="mem-row">
            <span>Free heap:</span>
            <Bar value={62.6} color="#4caf50">
                <span>Whatever</span>
            </Bar>
        </div>
        <div class="mem-row">
            <span>Min free heap:</span>
            <Bar value={62.6} color="#4caf50"><span> 82040 bytes </span></Bar>
        </div>
        <div class="mem-row">
            <span>Heap used:</span>
            <Bar value={37.4} color="#f39c12">
                <span> 49040 bytes </span>
            </Bar>
        </div>
    </div>

    <div class="card">
        <h2>WiFi</h2>
        <ul>
            <li><b>Mode:</b> Client</li>
            <li><b>SSID:</b> REDACTED</li>
            <li><b>Channel:</b> 1</li>
            <li><b>RSSI:</b> -52</li>
            <li><b>IP:</b> REDACTED</li>
            <li><b>MAC:</b> REDACTED</li>
        </ul>
    </div>

    <div class="card">
        <h2>ADC</h2>
        <ul>
            <li><b>ADC0:</b> 0.086 V (Raw: 107, GPIO26)</li>
            <li><b>ADC1:</b> 0.077 V (Raw: 95, GPIO27)</li>
            <li><b>ADC2:</b> 0.076 V (Raw: 94, GPIO28)</li>
            <li><b>System Voltage:</b> 4.233 V (Raw: 1751, GPIO29)</li>
            <li><b>CPU Temp:</b> 28.9 °C (Raw: 872)</li>
        </ul>
    </div>

    <div class="card uptime">
        <h2>System Uptime</h2>
        <div>0 hours, 0 minutes, 19 seconds</div>
    </div>
</div>

<style lang="postcss">
.grid {
    display: grid;
    width: 100%;
    height: fit-content;
    grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
    place-items: center;
    gap: 2em;
    grid-auto-flow: dense;
    @media (orientation: portrait) {
        grid-template-columns: 1fr;
    }
}

.card {
    width: 100%;
}

.cpu {
    grid-column: span 2;
    @media (orientation: portrait) {
        grid-column: auto;
    }
}

.uptime {
    grid-column: 1 / -1;
    @media (orientation: portrait) {
        grid-column: auto;
    }
}

li {
    list-style: none;
    padding-left: 0.5em;
}

.memory .mem-row {
    display: flex;
    align-items: center;
    gap: 0.5em;
    margin-bottom: 0.5em;
    font-size: 1em;
}
</style>
