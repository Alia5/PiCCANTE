<script lang="ts">
import type { Snippet } from 'svelte';

const {
    backgroundColor = '#eee',
    animate = true,
    vertical = false,
    value = 0,
    min = 0,
    max = 100,
    color = '',
    radius = '0.5em',
    children
}: {
    backgroundColor?: string;
    animate?: boolean;
    vertical?: boolean;
    value?: number;
    min?: number;
    max?: number;
    color?: string;
    radius?: string;
    children: Snippet;
} = $props();

const percent = $derived(((value - min) / (max - min)) * 100);
</script>

<div class="bar" style={`--bgc: ${backgroundColor}; --radius: ${radius};`}>
    <div
        class={`v ${animate ? 'a' : ''}`}
        style={`--color: ${color}; ${vertical ? 'height:' + percent + '%' : 'width:' + percent + '%'};`}>
    </div>
    {@render children?.()}
</div>

<style lang="postcss">
.bar {
    background-color: var(--bgc);
    border-radius: var(--radius);
    position: relative;
    overflow: hidden;
}
.v {
    position: absolute;
    inset: 0;
    background-color: var(--color);
}
.a {
    transition: all var(--transition-duration) ease;
    transition-property: width, height;
    transition-duration: var(--transition-duration);
    transition-timing-function: ease-in-out;
}
</style>
