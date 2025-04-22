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
    children?: Snippet;
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
    background: var(--bgc);
    border-radius: var(--radius);
    position: relative;
    overflow: hidden;
    height: 100%;
}
.v {
    position: absolute;
    inset: 0;
    background: var(--color);
}
.a {
    transition:
        width var(--transition-duration) ease-in-out,
        height var(--transition-duration) ease-in-out,
        background calc(var(--transition-duration) * 2) ease-in-out;
}
</style>
