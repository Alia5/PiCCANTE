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
import Header from '$/lib/components/Header.svelte';
import '../main.pcss';
import Nav from '$/lib/components/Nav.svelte';
const { children } = $props();
</script>

<div class="wrapper">
    <Header />
    <div class="main">
        <aside class="sidebar">
            <Nav />
        </aside>
        <section>
            {@render children()}
        </section>
    </div>
</div>

<style lang="postcss">
.wrapper {
    min-height: 100dvh;
    display: grid;
    grid-template-rows: min-content auto;
}
.main {
    display: grid;
    grid-template-columns: min-content auto;

    @media (orientation: portrait) {
        grid-template-columns: auto;
    }
}

section {
    padding: 2em;
    display: grid;
    justify-items: center;
    height: 100%;
    width: 100%;
    @media (orientation: portrait) {
        padding: 1em;
    }
    & > :global(*) {
        grid-row: 1;
        grid-column: 1;
    }
    &:has(:global(*[inert])) {
        overflow: hidden;
    }
}

aside {
    height: 100%;
    background-color: var(--card-color);
    position: relative;
    &::before {
        content: '';
        position: absolute;
        top: 0;
        left: 0;
        right: 0;
        bottom: 0;
        box-shadow: 0 0 10px rgba(0, 0, 0, 0.5);
        pointer-events: none;
    }
    &::after {
        content: '';
        position: absolute;
        width: 100%;
        height: 11px;
        left: 3px;
        top: -11px;
        background-color: var(--card-color);
        z-index: 1;
    }
    @media (orientation: portrait) {
        display: none;
    }
}
</style>
