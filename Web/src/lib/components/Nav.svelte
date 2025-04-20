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
import { page } from '$app/state';
</script>

<nav>
    <ul>
        <li><a href="/" class={page.route.id === '/' ? 'selected' : ''}>Status</a></li>
        <li>
            <a href="/settings" class={page.url.pathname.endsWith('/settings') ? 'selected' : ''}>Settings</a>
        </li>
        <div></div>
        <li><a href="/about" class={page.url.pathname.endsWith('/about') ? 'selected' : ''}>About</a></li>
    </ul>
</nav>

<style lang="postcss">
nav {
    height: 100%;
    min-width: 12em;
}
ul {
    display: flex;
    flex-direction: column;
    gap: 0.5em;
    padding: 1em 1em 1em 1em;
    height: 100%;
    height: 100%;
}

.selected {
    font-weight: bold;
    opacity: 1;
}

nav li {
    position: relative;
    display: block;
}

nav li {
    display: grid;
    place-items: center;
    &::after {
        content: '';
        height: 3px;
        width: 100%;
        bottom: 0em;
        transform: translateY(0.25em);
        left: 0;
        position: absolute;
        background: var(--text-color);
        scale: 0 1;
        transition:
            scale 200ms var(--scaleDelay, 0ms) ease,
            translate 400ms var(--translateDelay, 0ms) ease;
    }
    &:hover,
    &:focus-within {
        &::after {
            scale: 1 1;
        }
    }
}

@supports selector(:has(h1)) {
    nav li {
        &:hover {
            & + li::after {
                --scaleDelay: 200ms;
                --translateDelay: 200ms;
            }
        }
        &:has(+ :hover) {
            &::after {
                --scaleDelay: 200ms;
                --translateDelay: 200ms;
            }
        }
    }
}

a {
    padding: 0 1em;
    transition-property: all;
    transition-duration: var(--transition-duration);
    transition-timing-function: ease-in-out;
    transition-delay: 0.001ms;
    text-decoration: none;
    color: var(--text-color);
    border-radius: 0.5em;
    font-size: 1.5em;
    opacity: 0.8;
    &:hover {
        opacity: 1;
    }
}

div {
    flex: 1;
}
</style>
