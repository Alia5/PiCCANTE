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
import { onNavigate } from '$app/navigation';
import { onMount, type Snippet } from 'svelte';
import IcRoundArrowBack from '~icons/ic/round-arrow-back';
import { browser } from '$app/environment';

let {
    open = $bindable(false),
    // eslint-disable-next-line prefer-const
    cancellable = true,
    // eslint-disable-next-line prefer-const
    title,
    // eslint-disable-next-line prefer-const
    header,
    // eslint-disable-next-line prefer-const
    children
}: {
    open?: boolean;
    cancellable?: boolean;
    title?: string;
    header?: Snippet;
    children?: Snippet;
} = $props();

const close = () => {
    open = false;
};

onMount(() => {
    const handleKeydown = (e: KeyboardEvent) => {
        if (e.key === 'Escape' && cancellable) {
            close();
        }
    };
    window.addEventListener('keydown', handleKeydown);
    return () => {
        window.removeEventListener('keydown', handleKeydown);
    };
});

onNavigate(() => {
    close();
});

const scrollbarWidth = $derived(browser ? window.innerWidth - document.documentElement.clientWidth : 0);

$effect(() => {
    if (browser) {
        if (open) {
            document.body.style.overflow = 'hidden';
            document.body.style.paddingRight = `${scrollbarWidth}px`;
            document.body.style.transition = 'none';
        } else {
            document.body.style.overflow = '';
            document.body.style.paddingRight = '';
            setTimeout(() => (document.body.style.transition = ''), 250);
        }
    }
});
</script>

<aside class="drawer left {open && 'open'}">
    {#if header}
        {@render header()}
    {:else}
        <div class="drawer-header">
            <button onclick={close} aria-label="close left drawer"><IcRoundArrowBack /></button>
            <div class="logo-title-wrapper">
                <img src="/logo.svg" alt="PiCCANTE Logo" style="width: 4rem;" />
                {#if title}
                    <h2>{title}</h2>
                {/if}
            </div>
        </div>
    {/if}
    {#if children}
        {@render children()}
    {/if}
</aside>
<button class="scrim {open && 'open'}" aria-label="scrim-close-drawer" onclick={() => cancellable && close()}
></button>

<style lang="postcss">
.drawer {
    position: fixed;
    top: 0;
    bottom: 0;
    background-color: var(--card-color);
    z-index: 100;
    transition: transform 0.2s ease-in-out;
    display: grid;
    grid-template-rows: min-content auto;
    height: 100%;
    width: fit-content;
    &.left {
        box-shadow: 0 0 10px rgba(0, 0, 0, 0.5);
    }
    &.open {
        transform: translateX(0);
    }
}
.left {
    left: -1px;
    transform: translateX(calc(-100% - 10px));
}

.scrim {
    position: fixed;
    width: 100%;
    inset: 0;
    background-color: rgba(0, 0, 0, 0.5);
    margin: 0;
    border: none;
    outline: none;
    border-radius: 0;
    opacity: 0;
    z-index: -1;
    transition: opacity var(--transition-duration) ease;
    &.open {
        z-index: 90;
        opacity: 1;
    }
}

.drawer-header {
    display: grid;
    grid-template-columns: auto auto;
    align-items: center;
    padding: 0.5rem 1rem;
    & button {
        padding: 0;
        border: none;
        background-color: transparent;
        font-size: 2rem;
        cursor: pointer;
        margin: 0;
        box-shadow: none;
    }
}

.logo-title-wrapper {
    display: grid;
    place-items: center;
    padding: 0.5em 0em 1em 1em;
}
</style>
