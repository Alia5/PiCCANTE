<script lang="ts">
import { onNavigate } from '$app/navigation';
import { onMount, type Snippet } from 'svelte';
import Logo from '$assets/logo.svg?component';
import IcRoundArrowBack from '~icons/ic/round-arrow-back';
import { browser } from '$app/environment';

let {
    open = $bindable(false),
    // eslint-disable-next-line prefer-const
    side = 'left',
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
    side?: 'left' | 'right';
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

<aside class="drawer {side === 'left' || side === undefined ? 'left' : 'right'} {open && 'open'}">
    {#if header}
        {@render header()}
    {:else}
        <div class="drawer-header">
            {#if side === 'left'}
                <button onclick={close} aria-label="close left drawer"><IcRoundArrowBack /></button>
            {/if}
            <div class="logo-title-wrapper">
                <Logo style="width: 5rem;" />
                {#if title}
                    <h2>{title}</h2>
                {/if}
            </div>
            {#if side === 'right'}
                <button onclick={close} style="transform: rotate(180deg);" aria-label="close right drawer"
                    ><IcRoundArrowBack /></button>
            {/if}
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
    &.right {
        box-shadow: -10px 0 10px rgba(0, 0, 0, 0.5);
    }
    &.open {
        transform: translateX(0);
    }
}
.left {
    left: -1px;
    transform: translateX(calc(-100% - 10px));
}
.right {
    right: 1px;
    transform: translateX(100%);
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
