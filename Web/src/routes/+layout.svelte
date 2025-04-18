<script lang="ts">
import Header from '$/lib/components/Header.svelte';
import '../main.pcss';
import Nav from '$/lib/components/Nav.svelte';
import { onNavigate } from '$app/navigation';
const { children } = $props();

onNavigate((navigation) => {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    if (!(document as any).startViewTransition) return;
    return new Promise((resolve) => {
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        (document as any).startViewTransition(async () => {
            resolve();
            await navigation.complete;
        });
    });
});
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
