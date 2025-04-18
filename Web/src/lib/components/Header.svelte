<script lang="ts">
import Logo from '$assets/logo.svg?component';
import IcRoundMenu from '~icons/ic/round-menu';
import Drawer from '$/lib/components/Drawer.svelte';
import Nav from './Nav.svelte';

let drawerOpen = $state(false);
</script>

<div class="header">
    <button
        onclick={() => {
            drawerOpen = !drawerOpen;
        }}><IcRoundMenu /></button>
    <h1>
        {document.title || 'PiCCANTE'}
    </h1>
    <div class="logo-container">
        <Logo style="height: 100%" />
    </div>
</div>

<Drawer bind:open={drawerOpen} side="left">
    <Nav />
</Drawer>

<style lang="postcss">
button {
    border: none;
    height: 100%;
    aspect-ratio: 1 / 1;
    display: grid;
    font-size: 2rem;
    place-items: center;

    @media (orientation: landscape) {
        display: none;
    }
}
h1 {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    margin: 0;
    padding: 0.5em;
}
.header {
    width: 100%;
    background-color: var(--card-color);
    display: grid;
    grid-template-columns: min-content auto min-content;
    align-items: center;
    box-shadow: 0 0 10px rgba(0, 0, 0, 0.5);
    height: fit-content;
    overflow: hidden;
}
.logo-container {
    height: 100%;
    display: flex;
    align-items: center;
    padding: 0.5em 1em 0.5em 0;
    @media (orientation: landscape) {
        grid-row: 1 / span 1;
        padding: 0.5em 0em 1em 1em;
    }
}
</style>
