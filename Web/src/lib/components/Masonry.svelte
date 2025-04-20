<!-- Stolen and adapted from https://github.com/janzheng/svelte-masonry/blob/master/src/lib/components/Masonry.svelte  -->
<script lang="ts">
import { browser } from '$app/environment';
import { onMount, tick, type Snippet } from 'svelte';

interface MasonryProps {
    gridGap?: string;
    colWidth?: string;
    children: Snippet;
}

interface GridItem {
    element: HTMLElement;
    items: HTMLElement[];
    ncol: number;
    mod: number;
}

const { gridGap = '1em', colWidth = 'minmax(min(10em, 100%), 1fr)', children }: MasonryProps = $props();

let grids: GridItem[] = $state([]);
let masonryElement: HTMLElement | undefined = $state(undefined);

const refreshLayout = () => {
    grids.forEach((grid) => {
        const ncol = getComputedStyle(grid.element).gridTemplateColumns.split(' ').length;

        grid.items.forEach((c) => {
            const newHeight = c.getBoundingClientRect().height;
            const currentHeight = parseFloat(c.dataset.h || '0');

            if (newHeight !== currentHeight) {
                c.dataset.h = newHeight.toString();
                grid.mod++;
            }
        });

        if (grid.ncol !== ncol || grid.mod) {
            grid.ncol = ncol;
            grid.items.forEach((c) => c.style.removeProperty('margin-top'));

            if (grid.ncol > 1) {
                grid.items.slice(ncol).forEach((c, i) => {
                    const prevBottom = grid.items[i].getBoundingClientRect().bottom;
                    const currTop = c.getBoundingClientRect().top;
                    c.style.marginTop = `calc(${prevBottom - currTop}px + var(--grid-gap))`;
                });
            }

            grid.mod = 0;
        }
    });
};

const calcGrid = async (masonryArr: HTMLElement[]) => {
    await tick();

    if (masonryArr.length && getComputedStyle(masonryArr[0]).gridTemplateRows !== 'masonry') {
        grids = masonryArr.map((grid) => ({
            element: grid,
            items: Array.from(grid.childNodes).filter(
                (c): c is HTMLElement => c instanceof HTMLElement && +getComputedStyle(c).gridColumnEnd !== -1
            ),
            ncol: 0,
            mod: 0
        }));

        refreshLayout();
    }
};

onMount(() => {
    if (browser) {
        window.addEventListener('resize', refreshLayout, false);

        return () => {
            window.removeEventListener('resize', refreshLayout, false);
        };
    }
});

$effect(() => {
    if (masonryElement) {
        void calcGrid([masonryElement]);
    }
});
</script>

<div
    bind:this={masonryElement}
    class="grid-masonry"
    style={`--grid-gap: ${gridGap}; --col-width: ${colWidth}; width: 100%;`}>
    {@render children?.()}
</div>

<style>
.grid-masonry {
    display: grid;
    grid-template-columns: repeat(auto-fit, var(--col-width));
    grid-template-rows: masonry;
    justify-content: center;
    grid-gap: var(--grid-gap);
    transition: all 0.2s ease;
    & > :global(*) {
        align-self: start;
        transition: all 0.2s ease;
    }
}
</style>
