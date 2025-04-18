import adapter from '@sveltejs/adapter-static';
import { vitePreprocess } from '@sveltejs/vite-plugin-svelte';
import htmlMinifierAdaptor from "sveltekit-html-minifier";


/** @type {import('@sveltejs/kit').Config} */
const config = {
    // Consult https://svelte.dev/docs/kit/integrations
    // for more information about preprocessors
    preprocess: vitePreprocess(),
    compilerOptions: {
        runes: true,
        preserveComments: false,
        preserveWhitespace: false,
        discloseVersion: false,
        modernAst: true,
    },

    kit: {
        // adapter-auto only supports some environments, see https://svelte.dev/docs/kit/adapter-auto for a list.
        // If your environment is not supported, or you settled on a specific environment, switch out the adapter.
        // See https://svelte.dev/docs/kit/adapters for more information about adapters.
        adapter: htmlMinifierAdaptor(adapter({
            fallback: 'index.html',
            precompress: true,
        })),
        prerender: {
            entries: []
        },
        alias: {
            '$': 'src',
            '$components': 'src/lib/components',
            '$assets': 'src/assets',

        },
    }
};

export default config;
