import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';
import { compression } from 'vite-plugin-compression2'
import fs from 'fs';
import path from 'path';
import { minify } from 'html-minifier-terser';
import svg from '@poppanator/sveltekit-svg'
import Icons from 'unplugin-icons/vite'



const removeUncompressed = () => ({
    name: 'keep-smallest-compressed',
    closeBundle() {
        const outDir = path.resolve('build');
        function recurse(dir: string) {
            for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    recurse(fullPath);
                    if (fs.readdirSync(fullPath).length === 0) fs.rmdirSync(fullPath);
                } else {
                    // Only operate on original files (not .gz/.br)
                    if (!entry.name.endsWith('.gz') && !entry.name.endsWith('.br')) {
                        const gzPath = fullPath + '.gz';
                        const brPath = fullPath + '.br';
                        const candidates = [
                            { path: fullPath, exists: true, size: fs.statSync(fullPath).size },
                            { path: gzPath, exists: fs.existsSync(gzPath), size: fs.existsSync(gzPath) ? fs.statSync(gzPath).size : Infinity },
                            { path: brPath, exists: fs.existsSync(brPath), size: fs.existsSync(brPath) ? fs.statSync(brPath).size : Infinity }
                        ];
                        // Only keep the smallest
                        const smallest = candidates.filter(c => c.exists).sort((a, b) => a.size - b.size)[0];
                        for (const c of candidates) {
                            if (c.exists && c.path !== smallest.path) {
                                fs.unlinkSync(c.path);
                            }
                        }
                    }
                }
            }
        }
        if (fs.existsSync(outDir)) recurse(outDir);
    }
});

const bundleAnalyzer = () => ({
    name: 'bundle-analyzer',
    closeBundle() {
        const outDir = path.resolve('build');
        function getDirSize(dir: string): number {
            let total = 0;
            for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    total += getDirSize(fullPath);
                } else {
                    total += fs.statSync(fullPath).size;
                }
            }
            return total;
        }
        if (fs.existsSync(outDir)) {
            const size = getDirSize(outDir);
            const sizeKB = (size / (1024)).toFixed(2);
            console.log(`\n\x1b[36mFinal bundle size: ${sizeKB} KB (${size} bytes)\x1b[0m\n`);
        }
    }
});

const minifyHtmlPlugin = () => ({
    name: 'minify-html',
    async writeBundle() {
        const outDir = path.resolve('build');
        function recurse(dir: string) {
            for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    recurse(fullPath);
                } else if (entry.name.endsWith('.html')) {
                    const html = fs.readFileSync(fullPath, 'utf8');
                    minify(html, {
                        collapseWhitespace: true,
                        removeComments: true,
                        minifyJS: true,
                        minifyCSS: true,
                    }).then((minified: string) => {
                        fs.writeFileSync(fullPath, minified, 'utf8');
                    });
                }
            }
        }
        if (fs.existsSync(outDir)) recurse(outDir);
    }
});

export default defineConfig(({ mode }) => ({
    plugins: [
        sveltekit(),
        Icons({
            compiler: 'svelte',
            autoInstall: true,
            scale: 1.2,
            transform: (svg) => svg.replace(/^<svg /, '<svg ')
          }),
        svg(),
        minifyHtmlPlugin(),
        // compression({
        //     deleteOriginalAssets: false,
        //     include: /\.*$/,
        //     algorithm: 'brotliCompress',
        //     skipIfLargerOrEqual: true,
        // }),
        removeUncompressed(), // Remove uncompressed AFTER the fact or svelteKit freaks out...
        bundleAnalyzer(),
    ],
    build: {
        sourcemap: mode !== 'production',
        treeshake: true,
        target: 'es2020',
        minify: 'esbuild',
        terserOptions: {
            format: {
                comments: false,
            },
            compress: {
                ecma: 2020,
                passes: 2,
            },
        }
    }
}));

process
    .on('unhandledRejection', (reason, p) => {
        console.error(reason, 'Unhandled Rejection at Promise', p);
        process.exit(1);
    })
    .on('uncaughtException', (err) => {
        console.error(err, 'Uncaught Exception thrown');
        process.exit(1);
    });
