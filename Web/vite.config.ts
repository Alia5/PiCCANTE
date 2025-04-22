import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';
import { compression } from 'vite-plugin-compression2';
import fs from 'fs';
import path from 'path';
import { minify } from 'html-minifier-terser';
import svg from '@poppanator/sveltekit-svg';
import Icons from 'unplugin-icons/vite';
import { visualizer } from 'rollup-plugin-visualizer';


function safeUnlinkSync(file: string) {
    try {
        fs.unlinkSync(file);
    } catch (e) {
        if (e.code !== 'ENOENT') throw e;
    }
}

const removeUncompressed = () => ({
    name: 'keep-only-gzip',
    closeBundle() {
        const outDir = path.resolve('build');
        function recurse(dir: string) {
            for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
                const fullPath = path.join(dir, entry.name);
                if (entry.isDirectory()) {
                    recurse(fullPath);
                    if (fs.readdirSync(fullPath).length === 0) fs.rmdirSync(fullPath);
                } else {
                    if (!entry.name.endsWith('.gz') && !entry.name.endsWith('.br')) {
                        const gzPath = fullPath + '.gz';
                        const brPath = fullPath + '.br';
                        if (fs.existsSync(gzPath)) {

                            const gzStat = fs.statSync(gzPath);
                            const normalStat = fs.statSync(fullPath);
                            if (gzStat.size < normalStat.size) {
                                safeUnlinkSync(fullPath); // remove original
                            } else {
                                safeUnlinkSync(gzPath);   // remove gzip
                            }
                            safeUnlinkSync(brPath);   // remove brotli
                        } else if (fs.existsSync(brPath)) {
                            safeUnlinkSync(brPath);
                        }
                    } else if (entry.name.endsWith('.br')) {
                        safeUnlinkSync(fullPath);
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
                        minifyCSS: true
                    }).then((minified: string) => {
                        fs.writeFileSync(fullPath, minified, 'utf8');
                    });
                }
            }
        }
        if (fs.existsSync(outDir)) recurse(outDir);
    }
});

function copyBuildOutput() {
    return {
        name: 'copy-build-output',
        closeBundle() {
            const srcDir = path.resolve(__dirname, 'build');
            let destDir = process.env.WEB_ASSET_OUT_DIR;
            if (!destDir) {
                destDir = path.resolve(__dirname, '../build/fs_files/web');
                console.warn(
                    '\x1b[33m[copy-build-output] Warning: WEB_ASSET_OUT_DIR not set, using fallback:', destDir, '\x1b[0m'
                );
            } else {
                destDir = path.resolve(destDir);
            }

            if (fs.existsSync(destDir)) {
                console.log(`\n\x1b[33mClearing output directory: ${destDir}\x1b[0m`);
                fs.rmSync(destDir, { recursive: true, force: true });
            }

            fs.mkdirSync(destDir, { recursive: true });

            function copyRecursive(src: string, dest: string) {
                if (!fs.existsSync(src)) return;
                // Don't need to check dest existence now since we just created it
                for (const entry of fs.readdirSync(src, { withFileTypes: true })) {
                    const srcPath = path.join(src, entry.name);
                    const destPath = path.join(dest, entry.name);
                    if (entry.isDirectory()) {
                        fs.mkdirSync(destPath, { recursive: true });
                        copyRecursive(srcPath, destPath);
                    } else {
                        fs.copyFileSync(srcPath, destPath);
                    }
                }
            }

            copyRecursive(srcDir, destDir);
            console.log(`\n\x1b[32mCopied build output to ${destDir}\x1b[0m\n`);
        }
    };
}

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
        copyBuildOutput(),
        process.env?.ANALYZE_BUNDLE ? visualizer(
            {
                emitFile: false,
                filename: 'stats.html'
            }
        ) : null
    ],
    build: {
        sourcemap: mode !== 'production',
        treeshake: true,
        target: 'es2023',
        minify: 'esbuild',
        rollupOptions: {
            output: {
                compact: true
            },
            treeshake: {
            }
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
