import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig(({ mode }) => {
  const isDev = mode === 'dev'

  return {
    plugins: [svelte({
      compilerOptions: {
        dev: isDev,
      },
      emitCss: true
    })],

    // Tell Vite/Rollup explicitly what the entry HTML is
    build: {
      sourcemap: isDev,
      minify: isDev ? false : "terser",
      outDir: 'dist-' + mode,
      assetsDir: '',
      cssCodeSplit: false,
      rollupOptions: {
        output: {
          // Inline small assets to reduce HTTP requests
          inlineDynamicImports: true,
          manualChunks: undefined
        }
      },
      chunkSizeWarningLimit: Infinity
    },
  }
});
