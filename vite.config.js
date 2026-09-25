import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    rolldownOptions: {
      output: {
        codeSplitting: {
          includeDependenciesRecursively: false,
          groups: [
            { name: 'three-renderer', test: /node_modules[\\/]three[\\/]build[\\/]three\.module\.js/, priority: 20 },
            { name: 'three-core', test: /node_modules[\\/]three[\\/]/, priority: 10 },
            { name: 'react', test: /node_modules[\\/](react|react-dom|scheduler)[\\/]/ },
            { name: 'linear-algebra', test: /node_modules[\\/]ml-/ },
          ],
        },
      },
    },
  },
  server: {
    host: '127.0.0.1',
    proxy: { '/api': { target: process.env.API_PROXY_TARGET || `http://127.0.0.1:${process.env.PORT || 3001}` } },
  },
  preview: {
    proxy: { '/api': { target: process.env.API_PROXY_TARGET || `http://127.0.0.1:${process.env.PORT || 3001}` } },
  },
})
