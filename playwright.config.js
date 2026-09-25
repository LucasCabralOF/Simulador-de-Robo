import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  testDir: './tests/browser',
  fullyParallel: false,
  workers: 1,
  timeout: 45000,
  outputDir: 'artifacts/browser',
  use: {
    baseURL: 'http://127.0.0.1:5181',
    channel: process.env.PLAYWRIGHT_CHANNEL || undefined,
    launchOptions: { args: ['--enable-unsafe-swiftshader'] },
    screenshot: 'only-on-failure',
  },
  projects: [
    { name: 'desktop', use: { viewport: { width: 1440, height: 1000 } } },
    { name: 'mobile', use: { ...devices['iPhone 13'], defaultBrowserType: 'chromium' } },
  ],
  webServer: [
    { command: 'node server.js', url: 'http://127.0.0.1:3101/api/simulations', env: { PORT: '3101', DB_PATH: ':memory:' }, reuseExistingServer: false },
    { command: process.env.E2E_PRODUCTION ? 'npm run preview -- --host 127.0.0.1 --port 5181 --strictPort' : 'npm run dev:web -- --port 5181 --strictPort', url: 'http://127.0.0.1:5181', env: { PORT: '3101' }, reuseExistingServer: false },
  ],
});
