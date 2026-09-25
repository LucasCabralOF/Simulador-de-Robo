import sqlite3 from 'sqlite3';
import { fileURLToPath } from 'node:url';
import fs from 'node:fs';
import path from 'node:path';
import { createApp } from './server/app.js';

const dbPath = process.env.DB_PATH || fileURLToPath(new URL('./database.sqlite', import.meta.url));
if (dbPath !== ':memory:') {
  const dir = path.dirname(dbPath);
  if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
}

const db = new sqlite3.Database(dbPath);
const app = await createApp(db);
const port = Number(process.env.PORT || 3001);
const host = process.env.HOST || '127.0.0.1';
const server = app.listen(port, host, () => console.log(`Robotics Simulator running at http://${host}:${port}`));
server.on('error', error => { console.error(error.message); db.close(); process.exitCode = 1; });
for (const signal of ['SIGINT', 'SIGTERM']) {
  process.on(signal, () => server.close(() => db.close(() => process.exit(0))));
}
