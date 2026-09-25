import test from 'node:test';
import assert from 'node:assert/strict';
import sqlite3 from 'sqlite3';
import { createApp } from '../server/app.js';
import { DEFAULT_JOINTS } from '../src/kinematics.js';

test('isolated SQLite CRUD, validation, legacy compatibility and error responses', async (t) => {
  const db = new sqlite3.Database(':memory:');
  const app = await createApp(db);
  const server = await new Promise(resolve => { const instance = app.listen(0, '127.0.0.1', () => resolve(instance)); });
  t.after(async () => {
    await new Promise(resolve => server.close(resolve));
    await new Promise(resolve => db.close(resolve));
  });
  const base = `http://127.0.0.1:${server.address().port}/api/simulations`;
  const save = body => fetch(base, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const original = DEFAULT_JOINTS.map(({ speed: _speed, ...j }) => j);
  const saved = await save({ name: ' Test model ', data: original });
  assert.equal(saved.status, 201);
  const { id } = await saved.json();
  const rows = await (await fetch(base)).json();
  assert.equal(rows.length, 1); assert.equal(rows[0].name, 'Test model');
  const loaded = await (await fetch(`${base}/${id}`)).json();
  assert.deepEqual(loaded.data, DEFAULT_JOINTS);
  for (const body of [{ name: ' ', data: original }, { name: 'bad', data: [] }, { name: 'bad', data: [{ ...original[0], val: 1000 }] }, { name: 'bad', data: [{ ...original[0], alpha: null }] }]) {
    assert.equal((await save(body)).status, 400);
  }
  assert.equal((await fetch(`${base}/invalid`)).status, 400);
  assert.equal((await fetch(`${base}/999`)).status, 404);
  assert.equal((await fetch(base, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{' })).status, 400);
  await new Promise((resolve, reject) => db.run('INSERT INTO simulations(name,data) VALUES (?,?)', ['corrupted', '{'], err => err ? reject(err) : resolve()));
  assert.equal((await fetch(`${base}/${id + 1}`)).status, 422);
  assert.equal((await fetch(`${base}/${id}`, { method: 'DELETE' })).status, 200);
  assert.equal((await fetch(`${base}/${id}`, { method: 'DELETE' })).status, 404);

  // Test Cloud SaaS Share endpoint
  const shareRes = await fetch(`${base}/share`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: 'Robo Publico', data: original, waypoints: [{ id: 1, q: [10, 20] }] }),
  });
  assert.equal(shareRes.status, 201);
  const { slug, shareUrl } = await shareRes.json();
  assert.match(slug, /^rob-[0-9a-f]{8}$/);
  assert.equal(shareUrl, `/share/${slug}`);

  // Test public retrieval by slug
  const apiRoot = `http://127.0.0.1:${server.address().port}/api`;
  const getShare = await fetch(`${apiRoot}/share/${slug}`);
  assert.equal(getShare.status, 200);
  const sharedData = await getShare.json();
  assert.equal(sharedData.name, 'Robo Publico');
  assert.equal(sharedData.slug, slug);
  assert.deepEqual(sharedData.waypoints, [{ id: 1, q: [10, 20] }]);
  assert.deepEqual(sharedData.data, DEFAULT_JOINTS);

  assert.equal((await fetch(`${apiRoot}/share/invalid-slug`)).status, 400);
  assert.equal((await fetch(`${apiRoot}/share/rob-00000000`)).status, 404);
});
