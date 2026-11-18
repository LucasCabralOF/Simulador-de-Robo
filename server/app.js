import express from 'express';
import cors from 'cors';
import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { normalizeJoints, normalizeModelStyle, TOOL_TYPES } from '../src/kinematics.js';

function sanitizeToolConfig(raw) {
  if (!raw || typeof raw !== 'object') return null;
  const toolType = typeof raw.toolType === 'string' && TOOL_TYPES[raw.toolType] ? raw.toolType : 'gripper';
  const rpyRaw = Array.isArray(raw.toolOffsetRpy) && raw.toolOffsetRpy.length === 3 ? raw.toolOffsetRpy : [0, 0, 0];
  const toolOffsetRpy = rpyRaw.map(v => {
    const n = Number(v);
    return Number.isFinite(n) ? Math.max(-180, Math.min(180, n)) : 0;
  });
  const gripNum = Number(raw.toolGrip);
  const toolGrip = Number.isFinite(gripNum) ? Math.max(0, Math.min(100, gripNum)) : 50;
  const result = { toolType, toolOffsetRpy, toolGrip };
  if (raw.toolScale !== undefined && raw.toolScale !== null) {
    const scaleNum = Number(raw.toolScale);
    if (Number.isFinite(scaleNum)) {
      result.toolScale = Math.round(Math.max(0.4, Math.min(4.0, scaleNum)) * 100) / 100;
    }
  }
  if (raw.modelStyle && typeof raw.modelStyle === 'object') {
    result.modelStyle = normalizeModelStyle(raw.modelStyle);
  }
  return result;
}

export async function createApp(db) {
  await new Promise((resolve, reject) => db.run(`CREATE TABLE IF NOT EXISTS simulations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    data TEXT NOT NULL,
    slug TEXT UNIQUE,
    waypoints TEXT,
    tool_config TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
  )`, err => err ? reject(err) : resolve()));

  // Gracefully add optional columns to older databases
  await new Promise(resolve => db.run('ALTER TABLE simulations ADD COLUMN slug TEXT UNIQUE', () => resolve()));
  await new Promise(resolve => db.run('ALTER TABLE simulations ADD COLUMN waypoints TEXT', () => resolve()));
  await new Promise(resolve => db.run('ALTER TABLE simulations ADD COLUMN tool_config TEXT', () => resolve()));

  const app = express();
  app.use(cors());
  app.use(express.json({ limit: '128kb' }));
  const failure = res => res.status(500).json({ error: 'Falha no banco de simulações.' });

  app.param('id', (req, res, next, id) => {
    if (!/^\d+$/.test(id) || !Number.isSafeInteger(Number(id)) || Number(id) < 1) return res.status(400).json({ error: 'Identificador inválido.' });
    next();
  });

  app.get('/api/simulations', (req, res) => {
    db.all('SELECT id, name, slug, created_at FROM simulations ORDER BY created_at DESC, id DESC', [], (err, rows) => err ? failure(res) : res.json(rows));
  });

  app.get('/api/simulations/:id', (req, res) => {
    db.get('SELECT * FROM simulations WHERE id = ?', [req.params.id], (err, row) => {
      if (err) return failure(res);
      if (!row) return res.status(404).json({ error: 'Simulação não encontrada.' });
      try {
        res.json({
          ...row,
          data: normalizeJoints(JSON.parse(row.data)),
          waypoints: row.waypoints ? JSON.parse(row.waypoints) : null,
          toolConfig: row.tool_config ? sanitizeToolConfig(JSON.parse(row.tool_config)) : null,
        });
      } catch {
        res.status(422).json({ error: 'Simulação armazenada contém um modelo inválido.' });
      }
    });
  });

  app.post('/api/simulations', (req, res) => {
    const { name, data, waypoints, toolConfig } = req.body || {};
    if (typeof name !== 'string' || !name.trim() || name.trim().length > 120) return res.status(400).json({ error: 'Nome deve conter de 1 a 120 caracteres.' });
    let joints;
    try { joints = normalizeJoints(data); }
    catch (error) { return res.status(400).json({ error: error.message }); }
    const waypointsJson = Array.isArray(waypoints) ? JSON.stringify(waypoints) : null;
    const sanitizedTool = sanitizeToolConfig(toolConfig);
    const toolConfigJson = sanitizedTool ? JSON.stringify(sanitizedTool) : null;
    db.run('INSERT INTO simulations (name, data, waypoints, tool_config) VALUES (?, ?, ?, ?)', [name.trim(), JSON.stringify(joints), waypointsJson, toolConfigJson], function (err) {
      if (err) return failure(res);
      res.status(201).json({ id: this.lastID, name: name.trim() });
    });
  });

  // Cloud SaaS Sharing endpoint: generates public shareable slug
  app.post('/api/simulations/share', (req, res) => {
    const { name = 'Robô Compartilhado', data, waypoints, toolConfig } = req.body || {};
    let joints;
    try { joints = normalizeJoints(data); }
    catch (error) { return res.status(400).json({ error: error.message }); }
    const slug = 'rob-' + crypto.randomBytes(4).toString('hex');
    const waypointsJson = Array.isArray(waypoints) ? JSON.stringify(waypoints) : null;
    const sanitizedTool = sanitizeToolConfig(toolConfig);
    const toolConfigJson = sanitizedTool ? JSON.stringify(sanitizedTool) : null;
    db.run('INSERT INTO simulations (name, data, slug, waypoints, tool_config) VALUES (?, ?, ?, ?, ?)',
      [(typeof name === 'string' && name.trim()) ? name.trim() : 'Robô Compartilhado', JSON.stringify(joints), slug, waypointsJson, toolConfigJson],
      function (err) {
        if (err) return failure(res);
        res.status(201).json({ id: this.lastID, slug, shareUrl: `/share/${slug}` });
      }
    );
  });

  // Public read-only simulation retrieval by slug
  app.get('/api/share/:slug', (req, res) => {
    const { slug } = req.params;
    if (!/^rob-[0-9a-f]{8}$/.test(slug)) {
      return res.status(400).json({ error: 'Link de compartilhamento inválido.' });
    }
    db.get('SELECT * FROM simulations WHERE slug = ?', [slug], (err, row) => {
      if (err) return failure(res);
      if (!row) return res.status(404).json({ error: 'Modelo compartilhado não encontrado.' });
      try {
        res.json({
          id: row.id,
          name: row.name,
          slug: row.slug,
          data: normalizeJoints(JSON.parse(row.data)),
          waypoints: row.waypoints ? JSON.parse(row.waypoints) : null,
          toolConfig: row.tool_config ? sanitizeToolConfig(JSON.parse(row.tool_config)) : null,
          created_at: row.created_at,
        });
      } catch {
        res.status(422).json({ error: 'Simulação armazenada contém um modelo inválido.' });
      }
    });
  });

  app.delete('/api/simulations/:id', (req, res) => {
    db.run('DELETE FROM simulations WHERE id = ?', [req.params.id], function (err) {
      if (err) return failure(res);
      if (!this.changes) return res.status(404).json({ error: 'Simulação não encontrada.' });
      res.json({ deleted: this.changes });
    });
  });

  const distDir = fileURLToPath(new URL('../dist', import.meta.url));
  if (fs.existsSync(distDir)) {
    app.use(express.static(distDir));
    app.use((req, res, next) => {
      if (req.method !== 'GET') return next();
      if (req.path.startsWith('/api')) return next();
      res.sendFile(path.join(distDir, 'index.html'));
    });
  }

  app.use((err, req, res, next) => {
    if (res.headersSent) return next(err);
    res.status(err.status || 500).json({ error: err.status === 400 ? 'JSON inválido.' : 'Falha ao processar requisição.' });
  });

  return app;
}
