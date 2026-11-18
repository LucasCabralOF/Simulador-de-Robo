import { DEFAULT_ENVIRONMENT, normalizeModelStyle } from './kinematics';

export const API = (import.meta.env.VITE_API_URL || '/api').replace(/\/$/, '');

const OFFLINE_SIMS_KEY = 'rpr_offline_simulations_v1';

function readOfflineSims() {
  try {
    const raw = window.localStorage.getItem(OFFLINE_SIMS_KEY);
    const parsed = raw ? JSON.parse(raw) : [];
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

function writeOfflineSims(list) {
  try {
    window.localStorage.setItem(OFFLINE_SIMS_KEY, JSON.stringify(list));
  } catch {
    // Ignore storage quota errors
  }
}

function offlineSimRequest(path = '', options = {}) {
  const method = (options.method || 'GET').toUpperCase();
  const list = readOfflineSims();
  if (method === 'GET' && (!path || path === '/')) {
    return list.map(({ id, name, created_at }) => ({ id, name, created_at }));
  }
  if (method === 'POST' && (!path || path === '/')) {
    const body = options.body ? JSON.parse(options.body) : {};
    const entry = {
      id: `local-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
      name: String(body.name || 'Simulação Local').slice(0, 120),
      data: body.data,
      waypoints: body.waypoints,
      toolConfig: body.toolConfig,
      created_at: new Date().toISOString(),
    };
    writeOfflineSims([entry, ...list]);
    return { id: entry.id, name: entry.name };
  }
  const idMatch = path.match(/^\/(.+)$/);
  if (idMatch) {
    const id = decodeURIComponent(idMatch[1]);
    if (method === 'GET') {
      const found = list.find(item => String(item.id) === id);
      if (!found) throw new Error('Simulação local não encontrada.');
      return found;
    }
    if (method === 'DELETE') {
      writeOfflineSims(list.filter(item => String(item.id) !== id));
      return { deleted: true };
    }
  }
  throw new Error('Operação indisponível no modo offline.');
}

/**
 * Performs a simulation API request with transparent offline localStorage fallback.
 * @param {string} [path]
 * @param {RequestInit} [options]
 * @returns {Promise<any>}
 */
export async function request(path = '', options = {}) {
  const canOfflineFallback = path === '' || path === '/' || (/^\/[^/]+$/.test(path) && path !== '/share');
  let res;
  try {
    res = await fetch(`${API}/simulations${path}`, options);
  } catch (error) {
    if (error.name !== 'AbortError' && canOfflineFallback) {
      return offlineSimRequest(path, options);
    }
    throw error;
  }
  let data = null;
  try {
    data = await res.json();
  } catch {
    if (!res.ok && res.status >= 502 && canOfflineFallback) {
      return offlineSimRequest(path, options);
    }
    throw new Error(`Falha na API (${res.status}).`);
  }
  if (!res.ok) throw new Error(data?.error || `Falha na API (${res.status}).`);
  return data;
}

export async function fetchSharedSimulation(slug, signal) {
  const res = await fetch(`${API}/share/${slug}`, { signal });
  return res.json();
}

/**
 * Normalizes workbench floor and 3D obstacle settings.
 * @param {Partial<import('./types.js').EnvironmentConfig> | null | undefined} env
 * @returns {import('./types.js').EnvironmentConfig}
 */
export function normalizeEnvironment(env) {
  if (!env || typeof env !== 'object') {
    return {
      ...DEFAULT_ENVIRONMENT,
      obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
    };
  }
  const pos = Array.isArray(env.obstacle?.position) && env.obstacle.position.length === 3
    ? env.obstacle.position.map((v, i) => (Number.isFinite(Number(v)) ? Number(v) : DEFAULT_ENVIRONMENT.obstacle.position[i]))
    : [...DEFAULT_ENVIRONMENT.obstacle.position];
  return {
    floorEnabled: env.floorEnabled !== undefined ? Boolean(env.floorEnabled) : DEFAULT_ENVIRONMENT.floorEnabled,
    floorZ: Number.isFinite(Number(env.floorZ)) ? Number(env.floorZ) : DEFAULT_ENVIRONMENT.floorZ,
    obstacle: {
      enabled: Boolean(env.obstacle?.enabled),
      shape: env.obstacle?.shape === 'box' ? 'box' : 'sphere',
      position: pos,
      radius: Number.isFinite(Number(env.obstacle?.radius)) ? Math.max(0.1, Number(env.obstacle.radius)) : DEFAULT_ENVIRONMENT.obstacle.radius,
    },
  };
}

/**
 * Deep-clones undo/redo simulator state snapshot.
 * @param {import('./types.js').SimulationSnapshot} snapshot
 * @returns {import('./types.js').SimulationSnapshot}
 */
export function cloneSnapshot({ joints, toolType, toolOffsetRpy, toolGrip, toolScale, modelStyle, environment, waypoints }) {
  return {
    joints: joints.map(j => ({ ...j })),
    toolType,
    toolOffsetRpy: [...toolOffsetRpy],
    toolGrip,
    toolScale,
    modelStyle: normalizeModelStyle(modelStyle),
    environment: normalizeEnvironment(environment),
    waypoints: waypoints.map(w => ({ ...w, q: [...w.q] })),
  };
}
