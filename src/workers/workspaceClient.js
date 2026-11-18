import { analyzeRobotSingularities, generateWorkspaceCloud } from '../workspace.js';

let workerInstance = null;
let workerFailed = false;
let nextRequestId = 1;
const pending = new Map();

/**
 * Checks whether standard module Web Workers are available in the current runtime.
 * @returns {boolean}
 */
export function isWorkerSupported() {
  return typeof Worker !== 'undefined' && !workerFailed;
}

function runSyncFallback(taskType, payload) {
  if (taskType === 'workspaceCloud') {
    return generateWorkspaceCloud(payload.joints, payload.count, payload.environment);
  }
  if (taskType === 'singularities') {
    return analyzeRobotSingularities(payload.joints, payload.options);
  }
  throw new Error(`Unsupported workspace task: ${taskType}`);
}

function flushPendingWithFallback() {
  const entries = Array.from(pending.values());
  pending.clear();
  for (const entry of entries) {
    try {
      entry.resolve(runSyncFallback(entry.taskType, entry.payload));
    } catch (err) {
      entry.reject(err);
    }
  }
}

function ensureWorker() {
  if (!isWorkerSupported()) return null;
  if (workerInstance) return workerInstance;
  try {
    const w = new Worker(new URL('./workspaceWorker.js', import.meta.url), { type: 'module' });
    w.onmessage = (event) => {
      const { id, ok, result, error } = event.data || {};
      const entry = pending.get(id);
      if (!entry) return;
      pending.delete(id);
      if (ok) {
        entry.resolve(result);
      } else {
        try {
          entry.resolve(runSyncFallback(entry.taskType, entry.payload));
        } catch {
          entry.reject(new Error(error || 'Workspace worker failed'));
        }
      }
    };
    w.onerror = () => {
      workerFailed = true;
      terminateWorkspaceWorker();
      flushPendingWithFallback();
    };
    workerInstance = w;
    return workerInstance;
  } catch {
    workerFailed = true;
    return null;
  }
}

function dispatchWorkerTask(taskType, payload) {
  const worker = ensureWorker();
  if (!worker) {
    try {
      return Promise.resolve(runSyncFallback(taskType, payload));
    } catch (err) {
      return Promise.reject(err);
    }
  }
  const id = nextRequestId++;
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject, taskType, payload });
    try {
      worker.postMessage({ id, type: taskType, payload });
    } catch {
      pending.delete(id);
      workerFailed = true;
      terminateWorkspaceWorker();
      try {
        resolve(runSyncFallback(taskType, payload));
      } catch (err) {
        reject(err);
      }
    }
  });
}

/**
 * Generates a 3D workspace point cloud asynchronously in a Web Worker when available,
 * falling back seamlessly to main-thread calculation in Node/non-Worker environments.
 * @param {import('../types.js').Joint[]} joints
 * @param {number} [count]
 * @param {import('../types.js').EnvironmentConfig | null} [environment]
 * @returns {Promise<import('../types.js').WorkspaceCloud>}
 */
export function runWorkspaceCloudAsync(joints, count = 2000, environment = null) {
  return dispatchWorkerTask('workspaceCloud', { joints, count, environment });
}

/**
 * Computes Cartesian axis singularity slices, joint sweeps, and critical 3D poses off the main thread.
 * @param {import('../types.js').Joint[]} joints
 * @param {{ referencePos?: import('../types.js').Vec3 | null, mode?: import('../types.js').TaskMode, environment?: import('../types.js').EnvironmentConfig | null }} [options]
 * @returns {Promise<import('../types.js').SingularityReport>}
 */
export function runSingularityAnalysisAsync(joints, options = {}) {
  return dispatchWorkerTask('singularities', { joints, options });
}

/**
 * Terminates the active workspace Web Worker instance cleanly.
 */
export function terminateWorkspaceWorker() {
  if (workerInstance) {
    workerInstance.terminate();
    workerInstance = null;
  }
}
