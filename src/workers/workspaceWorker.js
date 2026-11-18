import { analyzeRobotSingularities, generateWorkspaceCloud } from '../workspace.js';

globalThis.addEventListener('message', (event) => {
  const { id, type, payload } = event.data || {};
  if (id === undefined) return;
  try {
    if (type === 'workspaceCloud') {
      const cloud = generateWorkspaceCloud(payload.joints, payload.count, payload.environment);
      globalThis.postMessage(
        { id, ok: true, result: cloud },
        [cloud.positions.buffer, cloud.colors.buffer]
      );
      return;
    }
    if (type === 'singularities') {
      const report = analyzeRobotSingularities(payload.joints, payload.options);
      globalThis.postMessage({ id, ok: true, result: report });
      return;
    }
    globalThis.postMessage({ id, ok: false, error: `Unknown worker task type: ${type}` });
  } catch (err) {
    globalThis.postMessage({ id, ok: false, error: err?.message || String(err) });
  }
});
