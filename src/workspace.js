import { forwardKinematics } from './kinematics.js';

function det3(m) {
  return (
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
  );
}

function yoshikawaMeasure(jacobian) {
  // Positional Jacobian: rows 0, 1, 2
  const rows = [jacobian[0], jacobian[1], jacobian[2]];
  const m = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let r1 = 0; r1 < 3; r1++) {
    for (let r2 = r1; r2 < 3; r2++) {
      let sum = 0;
      for (let c = 0; c < rows[0].length; c++) {
        sum += rows[r1][c] * rows[r2][c];
      }
      m[r1][r2] = sum;
      m[r2][r1] = sum;
    }
  }
  const d = det3(m);
  return d > 0 ? Math.sqrt(d) : 0;
}

// Map normalized value [0, 1] to RGB gradient (Blue -> Cyan -> Green -> Yellow -> Red)
function heatColor(t) {
  const clamped = Math.max(0, Math.min(1, t));
  // 4 segments: 0-0.25 (blue to cyan), 0.25-0.5 (cyan to green), 0.5-0.75 (green to yellow), 0.75-1.0 (yellow to red)
  if (clamped < 0.25) {
    const s = clamped / 0.25;
    return [0, s * 0.8, 1];
  } else if (clamped < 0.5) {
    const s = (clamped - 0.25) / 0.25;
    return [0, 0.8 + 0.2 * s, 1 - s];
  } else if (clamped < 0.75) {
    const s = (clamped - 0.5) / 0.25;
    return [s, 1, 0];
  } else {
    const s = (clamped - 0.75) / 0.25;
    return [1, 1 - s * 0.8, 0];
  }
}

/**
 * Monte Carlo workspace point cloud generation
 * @param {Array} joints - Model joint definitions
 * @param {number} count - Number of random configurations to sample (default 2000)
 */
export function generateWorkspaceCloud(joints, count = 2000) {
  const n = Math.max(100, Math.min(10000, count));
  const rawPositions = [];
  const rawMeasures = [];

  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  let maxW = 0;

  // Pseudo-random sampling with deterministic low-discrepancy seed or uniform random
  for (let i = 0; i < n; i++) {
    const sampledJoints = joints.map(j => {
      const u = Math.random();
      const val = j.min + u * (j.max - j.min);
      return { ...j, val };
    });

    const fk = forwardKinematics(sampledJoints);
    const end = fk.positions.at(-1);
    const w = yoshikawaMeasure(fk.jacobian);

    rawPositions.push(end);
    rawMeasures.push(w);

    if (end[0] < minX) minX = end[0];
    if (end[0] > maxX) maxX = end[0];
    if (end[1] < minY) minY = end[1];
    if (end[1] > maxY) maxY = end[1];
    if (end[2] < minZ) minZ = end[2];
    if (end[2] > maxZ) maxZ = end[2];
    if (w > maxW) maxW = w;
  }

  const positions = new Float32Array(n * 3);
  const colors = new Float32Array(n * 3);

  const denom = maxW > 1e-6 ? maxW : 1;
  for (let i = 0; i < n; i++) {
    const pos = rawPositions[i];
    positions[i * 3] = pos[0];
    positions[i * 3 + 1] = pos[1];
    positions[i * 3 + 2] = pos[2];

    const rgb = heatColor(rawMeasures[i] / denom);
    colors[i * 3] = rgb[0];
    colors[i * 3 + 1] = rgb[1];
    colors[i * 3 + 2] = rgb[2];
  }

  const dx = maxX - minX;
  const dy = maxY - minY;
  const dz = maxZ - minZ;
  const boundingVolume = dx * dy * dz;

  return {
    positions,
    colors,
    count: n,
    bounds: { minX, maxX, minY, maxY, minZ, maxZ, dx, dy, dz },
    boundingVolume,
    maxManipulability: maxW,
  };
}
