import { detectSelfCollision, forwardKinematics } from './kinematics.js';

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
export function generateWorkspaceCloud(joints, count = 2000, environment = null) {
  const n = Math.max(100, Math.min(10000, count));
  const rawPositions = [];
  const rawMeasures = [];

  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  let maxW = 0;

  // Random sampling filtering out self-colliding and environment-colliding configurations
  let attempts = 0;
  const maxAttempts = n * 4;
  while (rawPositions.length < n && attempts < maxAttempts) {
    attempts++;
    const sampledJoints = joints.map(j => {
      const u = Math.random();
      const val = j.min + u * (j.max - j.min);
      return { ...j, val };
    });

    const fk = forwardKinematics(sampledJoints);
    if (attempts <= n * 3 && detectSelfCollision(sampledJoints, fk, 'gripper', environment).colliding) continue;
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

  const actualCount = rawPositions.length;
  const positions = new Float32Array(actualCount * 3);
  const colors = new Float32Array(actualCount * 3);

  const denom = maxW > 1e-6 ? maxW : 1;
  for (let i = 0; i < actualCount; i++) {
    const pos = rawPositions[i];
    positions[i * 3] = pos[0];
    positions[i * 3 + 1] = pos[1];
    positions[i * 3 + 2] = pos[2];

    const rgb = heatColor(rawMeasures[i] / denom);
    colors[i * 3] = rgb[0];
    colors[i * 3 + 1] = rgb[1];
    colors[i * 3 + 2] = rgb[2];
  }

  const dx = Number.isFinite(minX) ? maxX - minX : 0;
  const dy = Number.isFinite(minY) ? maxY - minY : 0;
  const dz = Number.isFinite(minZ) ? maxZ - minZ : 0;
  const boundingVolume = dx * dy * dz;

  return {
    positions,
    colors,
    count: actualCount,
    bounds: {
      minX: Number.isFinite(minX) ? minX : 0,
      maxX: Number.isFinite(maxX) ? maxX : 0,
      minY: Number.isFinite(minY) ? minY : 0,
      maxY: Number.isFinite(maxY) ? maxY : 0,
      minZ: Number.isFinite(minZ) ? minZ : 0,
      maxZ: Number.isFinite(maxZ) ? maxZ : 0,
      dx,
      dy,
      dz,
    },
    boundingVolume,
    maxManipulability: maxW,
  };
}

const HALTON_PRIMES = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89];

function halton(index, base) {
  let result = 0;
  let f = 1 / base;
  let i = index;
  while (i > 0) {
    result += f * (i % base);
    i = Math.floor(i / base);
    f /= base;
  }
  return result;
}

const boundsCache = new Map();

function solveSmallSystem(A, b) {
  const m = [
    [A[0][0], A[0][1], A[0][2], b[0]],
    [A[1][0], A[1][1], A[1][2], b[1]],
    [A[2][0], A[2][1], A[2][2], b[2]],
  ];
  for (let col = 0; col < 3; col++) {
    let pivot = col;
    for (let r = col + 1; r < 3; r++) {
      if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
    }
    if (pivot !== col) {
      const tmp = m[col];
      m[col] = m[pivot];
      m[pivot] = tmp;
    }
    const diag = m[col][col];
    if (Math.abs(diag) < 1e-12) continue;
    for (let r = col + 1; r < 3; r++) {
      const factor = m[r][col] / diag;
      for (let c = col; c < 4; c++) m[r][c] -= factor * m[col][c];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let sum = m[r][3];
    for (let c = r + 1; c < 3; c++) sum -= m[r][c] * x[c];
    x[r] = Math.abs(m[r][r]) < 1e-12 ? 0 : sum / m[r][r];
  }
  return x;
}

function solveDlsStep(joints, fk, vDes, taskDims, refLen) {
  const n = joints.length;
  const J = fk.jacobian;
  const deg = Math.PI / 180;
  const scales = joints.map(j => (j.type === 'P' ? refLen : 1));
  const blocked = new Array(n).fill(false);
  const lambda2 = 0.002 * refLen * refLen;
  let dq = new Array(n).fill(0);

  for (let pass = 0; pass < 3; pass++) {
    const A = [
      [lambda2, 0, 0],
      [0, lambda2, 0],
      [0, 0, lambda2],
    ];
    for (let r = 0; r < taskDims; r++) {
      for (let c = r; c < taskDims; c++) {
        let dot = 0;
        for (let j = 0; j < n; j++) {
          if (blocked[j]) continue;
          const s = scales[j];
          dot += J[r][j] * s * (J[c][j] * s);
        }
        A[r][c] += dot;
        if (c !== r) A[c][r] += dot;
      }
    }
    const rhs = [
      taskDims > 0 ? vDes[0] : 0,
      taskDims > 1 ? vDes[1] : 0,
      taskDims > 2 ? vDes[2] : 0,
    ];
    const w = solveSmallSystem(A, rhs);
    let newlyBlocked = false;
    for (let j = 0; j < n; j++) {
      if (blocked[j]) {
        dq[j] = 0;
        continue;
      }
      let normStep = 0;
      for (let r = 0; r < taskDims; r++) {
        normStep += J[r][j] * scales[j] * w[r];
      }
      const rawRate = normStep * scales[j];
      const rateDegOrU = joints[j].type === 'R' ? rawRate / deg : rawRate;
      dq[j] = rateDegOrU;
      if ((joints[j].val <= joints[j].min + 1e-7 && rateDegOrU < -1e-9)
        || (joints[j].val >= joints[j].max - 1e-7 && rateDegOrU > 1e-9)) {
        blocked[j] = true;
        dq[j] = 0;
        newlyBlocked = true;
      }
    }
    if (!newlyBlocked) break;
  }

  let maxRatio = 1;
  for (let j = 0; j < n; j++) {
    const maxAllowed = joints[j].type === 'R' ? 14 : Math.max(0.05, refLen * 0.2);
    maxRatio = Math.max(maxRatio, Math.abs(dq[j]) / maxAllowed);
  }
  const scale = maxRatio > 1 ? 1 / maxRatio : 1;
  return dq.map(v => v * scale);
}

function marchAxisBound(seedQ, axis, sign, targetRef, otherAxes, jointsTemplate, taskDims, refLen, stepSize, environment = null) {
  let curr = jointsTemplate.map((j, k) => ({
    ...j,
    val: Math.max(j.min, Math.min(j.max, seedQ[k])),
  }));
  let currFk = forwardKinematics(curr, { includeOrientation: false });
  if (detectSelfCollision(curr, currFk, 'gripper', environment).colliding) return null;

  let bestCoord = null;
  let bestState = null;
  const transTol = Math.max(0.02, refLen * 0.006);
  const initialErr = Math.hypot(...otherAxes.map(b => currFk.positions.at(-1)[b] - targetRef[b]));
  if (initialErr <= transTol) {
    bestCoord = currFk.positions.at(-1)[axis];
    bestState = { joints: curr, fk: currFk, err: initialErr };
  }

  let currStep = stepSize;
  let stallCount = 0;

  for (let iter = 0; iter < 38; iter++) {
    const pCurr = currFk.positions.at(-1);
    const tErr = Math.hypot(...otherAxes.map(b => targetRef[b] - pCurr[b]));
    const vDes = [0, 0, 0];

    for (const b of otherAxes) {
      vDes[b] = targetRef[b] - pCurr[b];
    }

    if (tErr <= transTol) {
      const alignFactor = Math.max(0.35, 1 - (tErr / transTol) * 0.65);
      vDes[axis] = sign * currStep * alignFactor;
    } else {
      // Pure transverse projection onto the slice before marching along the axis
      vDes[axis] = 0;
    }

    const dq = solveDlsStep(curr, currFk, vDes, taskDims, refLen);
    if (dq.every(v => Math.abs(v) < 1e-7)) break;

    let accepted = false;
    for (const frac of [1, 0.5, 0.25, 0.125, 0.0625]) {
      const cand = curr.map((j, k) => ({
        ...j,
        val: Math.max(j.min, Math.min(j.max, j.val + dq[k] * frac)),
      }));
      const candFk = forwardKinematics(cand, { includeOrientation: false });
      const pNext = candFk.positions.at(-1);
      const nextTErr = Math.hypot(...otherAxes.map(b => pNext[b] - targetRef[b]));
      const progress = (pNext[axis] - pCurr[axis]) * sign;

      if (tErr <= transTol) {
        if (nextTErr > transTol * 2.5) continue;
        if (progress <= 0 && nextTErr > tErr) continue;
      } else {
        if (nextTErr >= tErr - 1e-5) continue;
      }
      if (detectSelfCollision(cand, candFk, 'gripper', environment).colliding) continue;

      curr = cand;
      currFk = candFk;
      accepted = true;
      if (nextTErr <= transTol) {
        const val = pNext[axis];
        if (bestCoord === null || (sign > 0 ? val > bestCoord : val < bestCoord)) {
          bestCoord = val;
          bestState = { joints: curr, fk: currFk, err: nextTErr };
        }
      }
      if (tErr <= transTol) {
        if (progress < currStep * 0.05) {
          stallCount++;
          currStep *= 0.45;
        } else if (frac < 0.5) {
          stallCount = 0;
          currStep = Math.max(stepSize * 0.02, currStep * 0.65);
        } else {
          stallCount = 0;
        }
      }
      break;
    }

    if (!accepted || stallCount >= 4 || currStep < stepSize * 0.01) break;
  }

  // Project best boundary point squarely back onto the exact transverse slice line
  if (bestState && bestState.err > 0.004) {
    let proj = bestState.joints;
    let projFk = bestState.fk;
    let projErr = bestState.err;
    for (let fix = 0; fix < 4 && projErr > 0.002; fix++) {
      const p = projFk.positions.at(-1);
      const vFix = [0, 0, 0];
      for (const b of otherAxes) {
        vFix[b] = targetRef[b] - p[b];
      }
      const dqFix = solveDlsStep(proj, projFk, vFix, taskDims, refLen);
      if (dqFix.every(v => Math.abs(v) < 1e-7)) break;
      let improved = false;
      for (const frac of [1, 0.5, 0.25]) {
        const cand = proj.map((j, k) => ({
          ...j,
          val: Math.max(j.min, Math.min(j.max, j.val + dqFix[k] * frac)),
        }));
        const candFk = forwardKinematics(cand, { includeOrientation: false });
        const nextErr = Math.hypot(...otherAxes.map(b => candFk.positions.at(-1)[b] - targetRef[b]));
        if (nextErr < projErr - 1e-5 && !detectSelfCollision(cand, candFk, 'gripper', environment).colliding) {
          proj = cand;
          projFk = candFk;
          projErr = nextErr;
          improved = true;
          break;
        }
      }
      if (!improved) break;
    }
    if (projErr < bestState.err) {
      bestCoord = projFk.positions.at(-1)[axis];
    }
  }

  return bestCoord;
}

function getStructuralEnvelope(joints, environment = null) {
  const envKey = environment
    ? JSON.stringify({
        fe: Boolean(environment.floorEnabled),
        fz: environment.floorZ,
        ob: environment.obstacle?.enabled ? [environment.obstacle.shape, environment.obstacle.position, environment.obstacle.radius] : null,
      })
    : '';
  const cacheKey = `${joints.map(j => `${j.type}:${j.a}:${j.alpha}:${j.d}:${j.theta}:${j.min}:${j.max}`).join('|')}@${envKey}`;
  const cached = boundsCache.get(cacheKey);
  if (cached) return cached;

  const evalSample = q => {
    const sampled = joints.map((j, k) => ({ ...j, val: q[k] }));
    const fk = forwardKinematics(sampled, { includeOrientation: false });
    return { pos: fk.positions.at(-1), valid: !detectSelfCollision(sampled, fk, 'gripper', environment).colliding };
  };

  const minQ = joints.map(j => j.min);
  const maxQ = joints.map(j => j.max);
  const midQ = joints.map(j => (j.min + j.max) / 2);

  const minPos = [Infinity, Infinity, Infinity];
  const maxPos = [-Infinity, -Infinity, -Infinity];
  const minSeeds = [midQ, midQ, midQ];
  const maxSeeds = [midQ, midQ, midQ];
  const validSamples = [];

  const record = (q, requireValid = true, keepSample = false) => {
    const { pos: p, valid } = evalSample(q);
    if (requireValid && !valid) return p;
    if (keepSample && valid) validSamples.push({ q, pos: p });
    for (let axis = 0; axis < 3; axis++) {
      if (p[axis] < minPos[axis]) { minPos[axis] = p[axis]; minSeeds[axis] = q; }
      if (p[axis] > maxPos[axis]) { maxPos[axis] = p[axis]; maxSeeds[axis] = q; }
    }
    return p;
  };

  record(midQ, true, true);
  record(minQ, true, true);
  record(maxQ, true, true);

  // Deterministic low-discrepancy Halton sweep across [min, max]^n
  const sampleCount = 180;
  for (let s = 1; s <= sampleCount; s++) {
    const q = joints.map((j, idx) => {
      const prime = HALTON_PRIMES[idx % HALTON_PRIMES.length];
      const u = halton(s, prime);
      return j.min + u * (j.max - j.min);
    });
    record(q, true, true);
  }

  if (!Number.isFinite(minPos[0])) {
    const fallbackQ = joints.map(j => j.val);
    const p = record(fallbackQ, false, false);
    validSamples.push({ q: fallbackQ, pos: p });
  }

  // Coordinate ascent/descent refinement along each axis from both best Halton seed and midpoint
  const stepFractions = [0.5, 0.25, 0.125, 0.05, 0.015, 0.004, 0.001];
  const refineAxis = (startQ, axis, sign) => {
    const q = [...startQ];
    const startSample = evalSample(q);
    let bestVal = startSample.pos[axis] * sign;
    for (const frac of stepFractions) {
      let improved = true;
      let rounds = 0;
      while (improved && rounds < 3) {
        improved = false;
        rounds++;
        for (let j = 0; j < joints.length; j++) {
          const span = joints[j].max - joints[j].min;
          if (span <= 0) continue;
          const delta = span * frac;
          for (const dir of [-1, 1]) {
            const candidateVal = Math.max(joints[j].min, Math.min(joints[j].max, q[j] + dir * delta));
            if (candidateVal === q[j]) continue;
            const prev = q[j];
            q[j] = candidateVal;
            const res = evalSample(q);
            const score = res.pos[axis] * sign;
            if (res.valid && score > bestVal + 1e-9) {
              bestVal = score;
              improved = true;
            } else {
              q[j] = prev;
            }
          }
        }
      }
    }
    record(q, true, true);
  };

  for (let axis = 0; axis < 3; axis++) {
    const minSeed = minSeeds[axis];
    const maxSeed = maxSeeds[axis];
    refineAxis(minSeed, axis, -1);
    refineAxis(midQ, axis, -1);
    refineAxis(maxSeed, axis, 1);
    refineAxis(midQ, axis, 1);
  }

  const clean = v => (Math.abs(v) < 1e-9 ? 0 : v);
  const min = minPos.map(clean);
  const max = maxPos.map(clean);
  const span = [0, 1, 2].map(i => {
    const diff = max[i] - min[i];
    if (diff < 1e-6) {
      max[i] = min[i];
      return 0;
    }
    return diff;
  });

  const entry = { globalBounds: { min, max, span }, validSamples };
  if (boundsCache.size >= 16) {
    boundsCache.delete(boundsCache.keys().next().value);
  }
  boundsCache.set(cacheKey, entry);
  return entry;
}

/**
 * Deterministically estimates the reachable Cartesian bounds [min, max] per axis.
 * When `referencePos` is supplied, adapts each active axis interval to the robot's
 * current pose, transverse coordinates, joint [min, max] limits, and self-collision rules.
 */
export function computeCartesianBounds(joints, referencePos = null, mode = 'position', environment = null) {
  if (!Array.isArray(joints) || joints.length === 0) {
    return { min: [0, 0, 0], max: [0, 0, 0], span: [0, 0, 0] };
  }

  const { globalBounds, validSamples } = getStructuralEnvelope(joints, environment);
  if (!Array.isArray(referencePos) || referencePos.length !== 3 || !referencePos.every(Number.isFinite)) {
    return globalBounds;
  }

  const taskDims = mode === 'xy' ? 2 : 3;
  const currentQ = joints.map(j => j.val);
  const refLen = Math.max(
    1,
    joints.reduce((acc, j) => acc + Math.abs(j.a) + Math.abs(j.type === 'P' ? j.max : j.d), 0) / Math.max(1, joints.length)
  );

  const min = [...globalBounds.min];
  const max = [...globalBounds.max];
  const span = [...globalBounds.span];

  for (let axis = 0; axis < taskDims; axis++) {
    const gSpan = globalBounds.span[axis];
    if (gSpan < 1e-5) {
      min[axis] = referencePos[axis];
      max[axis] = referencePos[axis];
      span[axis] = 0;
      continue;
    }

    const otherAxes = [];
    for (let b = 0; b < taskDims; b++) {
      if (b !== axis) otherAxes.push(b);
    }

    const stepSize = Math.max(0.08, Math.min(refLen * 0.35, gSpan * 0.045));
    let lo = referencePos[axis];
    let hi = referencePos[axis];

    const recordBound = (val) => {
      if (val === null || !Number.isFinite(val)) return;
      if (val < lo) lo = val;
      if (val > hi) hi = val;
    };

    // 1. Directional kinematic ray-marching from current pose
    recordBound(marchAxisBound(currentQ, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));
    recordBound(marchAxisBound(currentQ, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));

    // 2. Best Halton seeds on opposite sides of workspace center near transverse slice (escapes joint wrap / central column)
    let negSeed = null;
    let negScore = Infinity;
    let posSeed = null;
    let posScore = Infinity;
    const gCenter = (globalBounds.min[axis] + globalBounds.max[axis]) / 2;

    for (const s of validSamples) {
      const tErr = Math.hypot(...otherAxes.map(b => s.pos[b] - referencePos[b]));
      if (s.pos[axis] <= gCenter) {
        const sc = tErr + 0.02 * (s.pos[axis] - globalBounds.min[axis]);
        if (sc < negScore) { negScore = sc; negSeed = s.q; }
      }
      if (s.pos[axis] >= gCenter) {
        const sc = tErr + 0.02 * (globalBounds.max[axis] - s.pos[axis]);
        if (sc < posScore) { posScore = sc; posSeed = s.q; }
      }
    }

    // If base joint is revolute, also mirror across XY slice when searching opposite-side branch
    const mirrorSeed = (axis < 2 && joints[0]?.type === 'R')
      ? currentQ.map((v, idx) => (idx === 0 ? v + 180 : v)).map((v, idx) => {
          const j = joints[idx];
          if (v > j.max) return Math.max(j.min, v - 360);
          return v;
        })
      : null;

    if (negSeed) recordBound(marchAxisBound(negSeed, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));
    if (posSeed) recordBound(marchAxisBound(posSeed, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));
    if (mirrorSeed && mirrorSeed[0] >= joints[0].min && mirrorSeed[0] <= joints[0].max) {
      if (referencePos[axis] >= 0 && lo >= referencePos[axis] - 1e-3) {
        recordBound(marchAxisBound(mirrorSeed, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));
      }
      if (referencePos[axis] <= 0 && hi <= referencePos[axis] + 1e-3) {
        recordBound(marchAxisBound(mirrorSeed, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment));
      }
    }

    lo = Math.max(globalBounds.min[axis], Math.min(referencePos[axis], lo));
    hi = Math.min(globalBounds.max[axis], Math.max(referencePos[axis], hi));

    const clean = v => (Math.abs(v) < 1e-9 ? 0 : v);
    min[axis] = clean(lo);
    max[axis] = clean(hi);
    span[axis] = Math.max(0, max[axis] - min[axis]);
    if (span[axis] < 1e-6) {
      max[axis] = min[axis];
      span[axis] = 0;
    }
  }

  return { min, max, span };
}
