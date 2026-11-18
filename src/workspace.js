import { detectSelfCollision, forwardKinematics } from './kinematics.js';

function det3(m) {
  return (
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
  );
}

function minor2x2Sum(m) {
  return (
    (m[0][0] * m[1][1] - m[0][1] * m[1][0])
    + (m[0][0] * m[2][2] - m[0][2] * m[2][0])
    + (m[1][1] * m[2][2] - m[1][2] * m[2][1])
  );
}

function yoshikawaMeasures(jacobian) {
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
  const p2 = minor2x2Sum(m);
  return {
    w3: d > 0 ? Math.sqrt(d) : 0,
    w2: p2 > 0 ? Math.sqrt(p2) : 0,
  };
}

// Map normalized manipulability t in [0, 1] to RGB gradient: Blue (near singularity) -> Cyan -> Green (high dexterity)
export function heatColor(t) {
  const clamped = Math.max(0, Math.min(1, t));
  const lerpColor = (a, b, s) => [
    a[0] + (b[0] - a[0]) * s,
    a[1] + (b[1] - a[1]) * s,
    a[2] + (b[2] - a[2]) * s,
  ];
  const blue = [0.10, 0.46, 1.00];
  const cyan = [0.00, 0.84, 0.90];
  const green = [0.14, 0.92, 0.45];

  if (clamped < 0.5) return lerpColor(blue, cyan, clamped / 0.5);
  return lerpColor(cyan, green, (clamped - 0.5) / 0.5);
}

/**
 * Monte Carlo workspace point cloud generation
 * @param {Array} joints - Model joint definitions
 * @param {number} count - Number of random configurations to sample (default 2000)
 */
export function generateWorkspaceCloud(joints, count = 2000, environment = null) {
  const n = Math.max(100, Math.min(10000, count));
  const rawPositions = [];
  const rawMeasures3 = [];
  const rawMeasures2 = [];

  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  let maxW3 = 0;
  let maxW2 = 0;

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
    const { w3, w2 } = yoshikawaMeasures(fk.jacobian);

    rawPositions.push(end);
    rawMeasures3.push(w3);
    rawMeasures2.push(w2);

    if (end[0] < minX) minX = end[0];
    if (end[0] > maxX) maxX = end[0];
    if (end[1] < minY) minY = end[1];
    if (end[1] > maxY) maxY = end[1];
    if (end[2] < minZ) minZ = end[2];
    if (end[2] > maxZ) maxZ = end[2];
    if (w3 > maxW3) maxW3 = w3;
    if (w2 > maxW2) maxW2 = w2;
  }

  const use3D = maxW3 > 1e-6;
  const rawMeasures = use3D ? rawMeasures3 : rawMeasures2;
  const maxW = use3D ? maxW3 : maxW2;

  const actualCount = rawPositions.length;
  const positions = new Float32Array(actualCount * 3);
  const colors = new Float32Array(actualCount * 3);

  const denom = maxW > 1e-6 ? maxW : 1;
  for (let i = 0; i < actualCount; i++) {
    const pos = rawPositions[i];
    positions[i * 3] = pos[0];
    positions[i * 3 + 1] = pos[1];
    positions[i * 3 + 2] = pos[2];

    const normalized = Math.pow(Math.max(0, Math.min(1, rawMeasures[i] / denom)), 0.65);
    const rgb = heatColor(normalized);
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

function marchAxisBound(seedQ, axis, sign, targetRef, otherAxes, jointsTemplate, taskDims, refLen, stepSize, environment = null, use3D = true) {
  let curr = jointsTemplate.map((j, k) => ({
    ...j,
    val: Math.max(j.min, Math.min(j.max, seedQ[k])),
  }));
  let currFk = forwardKinematics(curr, { includeOrientation: false });
  if (detectSelfCollision(curr, currFk, 'gripper', environment).colliding) return { bestCoord: null, samples: [] };

  const samples = [];
  const pushSample = (stateJoints, stateFk) => {
    const { w3, w2 } = yoshikawaMeasures(stateFk.jacobian);
    const w = use3D ? w3 : w2;
    const atLimit = stateJoints.some(j => (j.max - j.min) > 1e-6 && (j.val <= j.min + 1e-4 || j.val >= j.max - 1e-4));
    samples.push({ coord: stateFk.positions.at(-1)[axis], w, atLimit });
  };

  let bestCoord = null;
  let bestState = null;
  const transTol = Math.max(0.02, refLen * 0.006);
  const initialErr = Math.hypot(...otherAxes.map(b => currFk.positions.at(-1)[b] - targetRef[b]));
  if (initialErr <= transTol) {
    bestCoord = currFk.positions.at(-1)[axis];
    bestState = { joints: curr, fk: currFk, err: initialErr };
    pushSample(curr, currFk);
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
        pushSample(curr, currFk);
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
      pushSample(proj, projFk);
    }
  }

  return { bestCoord, samples };
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

  let maxW3 = 0;
  let maxW2 = 0;

  const evalSample = (q, measureManip = false) => {
    const sampled = joints.map((j, k) => ({ ...j, val: q[k] }));
    const fk = forwardKinematics(sampled, { includeOrientation: false });
    const valid = !detectSelfCollision(sampled, fk, 'gripper', environment).colliding;
    let w3 = 0;
    let w2 = 0;
    if (measureManip && valid) {
      const m = yoshikawaMeasures(fk.jacobian);
      w3 = m.w3;
      w2 = m.w2;
      if (w3 > maxW3) maxW3 = w3;
      if (w2 > maxW2) maxW2 = w2;
    }
    return { pos: fk.positions.at(-1), valid, w3, w2 };
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
    const { pos: p, valid, w3, w2 } = evalSample(q, true);
    if (requireValid && !valid) return p;
    if (keepSample && valid) validSamples.push({ q, pos: p, w3, w2 });
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

  const use3D = maxW3 > 1e-6;
  const globalMaxW = Math.max(1e-6, use3D ? maxW3 : maxW2);
  const entry = { globalBounds: { min, max, span }, validSamples, use3D, globalMaxW };
  if (boundsCache.size >= 16) {
    boundsCache.delete(boundsCache.keys().next().value);
  }
  boundsCache.set(cacheKey, entry);
  return entry;
}

function extractAxisSingularities({ axis, samples, lo, hi, spanAxis, globalMaxW, otherAxes, referencePos, validSamples, joints, use3D }) {
  if (spanAxis < 1e-5 || !Number.isFinite(lo) || !Number.isFinite(hi)) return [];
  const all = samples.map(s => ({ ...s, onSlice: true }));
  const sliceTol = Math.max(0.25, spanAxis * 0.05);
  for (const s of validSamples) {
    const tErr = Math.hypot(...otherAxes.map(b => s.pos[b] - referencePos[b]));
    if (tErr > sliceTol) continue;
    const w = use3D ? s.w3 : s.w2;
    const atLimit = joints.some((j, k) => (j.max - j.min) > 1e-6 && (s.q[k] <= j.min + 1e-4 || s.q[k] >= j.max - 1e-4));
    all.push({ coord: s.pos[axis], w, atLimit, onSlice: false });
  }

  const filtered = all
    .filter(s => Number.isFinite(s.coord) && Number.isFinite(s.w) && s.coord >= lo - 1e-4 && s.coord <= hi + 1e-4)
    .sort((a, b) => a.coord - b.coord);
  if (filtered.length === 0) return [];

  const localMaxW = filtered.reduce((acc, s) => Math.max(acc, s.w), 0);
  const refMax = Math.max(globalMaxW * 0.65, localMaxW, 1e-6);
  const cleanVal = v => {
    const r = Math.round(v * 100) / 100;
    return Object.is(r, -0) ? 0 : r;
  };

  const numBins = 32;
  const binSize = spanAxis / numBins;
  const bins = new Array(numBins).fill(null);
  for (const s of filtered) {
    const idx = Math.max(0, Math.min(numBins - 1, Math.floor((s.coord - lo) / binSize)));
    const b = bins[idx];
    if (!b) {
      bins[idx] = {
        minW: s.w,
        minSliceW: s.onSlice ? s.w : Infinity,
        maxW: s.w,
        minCoord: s.coord,
        maxCoord: s.coord,
        atLimit: Boolean(s.atLimit),
        hasSlice: Boolean(s.onSlice),
      };
    } else {
      if (s.coord < b.minCoord) b.minCoord = s.coord;
      if (s.coord > b.maxCoord) b.maxCoord = s.coord;
      if (s.w > b.maxW) b.maxW = s.w;
      if (s.onSlice) {
        b.hasSlice = true;
        if (s.w < b.minSliceW) {
          b.minSliceW = s.w;
          b.atLimit = Boolean(s.atLimit);
        }
      } else if (!b.hasSlice && s.w < b.minW) {
        b.atLimit = Boolean(s.atLimit);
      }
      if (s.w < b.minW) b.minW = s.w;
    }
  }

  let firstOccupied = -1;
  let lastOccupied = -1;
  for (let i = 0; i < numBins; i++) {
    if (bins[i]) {
      bins[i].effectiveW = bins[i].hasSlice ? bins[i].minSliceW : bins[i].minW;
      if (firstOccupied === -1) firstOccupied = i;
      lastOccupied = i;
    }
  }
  if (firstOccupied === -1) return [];

  const singularFlags = new Array(numBins).fill(false);
  const binEffW = new Array(numBins).fill(Infinity);
  const binStartCoord = new Array(numBins).fill(0).map((_, i) => lo + i * binSize);
  const binEndCoord = new Array(numBins).fill(0).map((_, i) => lo + (i + 1) * binSize);

  for (let i = firstOccupied; i <= lastOccupied; i++) {
    const cur = bins[i];
    if (!cur) continue;
    binEffW[i] = cur.effectiveW;
    binStartCoord[i] = Math.max(lo, cur.minCoord);
    binEndCoord[i] = Math.min(hi, cur.maxCoord);

    const isOuter = i === firstOccupied || i === lastOccupied;
    if (isOuter && (cur.effectiveW <= refMax * 0.35 || (!cur.atLimit && cur.effectiveW <= refMax * 0.55))) {
      singularFlags[i] = true;
      continue;
    }
    if (cur.effectiveW <= refMax * 0.22) {
      singularFlags[i] = true;
      continue;
    }
    let leftPeak = 0;
    for (let k = Math.max(firstOccupied, i - 5); k < i; k++) {
      if (bins[k]) leftPeak = Math.max(leftPeak, bins[k].maxW);
    }
    let rightPeak = 0;
    for (let k = i + 1; k <= Math.min(lastOccupied, i + 5); k++) {
      if (bins[k]) rightPeak = Math.max(rightPeak, bins[k].maxW);
    }
    if (leftPeak > 1e-6 && rightPeak > 1e-6 && cur.effectiveW <= Math.min(leftPeak, rightPeak) * 0.52) {
      singularFlags[i] = true;
    }
  }

  // Detect contiguous interior dead-zone / branch-separation gaps strictly between occupied regions
  let scan = firstOccupied;
  while (scan <= lastOccupied) {
    if (bins[scan]) {
      scan++;
      continue;
    }
    const gStart = scan;
    while (scan <= lastOccupied && !bins[scan]) scan++;
    const gEnd = scan - 1;
    const leftIdx = gStart - 1;
    const rightIdx = scan;
    if (leftIdx >= firstOccupied && rightIdx <= lastOccupied) {
      const gapWidth = bins[rightIdx].minCoord - bins[leftIdx].maxCoord;
      const edgeMinW = Math.min(bins[leftIdx].effectiveW, bins[rightIdx].effectiveW);
      if (
        (gEnd - gStart + 1 >= 2 && gapWidth >= Math.max(0.35, spanAxis * 0.08))
        || (gapWidth >= Math.max(0.2, spanAxis * 0.05) && edgeMinW <= refMax * 0.55)
      ) {
        for (let b = gStart; b <= gEnd; b++) {
          singularFlags[b] = true;
          binEffW[b] = Math.min(binEffW[b], edgeMinW);
        }
        binStartCoord[gStart] = Math.max(lo, bins[leftIdx].maxCoord);
        binEndCoord[gEnd] = Math.min(hi, bins[rightIdx].minCoord);
      }
    }
  }

  // Bridge isolated 1-bin gaps flanked by singular bins so bands stay continuous
  for (let i = firstOccupied + 1; i < lastOccupied; i++) {
    if (!singularFlags[i] && singularFlags[i - 1] && singularFlags[i + 1]) {
      singularFlags[i] = true;
      binEffW[i] = bins[i] ? bins[i].effectiveW : Math.min(binEffW[i - 1], binEffW[i + 1]);
    }
  }

  const minBandSpan = Math.min(spanAxis, Math.max(binSize * 0.55, spanAxis * 0.03));
  const rawIntervals = [];
  let idx = 0;
  while (idx < numBins) {
    if (!singularFlags[idx]) {
      idx++;
      continue;
    }
    const rStart = idx;
    let runMinW = Infinity;
    while (idx < numBins && singularFlags[idx]) {
      if (Number.isFinite(binEffW[idx]) && binEffW[idx] < runMinW) {
        runMinW = binEffW[idx];
      }
      idx++;
    }
    const rEnd = idx - 1;
    if (!Number.isFinite(runMinW)) runMinW = 0;

    let ivStart;
    let ivEnd;
    if (rStart === rEnd && rStart === firstOccupied && lo + spanAxis * 0.04 >= binStartCoord[rStart]) {
      ivStart = lo;
      ivEnd = Math.min(hi, lo + minBandSpan);
    } else if (rStart === rEnd && rEnd === lastOccupied && hi - spanAxis * 0.04 <= binEndCoord[rEnd]) {
      ivStart = Math.max(lo, hi - minBandSpan);
      ivEnd = hi;
    } else {
      const startCoord = rStart === firstOccupied ? lo : Math.max(lo, Math.min(binStartCoord[rStart], lo + rStart * binSize));
      const endCoord = rEnd === lastOccupied ? hi : Math.min(hi, Math.max(binEndCoord[rEnd], lo + (rEnd + 1) * binSize));
      ivStart = Math.max(lo, Math.min(hi, startCoord));
      ivEnd = Math.max(ivStart, Math.min(hi, endCoord));
      if (ivEnd - ivStart < minBandSpan) {
        const mid = (ivStart + ivEnd) / 2;
        ivStart = Math.max(lo, mid - minBandSpan / 2);
        ivEnd = Math.min(hi, ivStart + minBandSpan);
        ivStart = Math.max(lo, ivEnd - minBandSpan);
      }
    }

    rawIntervals.push({ start: ivStart, end: ivEnd, w: runMinW });
  }

  // Merge intervals separated by very narrow gaps (< 3% span)
  const merged = [];
  const mergeGap = spanAxis * 0.03;
  for (const iv of rawIntervals) {
    const prev = merged.at(-1);
    if (prev && iv.start - prev.end <= mergeGap) {
      prev.end = Math.max(prev.end, iv.end);
      prev.w = Math.min(prev.w, iv.w);
    } else {
      merged.push({ ...iv });
    }
  }

  return merged.slice(0, 6).map(iv => {
    const s0 = cleanVal(Math.max(lo, Math.min(hi, iv.start)));
    const s1 = cleanVal(Math.max(s0, Math.min(hi, iv.end)));
    const val = cleanVal((s0 + s1) / 2);
    const kind = (s0 <= lo + spanAxis * 0.04 || s1 >= hi - spanAxis * 0.04) ? 'boundary' : 'interior';
    const pctW = Math.round(Math.max(0, Math.min(100, (iv.w / refMax) * 100)));
    const wide = s1 - s0 >= Math.max(0.15, spanAxis * 0.04);
    const label = wide
      ? `Faixa singular ${kind === 'boundary' ? 'de fronteira' : 'interna'} [${s0.toFixed(2)}, ${s1.toFixed(2)}] u (w mín=${pctW}%)`
      : `${kind === 'boundary' ? 'Singularidade de fronteira' : 'Singularidade interna'} (${val.toFixed(2)} u · w=${pctW}%)`;
    return { value: val, start: s0, end: s1, kind, label };
  });
}

/**
 * Deterministically estimates the reachable Cartesian bounds [min, max] per axis.
 * When `referencePos` is supplied, adapts each active axis interval to the robot's
 * current pose, transverse coordinates, joint [min, max] limits, and self-collision rules.
 */
export function computeCartesianBounds(joints, referencePos = null, mode = 'position', environment = null) {
  if (!Array.isArray(joints) || joints.length === 0) {
    return { min: [0, 0, 0], max: [0, 0, 0], span: [0, 0, 0], singularities: [[], [], []] };
  }

  const { globalBounds, validSamples, use3D, globalMaxW } = getStructuralEnvelope(joints, environment);
  if (!Array.isArray(referencePos) || referencePos.length !== 3 || !referencePos.every(Number.isFinite)) {
    return { ...globalBounds, singularities: [[], [], []] };
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
  const singularities = [[], [], []];

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
    const axisSamples = [];

    const recordBound = (res) => {
      if (!res) return;
      if (Array.isArray(res.samples) && res.samples.length > 0) {
        axisSamples.push(...res.samples);
      }
      const val = res.bestCoord;
      if (val === null || !Number.isFinite(val)) return;
      if (val < lo) lo = val;
      if (val > hi) hi = val;
    };

    // 1. Directional kinematic ray-marching from current pose
    const negPrimary = marchAxisBound(currentQ, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D);
    const posPrimary = marchAxisBound(currentQ, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D);
    recordBound(negPrimary);
    recordBound(posPrimary);

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

    if (negSeed) {
      recordBound(marchAxisBound(negSeed, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
      if (negPrimary.bestCoord === null || negPrimary.bestCoord > lo + stepSize * 1.5) {
        recordBound(marchAxisBound(negSeed, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
      }
    }
    if (posSeed) {
      recordBound(marchAxisBound(posSeed, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
      if (posPrimary.bestCoord === null || posPrimary.bestCoord < hi - stepSize * 1.5) {
        recordBound(marchAxisBound(posSeed, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
      }
    }
    if (mirrorSeed && mirrorSeed[0] >= joints[0].min && mirrorSeed[0] <= joints[0].max) {
      if (referencePos[axis] >= 0 && lo >= referencePos[axis] - 1e-3) {
        recordBound(marchAxisBound(mirrorSeed, axis, -1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
      }
      if (referencePos[axis] <= 0 && hi <= referencePos[axis] + 1e-3) {
        recordBound(marchAxisBound(mirrorSeed, axis, 1, referencePos, otherAxes, joints, taskDims, refLen, stepSize, environment, use3D));
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
    } else {
      singularities[axis] = extractAxisSingularities({
        axis,
        samples: axisSamples,
        lo: min[axis],
        hi: max[axis],
        spanAxis: span[axis],
        globalMaxW,
        otherAxes,
        referencePos,
        validSamples,
        joints,
        use3D,
      });
    }
  }

  return { min, max, span, singularities };
}
