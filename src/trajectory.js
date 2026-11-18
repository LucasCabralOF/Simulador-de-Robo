import { DEG, forwardKinematics, sweepJointsCollisionFree } from './kinematics.js';

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

function nudgeJoint(joint, delta) {
  const candidate = joint.val + delta;
  if (candidate >= joint.min && candidate <= joint.max) return candidate;
  const opposite = joint.val - delta;
  if (opposite >= joint.min && opposite <= joint.max) return opposite;
  return clamp(candidate, joint.min, joint.max);
}

/**
 * Builds default Home / Ponto A / Ponto B waypoints compatible with any joint architecture
 * arranged as an asymmetric non-collinear triangle within joint limits.
 * @param {import('./types.js').Joint[]} joints
 * @returns {import('./types.js').Waypoint[]}
 */
export function defaultWaypointsForJoints(joints) {
  if (!Array.isArray(joints) || joints.length === 0) return [];
  const home = joints.map(j => clamp(j.val, j.min, j.max));
  const pointA = joints.map((j, i) => {
    if (i === 0) return nudgeJoint(j, j.type === 'R' ? 40 : 2.5);
    if (i === 1) return nudgeJoint(j, j.type === 'R' ? 20 : 1.5);
    if (i === 2) return nudgeJoint(j, j.type === 'R' ? -25 : -1.5);
    return clamp(j.val, j.min, j.max);
  });
  const pointB = joints.map((j, i) => {
    if (i === 0) return nudgeJoint(j, j.type === 'R' ? 15 : 1.0);
    if (i === 1) return nudgeJoint(j, j.type === 'R' ? -30 : -2.0);
    if (i === 2) return nudgeJoint(j, j.type === 'R' ? 35 : 2.0);
    return clamp(j.val, j.min, j.max);
  });
  return [
    { id: 1, name: 'Home', q: home, duration: 0 },
    { id: 2, name: 'Ponto A', q: pointA, duration: 2.0 },
    { id: 3, name: 'Ponto B', q: pointB, duration: 2.5 },
  ];
}

/**
 * Synchronizes existing waypoints to match the current joints length and [min, max] limits.
 * @param {import('./types.js').Waypoint[]} waypoints
 * @param {import('./types.js').Joint[]} joints
 * @returns {import('./types.js').Waypoint[]}
 */
export function syncWaypointsToJoints(waypoints, joints) {
  if (!Array.isArray(waypoints)) return [];
  return waypoints.map(wp => ({
    ...wp,
    q: joints.map((j, i) => {
      const existing = Array.isArray(wp.q) ? wp.q[i] : undefined;
      const raw = Number.isFinite(existing) ? existing : j.val;
      return clamp(raw, j.min, j.max);
    }),
  }));
}

/**
 * Quintic polynomial interpolation (zero velocity and zero acceleration at endpoints)
 * s in [0, 1]
 */
export function quinticInterp(q0, q1, s, T) {
  const s2 = s * s;
  const s3 = s2 * s;
  const s4 = s3 * s;
  const s5 = s4 * s;

  const delta = q1 - q0;
  // Position: q0 + delta * (10s^3 - 15s^4 + 6s^5)
  const pos = q0 + delta * (10 * s3 - 15 * s4 + 6 * s5);

  // Velocity: delta * (30s^2 - 60s^3 + 30s^4) / T
  const vel = T > 0 ? (delta * (30 * s2 - 60 * s3 + 30 * s4)) / T : 0;

  // Acceleration: delta * (60s - 180s^2 + 120s^3) / T^2
  const acc = T > 0 ? (delta * (60 * s - 180 * s2 + 120 * s3)) / (T * T) : 0;

  return { pos, vel, acc };
}

/**
 * Quintic Hermite polynomial interpolation with arbitrary endpoint velocities and accelerations
 * s in [0, 1]
 */
export function quinticHermiteInterp(q0, q1, v0, v1, a0, a1, s, T) {
  if (T <= 0) {
    return { pos: s >= 1 ? q1 : q0, vel: 0, acc: 0 };
  }
  const s2 = s * s;
  const s3 = s2 * s;
  const s4 = s3 * s;
  const s5 = s4 * s;
  const T2 = T * T;

  const delta = q1 - q0;
  const c0 = q0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T2;
  const c3 = 10 * delta - (6 * v0 + 4 * v1) * T - 0.5 * (3 * a0 - a1) * T2;
  const c4 = -15 * delta + (8 * v0 + 7 * v1) * T + 0.5 * (3 * a0 - 2 * a1) * T2;
  const c5 = 6 * delta - 3 * (v0 + v1) * T - 0.5 * (a0 - a1) * T2;

  const pos = c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5;
  const vel = (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T;
  const acc = (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / T2;

  return { pos, vel, acc };
}

/**
 * Cubic polynomial interpolation (zero velocity at endpoints)
 * s in [0, 1]
 */
export function cubicInterp(q0, q1, s, T) {
  const s2 = s * s;
  const s3 = s2 * s;

  const delta = q1 - q0;
  const pos = q0 + delta * (3 * s2 - 2 * s3);
  const vel = T > 0 ? (delta * (6 * s - 6 * s2)) / T : 0;
  const acc = T > 0 ? (delta * (6 - 12 * s)) / (T * T) : 0;

  return { pos, vel, acc };
}

/**
 * Trapezoidal velocity profile (LSPB — Linear Segment with Parabolic Blends)
 * Default blendRatio = 1/3 matches classic symmetric 1/3 accel - 1/3 cruise - 1/3 decel.
 * s in [0, 1]
 */
export function trapezoidalInterp(q0, q1, s, T, blendRatio = 1 / 3) {
  const clampedS = Math.max(0, Math.min(1, s));
  if (T <= 0) {
    return { pos: clampedS >= 1 ? q1 : q0, vel: 0, acc: 0 };
  }
  const r = Math.max(0.05, Math.min(0.5, blendRatio));
  const delta = q1 - q0;
  const tb = r * T;
  const vc = delta / (T * (1 - r));
  const ac = vc / tb;
  const t = clampedS * T;

  if (t <= tb) {
    return {
      pos: q0 + 0.5 * ac * t * t,
      vel: ac * t,
      acc: ac,
    };
  }
  if (t >= T - tb) {
    const tau = T - t;
    return {
      pos: q1 - 0.5 * ac * tau * tau,
      vel: ac * tau,
      acc: -ac,
    };
  }
  return {
    pos: q0 + 0.5 * ac * tb * tb + vc * (t - tb),
    vel: vc,
    acc: 0,
  };
}

/**
 * Computes segment start/end timestamps for a sequence of waypoints
 */
export function computeTrajectoryTimeline(waypoints) {
  if (!Array.isArray(waypoints) || waypoints.length < 2) {
    return { segments: [], totalDuration: 0 };
  }

  const segments = [];
  let elapsed = 0;

  for (let i = 0; i < waypoints.length - 1; i++) {
    const duration = Math.max(0.01, Number(waypoints[i + 1].duration) || 2.0);
    segments.push({
      fromIndex: i,
      toIndex: i + 1,
      startTime: elapsed,
      endTime: elapsed + duration,
      duration,
    });
    elapsed += duration;
  }

  return { segments, totalDuration: elapsed };
}

function computeViaPointVelocities(waypoints, segments) {
  const count = waypoints.length;
  const dof = waypoints.reduce((max, wp) => Math.max(max, Array.isArray(wp.q) ? wp.q.length : 0), 0);
  const velocities = Array.from({ length: count }, () => Array(dof).fill(0));

  for (let m = 1; m < count - 1; m++) {
    const TPrev = segments[m - 1].duration;
    const TNext = segments[m].duration;
    const qPrev = waypoints[m - 1].q || [];
    const qCurr = waypoints[m].q || [];
    const qNext = waypoints[m + 1].q || [];
    const denom = TPrev + TNext;
    if (denom <= 0) continue;

    for (let i = 0; i < dof; i++) {
      const prevVal = Number.isFinite(qPrev[i]) ? qPrev[i] : 0;
      const currVal = Number.isFinite(qCurr[i]) ? qCurr[i] : prevVal;
      const nextVal = Number.isFinite(qNext[i]) ? qNext[i] : currVal;
      const slopePrev = (currVal - prevVal) / TPrev;
      const slopeNext = (nextVal - currVal) / TNext;
      velocities[m][i] = (TNext * slopePrev + TPrev * slopeNext) / denom;
    }
  }

  return velocities;
}

function solve3x3(mat, rhs) {
  const m = [
    [mat[0][0], mat[0][1], mat[0][2], rhs[0]],
    [mat[1][0], mat[1][1], mat[1][2], rhs[1]],
    [mat[2][0], mat[2][1], mat[2][2], rhs[2]],
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
    if (Math.abs(diag) < 1e-14) return [0, 0, 0];
    for (let r = col + 1; r < 3; r++) {
      const factor = m[r][col] / diag;
      for (let c = col; c < 4; c++) m[r][c] -= factor * m[col][c];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let sum = m[r][3];
    for (let c = r + 1; c < 3; c++) sum -= m[r][c] * x[c];
    x[r] = sum / m[r][r];
  }
  return x;
}

function resolveJointVal(qVec, k, joint, firstWaypoint) {
  if (Array.isArray(qVec) && Number.isFinite(qVec[k])) return qVec[k];
  if (Array.isArray(firstWaypoint?.q) && Number.isFinite(firstWaypoint.q[k])) return firstWaypoint.q[k];
  const fallback = joint.type === 'P' ? joint.d : joint.theta;
  return Number.isFinite(fallback) ? fallback : 0;
}

const cartesianCache = new Map();

export function solveCartesianSegmentSamples(waypoints, jointsTemplate, stepsPerSegment = 60, environment = null) {
  const { segments } = computeTrajectoryTimeline(waypoints);
  const firstWaypoint = waypoints[0];
  const n = jointsTemplate.length;
  const refLen = Math.max(
    1,
    jointsTemplate.reduce((acc, j) => acc + Math.hypot(j.a, j.d), 0) / Math.max(1, n)
  );
  const lambda = 0.02 * refLen;
  const lambda2 = lambda * lambda;
  const alpha = 0.25;

  return segments.map(seg => {
    const q0 = jointsTemplate.map((j, k) => clamp(resolveJointVal(waypoints[seg.fromIndex].q, k, j, firstWaypoint), j.min, j.max));
    const q1 = jointsTemplate.map((j, k) => clamp(resolveJointVal(waypoints[seg.toIndex].q, k, j, firstWaypoint), j.min, j.max));

    const p0 = forwardKinematics(jointsTemplate.map((j, k) => ({ ...j, val: q0[k] }))).positions.at(-1);
    const p1 = forwardKinematics(jointsTemplate.map((j, k) => ({ ...j, val: q1[k] }))).positions.at(-1);

    const qSamples = Array.from({ length: stepsPerSegment + 1 });
    qSamples[0] = q0;
    qSamples[stepsPerSegment] = q1;

    let qCurr = [...q0];
    for (let step = 1; step < stepsPerSegment; step++) {
      const s = step / stepsPerSegment;
      const s2 = s * s;
      const s3 = s2 * s;
      const sigma = 10 * s3 - 15 * s2 * s2 + 6 * s3 * s2;

      const pDes = [
        p0[0] + sigma * (p1[0] - p0[0]),
        p0[1] + sigma * (p1[1] - p0[1]),
        p0[2] + sigma * (p1[2] - p0[2]),
      ];
      const qGuide = q0.map((v0, j) => v0 + sigma * (q1[j] - v0));

      for (let iter = 0; iter < 24; iter++) {
        const fk = forwardKinematics(jointsTemplate.map((j, k) => ({ ...j, val: qCurr[k] })));
        const pCurr = fk.positions.at(-1);
        const ep = [pDes[0] - pCurr[0], pDes[1] - pCurr[1], pDes[2] - pCurr[2]];

        const eq = qGuide.map((vg, j) => (vg - qCurr[j]) * (jointsTemplate[j].type === 'R' ? DEG : 1));
        const Jp = fk.jacobian;

        const JpEq = [0, 0, 0];
        for (let r = 0; r < 3; r++) {
          let sum = 0;
          for (let j = 0; j < n; j++) sum += Jp[r][j] * eq[j];
          JpEq[r] = sum;
        }

        const rhs = [
          ep[0] - alpha * JpEq[0],
          ep[1] - alpha * JpEq[1],
          ep[2] - alpha * JpEq[2],
        ];

        const A = [
          [lambda2, 0, 0],
          [0, lambda2, 0],
          [0, 0, lambda2],
        ];
        for (let r = 0; r < 3; r++) {
          for (let c = r; c < 3; c++) {
            let dot = 0;
            for (let j = 0; j < n; j++) dot += Jp[r][j] * Jp[c][j];
            A[r][c] += dot;
            if (c !== r) A[c][r] += dot;
          }
        }

        const w = solve3x3(A, rhs);
        const dq = Array(n);
        let maxRatio = 0;
        for (let j = 0; j < n; j++) {
          const val = alpha * eq[j] + Jp[0][j] * w[0] + Jp[1][j] * w[1] + Jp[2][j] * w[2];
          dq[j] = val;
          const limit = jointsTemplate[j].type === 'R' ? 0.35 : refLen * 0.35;
          maxRatio = Math.max(maxRatio, Math.abs(val) / limit);
        }

        const stepScale = maxRatio > 1 ? 1 / maxRatio : 1;
        const prevJoints = jointsTemplate.map((joint, j) => ({ ...joint, val: qCurr[j] }));
        const candidateJoints = jointsTemplate.map((joint, j) => {
          const deltaDegOrU = (dq[j] * stepScale) / (joint.type === 'R' ? DEG : 1);
          return { ...joint, val: clamp(qCurr[j] + deltaDegOrU, joint.min, joint.max) };
        });
        const swept = sweepJointsCollisionFree(prevJoints, candidateJoints, 'gripper', environment);
        const safeJoints = swept.blocked ? swept.joints : candidateJoints;
        let maxDelta = 0;
        for (let j = 0; j < n; j++) {
          maxDelta = Math.max(maxDelta, Math.abs(safeJoints[j].val - qCurr[j]));
          qCurr[j] = safeJoints[j].val;
        }

        const errNorm = Math.hypot(ep[0], ep[1], ep[2]);
        if (errNorm < 1e-5 && maxDelta < 1e-4) break;
      }

      qSamples[step] = [...qCurr];
    }

    const dt = seg.duration / stepsPerSegment;
    const velocities = qSamples.map((_, k) => {
      if (k === 0 || k === stepsPerSegment || dt <= 0) return Array(n).fill(0);
      return qSamples[k + 1].map((vNext, j) => (vNext - qSamples[k - 1][j]) / (2 * dt));
    });
    const accelerations = qSamples.map((_, k) => {
      if (k === 0 || k === stepsPerSegment || dt <= 0) return Array(n).fill(0);
      return qSamples[k + 1].map((vNext, j) => (vNext - 2 * qSamples[k][j] + qSamples[k - 1][j]) / (dt * dt));
    });

    return { qSamples, velocities, accelerations, stepsPerSegment };
  });
}

function getCachedCartesianSegments(waypoints, jointsTemplate, stepsPerSegment = 60, environment = null) {
  const key = JSON.stringify({
    w: waypoints.map(wp => [wp.q, wp.duration]),
    j: jointsTemplate.map(j => [j.type, j.a, j.alpha, j.d, j.theta, j.min, j.max]),
    n: stepsPerSegment,
    e: environment
      ? [Boolean(environment.floorEnabled), environment.floorZ, Boolean(environment.obstacle?.enabled), environment.obstacle?.shape, environment.obstacle?.position, environment.obstacle?.radius]
      : null,
  });
  let cached = cartesianCache.get(key);
  if (!cached) {
    cached = solveCartesianSegmentSamples(waypoints, jointsTemplate, stepsPerSegment, environment);
    if (cartesianCache.size >= 12) {
      cartesianCache.delete(cartesianCache.keys().next().value);
    }
    cartesianCache.set(key, cached);
  }
  return cached;
}

/**
 * Evaluates the full robot joint state at a specific time t.
 * @param {import('./types.js').Waypoint[]} waypoints
 * @param {number} time
 * @param {string} [type]
 * @param {import('./types.js').Joint[] | null} [jointsTemplate]
 * @param {import('./types.js').EnvironmentConfig | null} [environment]
 * @returns {{ q: number[], velocities: number[], segmentIndex: number, progress: number, blocked?: boolean } | null}
 */
export function evaluateTrajectory(waypoints, time, type = 'quintic', jointsTemplate = null, environment = null) {
  if (!Array.isArray(waypoints) || waypoints.length === 0) {
    return null;
  }
  if (waypoints.length === 1) {
    return {
      q: [...waypoints[0].q],
      velocities: waypoints[0].q.map(() => 0),
      accelerations: waypoints[0].q.map(() => 0),
      segmentIndex: 0,
      progress: 1,
    };
  }

  const { segments, totalDuration } = computeTrajectoryTimeline(waypoints);
  if (totalDuration <= 0) return null;

  const clampedTime = Math.max(0, Math.min(totalDuration, time));

  // Find active segment
  let segIndex = segments.findIndex(s => clampedTime >= s.startTime && clampedTime <= s.endTime);
  if (segIndex === -1) {
    segIndex = segments.length - 1;
  }
  const seg = segments[segIndex];

  const s = clampedTime >= seg.endTime
    ? 1
    : clampedTime <= seg.startTime
      ? 0
      : (clampedTime - seg.startTime) / seg.duration;

  if (type === 'cartesian' && Array.isArray(jointsTemplate) && jointsTemplate.length > 0) {
    const stepsPerSegment = 60;
    const table = getCachedCartesianSegments(waypoints, jointsTemplate, stepsPerSegment, environment)[segIndex];
    const f = s * table.stepsPerSegment;
    const idx0 = Math.min(table.stepsPerSegment, Math.floor(f));
    const idx1 = Math.min(table.stepsPerSegment, idx0 + 1);
    const frac = f - idx0;

    const q = table.qSamples[idx0].map((v0, i) => v0 + frac * (table.qSamples[idx1][i] - v0));
    const velocities = table.velocities[idx0].map((v0, i) => v0 + frac * (table.velocities[idx1][i] - v0));
    const accelerations = table.accelerations[idx0].map((v0, i) => v0 + frac * (table.accelerations[idx1][i] - v0));

    return {
      q,
      velocities,
      accelerations,
      segmentIndex: seg.fromIndex,
      progress: clampedTime / totalDuration,
      totalDuration,
    };
  }

  const q0 = Array.isArray(waypoints[seg.fromIndex].q) ? waypoints[seg.fromIndex].q : [];
  const q1 = Array.isArray(waypoints[seg.toIndex].q) ? waypoints[seg.toIndex].q : [];

  const dof = Math.max(q0.length, q1.length);
  const q = [];
  const velocities = [];
  const accelerations = [];

  if (type === 'continuous') {
    const viaVelocities = computeViaPointVelocities(waypoints, segments);
    const vStart = viaVelocities[seg.fromIndex];
    const vEnd = viaVelocities[seg.toIndex];
    for (let i = 0; i < dof; i++) {
      const startVal = Number.isFinite(q0[i]) ? q0[i] : (Number.isFinite(q1[i]) ? q1[i] : 0);
      const endVal = Number.isFinite(q1[i]) ? q1[i] : startVal;
      const res = quinticHermiteInterp(startVal, endVal, vStart[i] || 0, vEnd[i] || 0, 0, 0, s, seg.duration);
      q.push(res.pos);
      velocities.push(res.vel);
      accelerations.push(res.acc);
    }
  } else {
    const interpFn = type === 'cubic'
      ? cubicInterp
      : type === 'trapezoidal'
        ? trapezoidalInterp
        : quinticInterp;
    for (let i = 0; i < dof; i++) {
      const startVal = Number.isFinite(q0[i]) ? q0[i] : (Number.isFinite(q1[i]) ? q1[i] : 0);
      const endVal = Number.isFinite(q1[i]) ? q1[i] : startVal;
      const res = interpFn(startVal, endVal, s, seg.duration);
      q.push(res.pos);
      velocities.push(res.vel);
      accelerations.push(res.acc);
    }
  }

  return {
    q,
    velocities,
    accelerations,
    segmentIndex: seg.fromIndex,
    progress: clampedTime / totalDuration,
    totalDuration,
  };
}

/**
 * Samples the 3D trajectory path curve and kinematic profiles for visualization in Three.js and SVG charts.
 * @param {import('./types.js').Waypoint[]} waypoints
 * @param {import('./types.js').Joint[]} jointsTemplate
 * @param {number} [numSamples]
 * @param {string} [interpType]
 * @param {import('./types.js').EnvironmentConfig | null} [environment]
 * @returns {import('./types.js').TrajectoryData}
 */
export function sampleTrajectoryPath(waypoints, jointsTemplate, numSamples, interpType = 'quintic', environment = null) {
  if (!Array.isArray(waypoints) || waypoints.length < 2) {
    return {
      pathPoints: new Float32Array(0),
      waypointCoords: [],
      totalDuration: 0,
      times: [],
      waypointTimes: [],
      jointProfiles: [],
      tcpSpeeds: [],
    };
  }

  const { segments, totalDuration } = computeTrajectoryTimeline(waypoints);
  if (totalDuration <= 0) {
    return {
      pathPoints: new Float32Array(0),
      waypointCoords: [],
      totalDuration: 0,
      times: [],
      waypointTimes: [],
      jointProfiles: [],
      tcpSpeeds: [],
    };
  }

  const firstWaypoint = waypoints[0];
  let times;

  if (typeof numSamples === 'number') {
    const count = Math.max(10, Math.min(500, numSamples));
    times = Array.from({ length: count }, (_, i) => (i / (count - 1)) * totalDuration);
  } else {
    const stepsPerSegment = 60;
    times = [];
    segments.forEach((seg, segIdx) => {
      const startStep = segIdx === 0 ? 0 : 1;
      for (let k = startStep; k <= stepsPerSegment; k++) {
        const t = k === 0
          ? seg.startTime
          : k === stepsPerSegment
            ? seg.endTime
            : seg.startTime + (k / stepsPerSegment) * seg.duration;
        times.push(t);
      }
    });
  }

  const pathPoints = new Float32Array(times.length * 3);
  const jointProfiles = jointsTemplate.map((j, idx) => ({
    index: idx,
    label: `J${idx + 1}`,
    type: j.type,
    q: [],
    v: [],
    a: [],
  }));
  const tcpSpeeds = [];

  for (let i = 0; i < times.length; i++) {
    const evalRes = evaluateTrajectory(waypoints, times[i], interpType, jointsTemplate, environment);
    const sampledJoints = jointsTemplate.map((j, k) => {
      const posVal = resolveJointVal(evalRes.q, k, j, firstWaypoint);
      const velVal = Number.isFinite(evalRes.velocities?.[k]) ? evalRes.velocities[k] : 0;
      const accVal = Number.isFinite(evalRes.accelerations?.[k]) ? evalRes.accelerations[k] : 0;
      jointProfiles[k].q.push(posVal);
      jointProfiles[k].v.push(velVal);
      jointProfiles[k].a.push(accVal);
      return { ...j, val: posVal };
    });
    const fk = forwardKinematics(sampledJoints, { includeOrientation: false });
    const end = fk.positions.at(-1);

    pathPoints[i * 3] = end[0];
    pathPoints[i * 3 + 1] = end[1];
    pathPoints[i * 3 + 2] = end[2];

    let vx = 0;
    let vy = 0;
    let vz = 0;
    for (let k = 0; k < jointsTemplate.length; k++) {
      const qDot = jointsTemplate[k].type === 'R' ? jointProfiles[k].v[i] * DEG : jointProfiles[k].v[i];
      vx += fk.jacobian[0][k] * qDot;
      vy += fk.jacobian[1][k] * qDot;
      vz += fk.jacobian[2][k] * qDot;
    }
    tcpSpeeds.push(Math.hypot(vx, vy, vz));
  }

  const waypointTimes = [0, ...segments.map(s => s.endTime)];

  // Calculate coordinates for each discrete waypoint
  const waypointCoords = waypoints.map(wp => {
    const sampledJoints = jointsTemplate.map((j, k) => ({
      ...j,
      val: resolveJointVal(wp.q, k, j, firstWaypoint),
    }));
    const fk = forwardKinematics(sampledJoints, { includeOrientation: false });
    return fk.positions.at(-1);
  });

  return {
    pathPoints,
    waypointCoords,
    totalDuration,
    times,
    waypointTimes,
    jointProfiles,
    tcpSpeeds,
  };
}
