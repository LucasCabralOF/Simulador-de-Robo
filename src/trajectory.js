import { forwardKinematics } from './kinematics.js';

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

/**
 * Evaluates the full robot joint state at a specific time t
 */
export function evaluateTrajectory(waypoints, time, type = 'quintic') {
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
  let seg = segments.find(s => clampedTime >= s.startTime && clampedTime <= s.endTime);
  if (!seg) {
    seg = segments[segments.length - 1];
  }

  const s = Math.max(0, Math.min(1, (clampedTime - seg.startTime) / seg.duration));
  const q0 = waypoints[seg.fromIndex].q;
  const q1 = waypoints[seg.toIndex].q;

  const interpFn = type === 'cubic' ? cubicInterp : quinticInterp;

  const q = [];
  const velocities = [];
  const accelerations = [];

  for (let i = 0; i < q0.length; i++) {
    const res = interpFn(q0[i], q1[i], s, seg.duration);
    q.push(res.pos);
    velocities.push(res.vel);
    accelerations.push(res.acc);
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
 * Samples the 3D trajectory path curve for visualization in Three.js
 */
export function sampleTrajectoryPath(waypoints, jointsTemplate, numSamples = 80) {
  if (!Array.isArray(waypoints) || waypoints.length < 2) {
    return { pathPoints: new Float32Array(0), waypointCoords: [] };
  }

  const { totalDuration } = computeTrajectoryTimeline(waypoints);
  if (totalDuration <= 0) {
    return { pathPoints: new Float32Array(0), waypointCoords: [] };
  }

  const count = Math.max(10, Math.min(500, numSamples));
  const pathPoints = new Float32Array(count * 3);

  for (let i = 0; i < count; i++) {
    const t = (i / (count - 1)) * totalDuration;
    const evalRes = evaluateTrajectory(waypoints, t, 'quintic');
    const sampledJoints = jointsTemplate.map((j, k) => ({
      ...j,
      val: evalRes.q[k] !== undefined ? evalRes.q[k] : j.val,
    }));
    const fk = forwardKinematics(sampledJoints);
    const end = fk.positions.at(-1);

    pathPoints[i * 3] = end[0];
    pathPoints[i * 3 + 1] = end[1];
    pathPoints[i * 3 + 2] = end[2];
  }

  // Calculate coordinates for each discrete waypoint
  const waypointCoords = waypoints.map(wp => {
    const sampledJoints = jointsTemplate.map((j, k) => ({
      ...j,
      val: wp.q[k] !== undefined ? wp.q[k] : j.val,
    }));
    const fk = forwardKinematics(sampledJoints);
    return fk.positions.at(-1);
  });

  return { pathPoints, waypointCoords, totalDuration };
}
