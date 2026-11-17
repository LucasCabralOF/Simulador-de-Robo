import test from 'node:test';
import assert from 'node:assert/strict';
import {
  quinticInterp,
  quinticHermiteInterp,
  cubicInterp,
  computeTrajectoryTimeline,
  evaluateTrajectory,
  sampleTrajectoryPath,
  defaultWaypointsForJoints,
  syncWaypointsToJoints,
} from '../src/trajectory.js';
import { DEFAULT_JOINTS } from '../src/kinematics.js';
import { PRESETS, PRESET_LIST } from '../src/presets.js';

test('quintic interpolation satisfies boundary conditions of zero velocity and acceleration', () => {
  const q0 = 10;
  const q1 = 50;
  const T = 2.0;

  const start = quinticInterp(q0, q1, 0, T);
  assert.equal(start.pos, q0);
  assert.equal(start.vel, 0);
  assert.equal(start.acc, 0);

  const end = quinticInterp(q0, q1, 1, T);
  assert.equal(end.pos, q1);
  assert.equal(end.vel, 0);
  assert.equal(end.acc, 0);

  const mid = quinticInterp(q0, q1, 0.5, T);
  assert.equal(mid.pos, (q0 + q1) / 2);
  assert.ok(mid.vel > 0, 'Velocity at midpoint should be positive for increasing q');
  assert.equal(mid.acc, 0, 'Acceleration at midpoint of symmetric quintic spline should be zero');
});

test('cubic interpolation satisfies boundary conditions of zero velocity', () => {
  const q0 = -20;
  const q1 = 30;
  const T = 1.5;

  const start = cubicInterp(q0, q1, 0, T);
  assert.equal(start.pos, q0);
  assert.equal(start.vel, 0);

  const end = cubicInterp(q0, q1, 1, T);
  assert.equal(end.pos, q1);
  assert.equal(end.vel, 0);
});

test('trajectory timeline calculates multi-segment durations correctly', () => {
  const waypoints = [
    { id: 1, name: 'P0', q: [0, 0, 0], duration: 0 },
    { id: 2, name: 'P1', q: [45, 5, 30], duration: 2.5 },
    { id: 3, name: 'P2', q: [-30, 2, 60], duration: 1.5 },
  ];

  const { segments, totalDuration } = computeTrajectoryTimeline(waypoints);
  assert.equal(segments.length, 2);
  assert.equal(totalDuration, 4.0);
  assert.equal(segments[0].startTime, 0);
  assert.equal(segments[0].endTime, 2.5);
  assert.equal(segments[1].startTime, 2.5);
  assert.equal(segments[1].endTime, 4.0);
});

test('trajectory evaluation interpolates continuous joint positions and velocities', () => {
  const waypoints = [
    { id: 1, name: 'P0', q: [10, 20], duration: 0 },
    { id: 2, name: 'P1', q: [50, 40], duration: 2.0 },
  ];

  const at0 = evaluateTrajectory(waypoints, 0);
  assert.deepEqual(at0.q, [10, 20]);
  assert.deepEqual(at0.velocities, [0, 0]);

  const at1 = evaluateTrajectory(waypoints, 1.0);
  assert.deepEqual(at1.q, [30, 30]);
  assert.ok(at1.velocities[0] > 0);
  assert.ok(at1.velocities[1] > 0);

  const at2 = evaluateTrajectory(waypoints, 2.0);
  assert.deepEqual(at2.q, [50, 40]);
  assert.deepEqual(at2.velocities, [0, 0]);
});

test('3D trajectory path sampling generates continuous Cartesian points and waypoint coordinates', () => {
  const waypoints = [
    { id: 1, name: 'A', q: [0, 5, 0], duration: 0 },
    { id: 2, name: 'B', q: [45, 8, 30], duration: 2.0 },
    { id: 3, name: 'C', q: [-45, 3, -30], duration: 2.0 },
  ];

  const { pathPoints, waypointCoords, totalDuration } = sampleTrajectoryPath(waypoints, DEFAULT_JOINTS, 50);
  assert.equal(totalDuration, 4.0);
  assert.equal(pathPoints.length, 150); // 50 * 3
  assert.equal(waypointCoords.length, 3);
  assert.ok(waypointCoords.every(p => p.length === 3 && p.every(Number.isFinite)));
});

test('per-segment path sampling passes through every intermediate waypoint vertex', () => {
  const waypoints = [
    { id: 1, name: 'P0', q: [0, 4, 15], duration: 0 },
    { id: 2, name: 'P1', q: [60, 7, -30], duration: 1.3 },
    { id: 3, name: 'P2', q: [-45, 2, 80], duration: 2.7 },
    { id: 4, name: 'P3', q: [90, 9, -60], duration: 0.9 },
  ];

  const { pathPoints, waypointCoords, totalDuration } = sampleTrajectoryPath(waypoints, DEFAULT_JOINTS);
  assert.ok(Math.abs(totalDuration - 4.9) < 1e-12);
  assert.equal(waypointCoords.length, 4);
  assert.equal(pathPoints.length, (1 + 3 * 60) * 3);

  waypointCoords.forEach((wp, wpIdx) => {
    const vertexIdx = wpIdx * 60;
    assert.equal(pathPoints[vertexIdx * 3], Math.fround(wp[0]));
    assert.equal(pathPoints[vertexIdx * 3 + 1], Math.fround(wp[1]));
    assert.equal(pathPoints[vertexIdx * 3 + 2], Math.fround(wp[2]));
  });
});

test('sampleTrajectoryPath respects interpType (cubic vs quintic)', () => {
  const waypoints = [
    { id: 1, name: 'Start', q: [0, 2, 0], duration: 0 },
    { id: 2, name: 'End', q: [90, 8, 60], duration: 2.0 },
  ];

  const quinticRes = sampleTrajectoryPath(waypoints, DEFAULT_JOINTS, undefined, 'quintic');
  const cubicRes = sampleTrajectoryPath(waypoints, DEFAULT_JOINTS, undefined, 'cubic');

  // Quarter segment (index 15 of 60): quintic and cubic polynomials differ substantially
  const idx = 15 * 3;
  const diff = Math.hypot(
    quinticRes.pathPoints[idx] - cubicRes.pathPoints[idx],
    quinticRes.pathPoints[idx + 1] - cubicRes.pathPoints[idx + 1],
    quinticRes.pathPoints[idx + 2] - cubicRes.pathPoints[idx + 2]
  );
  assert.ok(diff > 0.1, `Expected cubic and quintic paths to differ at s=0.25, got diff=${diff}`);
});

test('sampleTrajectoryPath is independent of mutable current joint.val', () => {
  const fullWaypoints = [
    { id: 1, name: 'A', q: [10, 4, 20], duration: 0 },
    { id: 2, name: 'B', q: [70, 8, -40], duration: 2.0 },
  ];
  const partialWaypoints = [
    { id: 1, name: 'A', q: [10, 4], duration: 0 },
    { id: 2, name: 'B', q: [70, 8], duration: 2.0 },
  ];

  const jointsA = DEFAULT_JOINTS.map(j => ({ ...j, val: 0 }));
  const jointsB = DEFAULT_JOINTS.map(j => ({ ...j, val: j.type === 'R' ? 123 : 9.5 }));

  const resFullA = sampleTrajectoryPath(fullWaypoints, jointsA);
  const resFullB = sampleTrajectoryPath(fullWaypoints, jointsB);
  assert.deepEqual(resFullA.pathPoints, resFullB.pathPoints);
  assert.deepEqual(resFullA.waypointCoords, resFullB.waypointCoords);

  const resPartialA = sampleTrajectoryPath(partialWaypoints, jointsA);
  const resPartialB = sampleTrajectoryPath(partialWaypoints, jointsB);
  assert.deepEqual(resPartialA.pathPoints, resPartialB.pathPoints);
  assert.deepEqual(resPartialA.waypointCoords, resPartialB.waypointCoords);
});

test('defaultWaypointsForJoints and syncWaypointsToJoints maintain joint count and limits', () => {
  for (const preset of PRESET_LIST) {
    const wps = defaultWaypointsForJoints(preset.joints);
    assert.equal(wps.length, 3);
    for (const wp of wps) {
      assert.equal(wp.q.length, preset.joints.length);
      wp.q.forEach((val, idx) => {
        const j = preset.joints[idx];
        assert.ok(val >= j.min && val <= j.max, `${preset.id} joint ${idx} value ${val} out of [${j.min}, ${j.max}]`);
      });
    }
  }

  const baseWps = [
    { id: 1, name: 'W1', q: [175, 12, -200], duration: 0 },
    { id: 2, name: 'W2', q: [45, 5], duration: 2.0 },
  ];
  const expandedJoints = [
    ...DEFAULT_JOINTS,
    { id: 4, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 22, min: -90, max: 90, speed: 0 },
  ];
  const syncedExpanded = syncWaypointsToJoints(baseWps, expandedJoints);
  assert.deepEqual(syncedExpanded[0].q, [175, 10, -180, 22]);
  assert.deepEqual(syncedExpanded[1].q, [45, 5, 30, 22]);

  const shrunkJoints = DEFAULT_JOINTS.slice(0, 2);
  const syncedShrunk = syncWaypointsToJoints(baseWps, shrunkJoints);
  assert.deepEqual(syncedShrunk[0].q, [175, 10]);
  assert.deepEqual(syncedShrunk[1].q, [45, 5]);
});

test('quinticHermiteInterp and continuous mode maintain non-zero C2 velocity at via-points', () => {
  const hermiteStart = quinticHermiteInterp(10, 50, 12, -8, 0, 0, 0, 2.0);
  assert.equal(hermiteStart.pos, 10);
  assert.ok(Math.abs(hermiteStart.vel - 12) < 1e-12);
  assert.ok(Math.abs(hermiteStart.acc) < 1e-12);

  const hermiteEnd = quinticHermiteInterp(10, 50, 12, -8, 0, 0, 1, 2.0);
  assert.equal(hermiteEnd.pos, 50);
  assert.ok(Math.abs(hermiteEnd.vel - (-8)) < 1e-12);
  assert.ok(Math.abs(hermiteEnd.acc) < 1e-12);

  const waypoints = [
    { id: 1, name: 'P0', q: [0, 2, 0], duration: 0 },
    { id: 2, name: 'P1', q: [40, 5, 30], duration: 2.0 },
    { id: 3, name: 'P2', q: [100, 8, 75], duration: 1.5 },
  ];

  const atStart = evaluateTrajectory(waypoints, 0, 'continuous');
  assert.deepEqual(atStart.velocities, [0, 0, 0]);
  assert.deepEqual(atStart.accelerations, [0, 0, 0]);

  const atEnd = evaluateTrajectory(waypoints, 3.5, 'continuous');
  assert.ok(atEnd.velocities.every(v => Math.abs(v) < 1e-12));
  assert.ok(atEnd.accelerations.every(a => Math.abs(a) < 1e-12));

  // Just before and just after intermediate waypoint (t = 2.0)
  const beforeVia = evaluateTrajectory(waypoints, 2.0 - 1e-6, 'continuous');
  const afterVia = evaluateTrajectory(waypoints, 2.0 + 1e-6, 'continuous');
  for (let i = 0; i < 3; i++) {
    assert.ok(beforeVia.velocities[i] > 1, `Expected positive via velocity on joint ${i}, got ${beforeVia.velocities[i]}`);
    assert.ok(Math.abs(beforeVia.velocities[i] - afterVia.velocities[i]) < 1e-3);
  }
});

function pointToSegmentDist(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const abLen2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
  if (abLen2 < 1e-12) return Math.hypot(...ap);
  const t = Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / abLen2));
  const proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
  return Math.hypot(p[0] - proj[0], p[1] - proj[1], p[2] - proj[2]);
}

test('cartesian LIN mode produces straight 3D lines and respects joint limits', () => {
  const ur5 = PRESETS.ur5.joints;
  const wps = defaultWaypointsForJoints(ur5);

  const ptp = sampleTrajectoryPath(wps, ur5, undefined, 'quintic');
  const lin = sampleTrajectoryPath(wps, ur5, undefined, 'cartesian');

  const maxLineDev = (pathPoints, a, b, startVertex, endVertex) => {
    let maxD = 0;
    for (let v = startVertex; v <= endVertex; v++) {
      const pt = [pathPoints[v * 3], pathPoints[v * 3 + 1], pathPoints[v * 3 + 2]];
      maxD = Math.max(maxD, pointToSegmentDist(pt, a, b));
    }
    return maxD;
  };

  const ptpSeg0Dev = maxLineDev(ptp.pathPoints, ptp.waypointCoords[0], ptp.waypointCoords[1], 0, 60);
  const linSeg0Dev = maxLineDev(lin.pathPoints, lin.waypointCoords[0], lin.waypointCoords[1], 0, 60);
  const linSeg1Dev = maxLineDev(lin.pathPoints, lin.waypointCoords[1], lin.waypointCoords[2], 60, 120);

  assert.ok(linSeg0Dev < 0.02, `Cartesian segment 0 deviated by ${linSeg0Dev}`);
  assert.ok(linSeg1Dev < 0.02, `Cartesian segment 1 deviated by ${linSeg1Dev}`);
  assert.ok(linSeg0Dev < ptpSeg0Dev * 0.1, `Expected LIN (${linSeg0Dev}) to be much straighter than PTP (${ptpSeg0Dev})`);

  // Verify exact endpoints and joint limit compliance during evaluation
  for (const t of [0, 1.0, 2.0, 3.25, lin.totalDuration]) {
    const state = evaluateTrajectory(wps, t, 'cartesian', ur5);
    state.q.forEach((val, idx) => {
      assert.ok(val >= ur5[idx].min - 1e-9 && val <= ur5[idx].max + 1e-9);
    });
  }
  const atStart = evaluateTrajectory(wps, 0, 'cartesian', ur5);
  const atEnd = evaluateTrajectory(wps, lin.totalDuration, 'cartesian', ur5);
  assert.deepEqual(atStart.q, wps[0].q);
  assert.deepEqual(atEnd.q, wps[2].q);
  assert.ok(atStart.velocities.every(v => v === 0));
  assert.ok(atEnd.velocities.every(v => v === 0));
});

test('sampleTrajectoryPath exports synchronized time, q, v, a, and TCP linear speed profiles', () => {
  const wps = defaultWaypointsForJoints(DEFAULT_JOINTS);
  const quintic = sampleTrajectoryPath(wps, DEFAULT_JOINTS, undefined, 'quintic');
  const continuous = sampleTrajectoryPath(wps, DEFAULT_JOINTS, undefined, 'continuous');
  const cartesian = sampleTrajectoryPath(wps, DEFAULT_JOINTS, undefined, 'cartesian');

  assert.equal(quintic.times.length, 121);
  assert.deepEqual(quintic.waypointTimes, [0, 2.0, 4.5]);
  assert.equal(quintic.jointProfiles.length, DEFAULT_JOINTS.length);
  assert.equal(quintic.tcpSpeeds.length, 121);

  for (const prof of quintic.jointProfiles) {
    assert.equal(prof.q.length, 121);
    assert.equal(prof.v.length, 121);
    assert.equal(prof.a.length, 121);
    // Quintic PTP stops cleanly at t = 0, waypoint t = 2.0 (index 60), and t = 4.5 (index 120)
    assert.ok(Math.abs(prof.v[0]) < 1e-9);
    assert.ok(Math.abs(prof.v[60]) < 1e-9);
    assert.ok(Math.abs(prof.v[120]) < 1e-9);
  }
  assert.equal(quintic.tcpSpeeds[0], 0);
  assert.ok(quintic.tcpSpeeds[30] > 0.1, 'Mid-segment TCP speed should be strictly positive');
  assert.ok(quintic.tcpSpeeds[60] < 1e-9, 'Quintic TCP speed pauses at intermediate waypoint');

  // Continuous spline passes through intermediate waypoint at index 60 with non-zero TCP speed
  assert.ok(continuous.tcpSpeeds[60] > 0.1, 'Continuous spline maintains non-zero TCP speed at intermediate waypoint');

  // Cartesian LIN profile computes finite positive TCP speeds inside segments
  assert.ok(cartesian.tcpSpeeds.every(Number.isFinite));
  assert.ok(Math.max(...cartesian.tcpSpeeds) > 0.1);
});
