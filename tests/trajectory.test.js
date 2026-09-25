import test from 'node:test';
import assert from 'node:assert/strict';
import { quinticInterp, cubicInterp, computeTrajectoryTimeline, evaluateTrajectory, sampleTrajectoryPath } from '../src/trajectory.js';
import { DEFAULT_JOINTS } from '../src/kinematics.js';

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
