import test from 'node:test';
import assert from 'node:assert/strict';
import { Matrix4 } from 'three';
import { DEG, DEFAULT_JOINTS, dhMatrix, forwardKinematics, integrateJoints, inverseKinematics, inverseVelocity, jacobianDiagnostics, matrixRows, multiplyVector, normalizeJoints, orientation, rotationFromRPY, validateJoints } from '../src/kinematics.js';

const near = (actual, expected, tolerance = 1e-8) => assert.ok(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);
const nearVector = (a, b, tolerance) => a.forEach((v, i) => near(v, b[i], tolerance));
const joint = (overrides = {}) => ({ id: 1, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 0, min: -180, max: 180, speed: 0, ...overrides });
const planar = [joint({ a: 3 }), joint({ id: 2, a: 2 })];

test('standard DH agrees with independent elementary transformations', () => {
  for (const j of [joint({ a: -2, alpha: -75, val: 34, d: 3 }), joint({ type: 'P', a: 2, theta: 66, alpha: 45, val: -1 })]) {
    const expected = new Matrix4().makeRotationZ((j.type === 'R' ? j.val : j.theta) * DEG)
      .multiply(new Matrix4().makeTranslation(0, 0, j.type === 'P' ? j.val : j.d))
      .multiply(new Matrix4().makeTranslation(j.a, 0, 0)).multiply(new Matrix4().makeRotationX(j.alpha * DEG));
    nearVector(dhMatrix(j).elements, expected.elements);
  }
});

test('RPR default agrees with analytical planar position and orientation', () => {
  const fk = forwardKinematics(DEFAULT_JOINTS);
  const t1 = 45 * DEG, t13 = 75 * DEG;
  nearVector(fk.positions.at(-1), [5 * Math.cos(t13) + 5 * Math.sin(t1), 5 * Math.sin(t13) - 5 * Math.cos(t1), 0]);
  nearVector(fk.orientation.rpy, [0, 0, 75]);
  assert.equal(fk.links.length, 3);
  near(fk.orientation.determinant, 1);
  near(fk.orientation.orthogonality, 0);
});

test('geometric Jacobian matches linear and angular finite differences for a spatial mixed chain', () => {
  const joints = [joint({ a: 2, alpha: 70, val: 35 }), joint({ id: 2, type: 'P', a: 1, alpha: -40, theta: 20, val: 3 }), joint({ id: 3, a: 2.5, alpha: 15, val: -50 })];
  const fk = forwardKinematics(joints);
  const h = 1e-6;
  joints.forEach((j, col) => {
    const step = h / (j.type === 'R' ? DEG : 1);
    const plus = forwardKinematics(joints.map((item, i) => ({ ...item, val: item.val + (i === col ? step : 0) })));
    const minus = forwardKinematics(joints.map((item, i) => ({ ...item, val: item.val - (i === col ? step : 0) })));
    const linear = plus.positions.at(-1).map((v, i) => (v - minus.positions.at(-1)[i]) / (2 * h));
    const r = matrixRows(fk.total, 3), rp = matrixRows(plus.total, 3), rm = matrixRows(minus.total, 3);
    const skew = r.map((_, i) => r.map(row => row.reduce((sum, value, j) => sum + (rp[i][j] - rm[i][j]) / (2 * h) * value, 0)));
    const angular = [skew[2][1], skew[0][2], skew[1][0]];
    nearVector(fk.jacobian.map(row => row[col]), [...linear, ...angular], 1e-7);
  });
});

test('RPY reconstruction, gimbal lock, and axis-angle at zero and pi', () => {
  for (const angles of [[20, -30, 40], [10, 90, 35], [-20, -90, 150], [180, 0, 0]]) {
    const rotation = rotationFromRPY(angles);
    const result = orientation(rotation);
    nearVector(rotationFromRPY(result.rpy).elements, rotation.elements, 1e-7);
    if (Math.abs(angles[1]) === 90) assert.equal(result.gimbalLock, true);
  }
  assert.equal(orientation(new Matrix4()).axis, null);
  const pi = orientation(rotationFromRPY([180, 0, 0]));
  near(pi.angle, 180);
  near(Math.abs(pi.axis[0]), 1);
});

test('rank diagnoses planar singularity without calling every 3-DOF spatial task singular', () => {
  const straight = forwardKinematics(planar);
  const bent = planar.map((j, i) => ({ ...j, val: i === 1 ? 60 : 0 }));
  const a = jacobianDiagnostics(planar, straight.jacobian, 'xy');
  const b = jacobianDiagnostics(bent, forwardKinematics(bent).jacobian, 'xy');
  assert.equal(a.rank, 1); assert.equal(a.condition, Infinity); near(a.determinant, 0);
  assert.equal(b.rank, 2); assert.equal(b.fullTask, true); assert.ok(Number.isFinite(b.condition));
  near(b.determinant, 6 * Math.sin(60 * DEG));
  const full = jacobianDiagnostics(DEFAULT_JOINTS, forwardKinematics(DEFAULT_JOINTS).jacobian, 'pose');
  assert.equal(full.rank, 3); assert.equal(full.deficient, false); assert.equal(full.fullTask, false);
});

test('SVD solves tall, wide, square and zero Jacobians with finite residuals', () => {
  for (const [joints, mode] of [[DEFAULT_JOINTS, 'xy'], [DEFAULT_JOINTS, 'pose'], [planar.map((j, i) => ({ ...j, val: i ? 40 : 20 })), 'xy'], [[joint()], 'position']]) {
    const fk = forwardKinematics(joints);
    const knownRates = joints.map((_, i) => (i + 1) * 0.1);
    const desired = multiplyVector(fk.jacobian, knownRates);
    const result = inverseVelocity(joints, fk.jacobian, desired, { mode, damping: 0 });
    assert.ok(result.rates.every(Number.isFinite));
    near(result.error, 0);
  }
  const zero = inverseVelocity([joint()], forwardKinematics([joint()]).jacobian, [1, 0, 0, 0, 0, 0], { damping: 0 });
  near(zero.rates[0], 0); assert.ok(zero.error > 0);
});

test('normalized conditioning is invariant under consistent length-unit changes', () => {
  const factor = 1000;
  const scaled = DEFAULT_JOINTS.map(j => ({ ...j, a: j.a * factor, d: j.d * factor, ...(j.type === 'P' ? { val: j.val * factor, min: j.min * factor, max: j.max * factor } : {}) }));
  const first = jacobianDiagnostics(DEFAULT_JOINTS, forwardKinematics(DEFAULT_JOINTS).jacobian, 'pose', 5);
  const second = jacobianDiagnostics(scaled, forwardKinematics(scaled).jacobian, 'pose', 5000);
  nearVector(first.singularValues, second.singularValues);
});

test('integration converts rad/s to degrees and reports bounded prismatic motion', () => {
  const result = integrateJoints(DEFAULT_JOINTS, [Math.PI / 2, 100, 0], 1);
  near(result.joints[0].val, 135); near(result.joints[1].val, 10);
  assert.deepEqual(result.limited, [2]);
});

test('inverse position escapes a straight-arm seed and reaches a known target', () => {
  const result = inverseKinematics(planar, { position: [3, 2, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  assert.equal(result.converged, true);
  nearVector(forwardKinematics(result.joints).positions.at(-1), [3, 2, 0], 1e-4);
});

test('inverse reports convergence achieved on the final allowed iteration', () => {
  const result = inverseKinematics([joint({ type: 'P', min: 0, max: 1 })],
    { position: [0, 0, 0.1], rpy: [0, 0, 0] }, { length: 1, damping: 0, maxIterations: 1 });
  assert.equal(result.converged, true);
  assert.equal(result.iterations, 1);
  near(result.joints[0].val, 0.1);
});

test('inverse RPR pose reaches a compatible changed target and handles 180-degree error', () => {
  const desired = DEFAULT_JOINTS.map((j, i) => ({ ...j, val: [20, 4, 65][i] }));
  const fk = forwardKinematics(desired);
  const result = inverseKinematics(DEFAULT_JOINTS, { position: fk.positions.at(-1), rpy: fk.orientation.rpy }, { mode: 'pose' });
  assert.equal(result.converged, true);
  nearVector(forwardKinematics(result.joints).total.elements, fk.total.elements, 1e-4);
  const turn = inverseKinematics([joint()], { position: [0, 0, 0], rpy: [0, 0, 180] }, { mode: 'pose' });
  assert.equal(turn.converged, true);
  near(Math.abs(turn.joints[0].val), 180, 1e-3);
});

test('unreachable and limit-constrained targets never report success or exceed limits', () => {
  for (const target of [[100, 100, 100], [3, 1, 4]]) {
    const result = inverseKinematics(DEFAULT_JOINTS, { position: target, rpy: [0, 0, 0] });
    assert.equal(result.converged, false);
    assert.ok(Number.isFinite(result.error));
    assert.ok(result.joints.every(j => j.val >= j.min && j.val <= j.max));
  }
  const limited = [joint({ type: 'P', min: 0, max: 1 })];
  const result = inverseKinematics(limited, { position: [0, 0, 4], rpy: [0, 0, 0] });
  assert.equal(result.converged, false); near(result.joints[0].val, 1);
});

test('validation rejects malformed models and accepts legacy models without speeds', () => {
  for (const data of [null, [], [null], [joint({ type: 'F' })], [joint({ val: NaN })], [joint({ a: Infinity })], [joint({ min: 2, max: 1 })], [joint({ val: 200 })], [joint(), joint()]]) {
    assert.ok(validateJoints(data).length > 0);
    assert.throws(() => normalizeJoints(data));
  }
  const legacy = DEFAULT_JOINTS.map(({ speed: _speed, ...j }) => j);
  assert.ok(normalizeJoints(legacy).every(j => j.speed === 0));
});
