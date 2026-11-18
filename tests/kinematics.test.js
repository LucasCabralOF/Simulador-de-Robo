import test from 'node:test';
import assert from 'node:assert/strict';
import { Matrix4 } from 'three';
import { DEG, DEFAULT_ENVIRONMENT, DEFAULT_JOINTS, DEFAULT_JOINT_COLORS, DEFAULT_MODEL_STYLE, JOINT_CHART_PALETTE, LINK_SHAPES, TOOL_LIST, TOOL_TYPES, computeManipulabilityEllipsoid, computeToolFrame, detectSelfCollision, dhMatrix, flangeRpyFromToolTarget, forwardKinematics, hexToHsl, hslToHex, integrateJoints, inverseKinematics, inverseVelocity, isDefaultModelStyle, jacobianDiagnostics, matrixRows, multiplyVector, normalizeJoints, normalizeModelStyle, orientation, resolveJointChartColor, resolveJointModelStyle, rotationFromRPY, solveBoundedTarget, staticJointEfforts, sweepJointsCollisionFree, validateJoints } from '../src/kinematics.js';
import { PRESET_LIST } from '../src/presets.js';

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

test('inverse converges via active-set projection when prismatic joint saturates at tight limits', () => {
  const constrainedRpr = [
    joint({ id: 1, type: 'R', a: 0, alpha: 90, val: 15, min: -90, max: 90 }),
    joint({ id: 2, type: 'P', a: 0, alpha: -90, val: 3, min: 0, max: 3 }),
    joint({ id: 3, type: 'R', a: 5, alpha: 0, val: 0, min: -180, max: 180 }),
  ];
  // Target requires saturating J2 at max = 3 while J1/J3 swing to (8, 0, 0)
  const res = inverseKinematics(constrainedRpr, { position: [8, 0, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  assert.equal(res.converged, true);
  assert.ok(res.joints.every(j => j.val >= j.min - 1e-9 && j.val <= j.max + 1e-9));
  nearVector(forwardKinematics(res.joints).positions.at(-1), [8, 0, 0], 1e-4);

  // Target with narrow prismatic range [0.5, 0.7] and non-zero angular reach
  const tightStroke = [
    joint({ id: 1, type: 'R', a: 0, alpha: 90, val: 0, min: -180, max: 180 }),
    joint({ id: 2, type: 'P', a: 0, alpha: -90, val: 0.6, min: 0.5, max: 0.7 }),
    joint({ id: 3, type: 'R', a: 5, alpha: 0, val: 0, min: -180, max: 180 }),
  ];
  const resTight = inverseKinematics(tightStroke, { position: [4, 3, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  assert.equal(resTight.converged, true);
  assert.ok(resTight.joints[1].val >= 0.5 && resTight.joints[1].val <= 0.7);
  nearVector(forwardKinematics(resTight.joints).positions.at(-1), [4, 3, 0], 1e-4);
});

test('validation rejects malformed models and accepts legacy models without speeds', () => {
  for (const data of [null, [], [null], [joint({ type: 'F' })], [joint({ val: NaN })], [joint({ a: Infinity })], [joint({ min: 2, max: 1 })], [joint({ val: 200 })], [joint(), joint()]]) {
    assert.ok(validateJoints(data).length > 0);
    assert.throws(() => normalizeJoints(data));
  }
  const legacy = DEFAULT_JOINTS.map(({ speed: _speed, ...j }) => j);
  assert.ok(normalizeJoints(legacy).every(j => j.speed === 0));
});

test('self-collision detector catches jackknife folds, 3D crossings, and spares all presets', () => {
  for (const preset of PRESET_LIST) {
    assert.equal(detectSelfCollision(preset.joints).colliding, false, `${preset.id} initial pose should not self-collide`);
  }

  // RPR folded straight back onto itself (q3 = 90 deg -> Elo 3 passes through Elo 2 & Base)
  const foldedRpr = DEFAULT_JOINTS.map((j, i) => (i === 2 ? { ...j, val: 90 } : j));
  const foldCol = detectSelfCollision(foldedRpr);
  assert.equal(foldCol.colliding, true);
  assert.ok(Array.isArray(foldCol.point) && foldCol.point.length === 3);

  // Planar 3R arm looping around so link 3 crosses link 1 in mid-space
  const looped3R = [
    joint({ id: 1, type: 'R', a: 4, val: 0, min: -180, max: 180 }),
    joint({ id: 2, type: 'R', a: 3, val: 120, min: -180, max: 180 }),
    joint({ id: 3, type: 'R', a: 4, val: 120, min: -180, max: 180 }),
  ];
  const crossCol = detectSelfCollision(looped3R);
  assert.equal(crossCol.colliding, true);
});

test('continuous sweep and integration block tunneling through self-collision boundaries', () => {
  // Swinging q3 from 30 deg across 90 deg to 120 deg must stop before colliding around 82 deg
  const targetJump = DEFAULT_JOINTS.map((j, i) => (i === 2 ? { ...j, val: 120 } : j));
  const swept = sweepJointsCollisionFree(DEFAULT_JOINTS, targetJump);
  assert.equal(swept.blocked, true);
  assert.ok(swept.joints[2].val < 83 && swept.joints[2].val > 80, `Expected sweep stop near 82 deg, got ${swept.joints[2].val}`);
  assert.equal(detectSelfCollision(swept.joints).colliding, false);

  // Differential integration velocity step must also stop safely at boundary
  const integrated = integrateJoints(DEFAULT_JOINTS, [0, 0, Math.PI], 0.5);
  assert.ok(integrated.collision !== null);
  assert.equal(detectSelfCollision(integrated.joints).colliding, false);
  assert.ok(integrated.joints[2].val < 83);
});

test('inverse kinematics never converges to or passes through a self-colliding configuration', () => {
  // Target at origin (0, 0, 0) could trivially fold RPR back if d2 = 5, q3 = 90; solver must never self-collide
  const resOrigin = inverseKinematics(DEFAULT_JOINTS, { position: [0, 0, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  assert.equal(detectSelfCollision(resOrigin.joints).colliding, false);

  // Reachable point on valid non-colliding branch (4, 1, 0)
  const resSafe = inverseKinematics(DEFAULT_JOINTS, { position: [4, 1, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  assert.equal(resSafe.converged, true);
  assert.equal(detectSelfCollision(resSafe.joints).colliding, false);
});

test('tool catalog, TCP kinematics, and mounted orientation offset solve tool pose accurately', () => {
  assert.equal(TOOL_LIST.length, 3);
  assert.deepEqual(Object.keys(TOOL_TYPES), ['gripper', 'vacuum', 'welder']);

  const fk = forwardKinematics(DEFAULT_JOINTS);
  const offsetRpy = [0, -90, 0];
  const welderTcp = computeToolFrame(fk.total, offsetRpy, 'welder');
  assert.equal(welderTcp.tool.id, 'welder');
  // Local +X rotated by -90 deg pitch points along flange +Z
  nearVector(welderTcp.approach, [0, 0, 1], 1e-6);

  // Default tool direction continues straight along flange +X_n (RPY [0, 0, 0])
  const defaultTcp = computeToolFrame(fk.total);
  const xFlange = [fk.total.elements[0], fk.total.elements[1], fk.total.elements[2]];
  const zFlange = [fk.total.elements[8], fk.total.elements[9], fk.total.elements[10]];
  const dotX = defaultTcp.approach[0] * xFlange[0] + defaultTcp.approach[1] * xFlange[1] + defaultTcp.approach[2] * xFlange[2];
  const dotZ = defaultTcp.approach[0] * zFlange[0] + defaultTcp.approach[1] * zFlange[1] + defaultTcp.approach[2] * zFlange[2];
  near(dotX, 1, 1e-6);
  near(dotZ, 0, 1e-6);

  // Inverse solve compensated by flangeRpyFromToolTarget achieves desired tool orientation [0, -90, 0]
  const ur5 = PRESET_LIST.find(p => p.id === 'ur5').joints;
  const targetToolRpy = [0, -90, 0];
  const flangeTargetRpy = flangeRpyFromToolTarget(targetToolRpy, [0, -45, 0]);
  const res = inverseKinematics(ur5, { position: [-4, 2, 4], rpy: flangeTargetRpy }, { mode: 'pose' });
  assert.equal(res.converged, true);
  const solvedTool = computeToolFrame(forwardKinematics(res.joints).total, [0, -45, 0], 'welder');
  nearVector(solvedTool.orientation.rotation.flat(), orientation(rotationFromRPY(targetToolRpy)).rotation.flat(), 1e-4);
});

test('environment collision detects workbench floor and blocks sphere/box obstacle penetration', () => {
  // Default environment (floorZ = -0.22, obstacle disabled) must never trigger false collision
  assert.equal(detectSelfCollision(DEFAULT_JOINTS, null, 'gripper', DEFAULT_ENVIRONMENT).colliding, false);

  // Raising workbench floor to Z = 0.5 catches RPR planar links lying at Z = 0
  const raisedFloorEnv = { ...DEFAULT_ENVIRONMENT, floorZ: 0.5 };
  const floorHit = detectSelfCollision(DEFAULT_JOINTS, null, 'gripper', raisedFloorEnv);
  assert.equal(floorHit.colliding, true);
  assert.ok(floorHit.message.includes('piso/bancada'));

  // Placing a sphere obstacle at [4, 3, 0] (safe from default pose [4.83, 1.29, 0]) blocks q1 swing 45 -> 110
  const sphereEnv = {
    floorEnabled: true,
    floorZ: -0.22,
    obstacle: { enabled: true, shape: 'sphere', position: [4, 3, 0], radius: 1.0 },
  };
  assert.equal(detectSelfCollision(DEFAULT_JOINTS, null, 'gripper', sphereEnv).colliding, false);
  const swingTarget = DEFAULT_JOINTS.map((j, i) => (i === 0 ? { ...j, val: 110 } : j));
  const sweptSphere = sweepJointsCollisionFree(DEFAULT_JOINTS, swingTarget, 'gripper', sphereEnv);
  assert.equal(sweptSphere.blocked, true);
  assert.ok(sweptSphere.collision.message.includes('Obstáculo 3D'));
  assert.ok(sweptSphere.joints[0].val < 80);
  assert.equal(detectSelfCollision(sweptSphere.joints, null, 'gripper', sphereEnv).colliding, false);

  // Placing a box obstacle right onto target coordinates blocks IK convergence on that point
  const boxEnv = {
    floorEnabled: true,
    floorZ: -0.22,
    obstacle: { enabled: true, shape: 'box', position: [2, 5, 0], radius: 1.0 },
  };
  const blockedIk = inverseKinematics(DEFAULT_JOINTS, { position: [2, 5, 0], rpy: [0, 0, 0] }, {
    mode: 'xy',
    environment: boxEnv,
  });
  assert.equal(blockedIk.converged, false);
});

test('staticJointEfforts computes tau = J^T * F with correct R/P units', () => {
  const fk = forwardKinematics(DEFAULT_JOINTS);
  const wrench = [0, -20, 0, 0, 0, 0];
  const res = staticJointEfforts(DEFAULT_JOINTS, fk.jacobian, wrench);

  assert.deepEqual(res.labels, ['T1', 'F2', 'T3']);
  assert.deepEqual(res.units, ['N·u', 'N', 'N·u']);

  const [x, , ] = fk.positions.at(-1);
  // For DEFAULT_JOINTS planar in XY: tau_1 = J_11*Fx + J_21*Fy = 0 + x*(-20)
  near(res.efforts[0], x * -20, 1e-8);
  // J_22 = -cos(45 deg) for d2, so F2 = (-cos 45)*(-20) = 20*cos(45 deg)
  near(res.efforts[1], 20 * Math.cos(45 * DEG), 1e-8);
  // T3 = a3*cos(75 deg)*(-20)
  near(res.efforts[2], 5 * Math.cos(75 * DEG) * -20, 1e-8);
});

test('solveBoundedTarget clamps gizmo drags strictly within joint, workspace, singularity, and obstacle limits', () => {
  // 1. Out-of-reach diagonal pull on RPR clamps strictly to circular boundary r <= 15 and matches FK
  const overDiag = solveBoundedTarget(DEFAULT_JOINTS, { position: [12, 12, 0], rpy: [0, 0, 75] }, { mode: 'xy' });
  assert.equal(overDiag.blocked, false);
  assert.equal(overDiag.converged, false);
  const rDiag = Math.hypot(overDiag.position[0], overDiag.position[1]);
  assert.ok(rDiag <= 15 + 1e-4 && rDiag > 14.9, `Expected boundary r ~ 15, got ${rDiag}`);
  nearVector(overDiag.position, forwardKinematics(overDiag.joints).positions.at(-1), 1e-6);

  // 2. PRP cylindrical bounds clamp strictly inside d1 in [0.5, 8] and d3 in [0.5, 8]
  const prp = PRESET_LIST.find(p => p.id === 'prp');
  const overPrp = solveBoundedTarget(prp.joints, { position: [12, 12, 14], rpy: [0, 0, 0] }, {
    mode: 'position',
    toolOffsetRpy: prp.toolOffsetRpy,
  });
  assert.equal(overPrp.blocked, false);
  assert.ok(overPrp.position[2] <= 8 + 1e-6 && overPrp.position[2] >= 0.5 - 1e-6);
  const rPrp = Math.hypot(overPrp.position[0], overPrp.position[1]);
  assert.ok(rPrp <= 8 + 1e-4 && rPrp >= 0.5 - 1e-4);

  // 3. SCARA stretched completely straight (theta2 = 0) escapes singularity when dragged straight inward
  const scara = PRESET_LIST.find(p => p.id === 'scara');
  const straightScara = scara.joints.map((j, idx) => (idx < 2 ? { ...j, val: 0 } : j));
  const straightEnd = forwardKinematics(straightScara).positions.at(-1);
  const inward = solveBoundedTarget(
    straightScara,
    { position: [straightEnd[0] - 1.5, straightEnd[1], straightEnd[2]], rpy: [0, 0, 0] },
    { mode: 'position', toolOffsetRpy: scara.toolOffsetRpy }
  );
  assert.equal(inward.blocked, false);
  assert.equal(inward.converged, true);
  nearVector(inward.position, [straightEnd[0] - 1.5, straightEnd[1], straightEnd[2]], 1e-3);

  // 4. Obstacle collision stops gizmo outside the obstacle surface
  const boxEnv = {
    floorEnabled: true,
    floorZ: -0.22,
    obstacle: { enabled: true, shape: 'box', position: [3.5, 2.5, 0], radius: 1.0 },
  };
  const blockedObs = solveBoundedTarget(DEFAULT_JOINTS, { position: [3.5, 2.5, 0], rpy: [0, 0, 75] }, {
    mode: 'xy',
    environment: boxEnv,
  });
  assert.equal(blockedObs.converged, false);
  assert.equal(detectSelfCollision(blockedObs.joints, null, 'gripper', boxEnv).colliding, false);

  // 5. Impossible out-of-plane Roll on planar RPR clamps Roll to 0
  const rotRoll = solveBoundedTarget(DEFAULT_JOINTS, { position: forwardKinematics(DEFAULT_JOINTS).positions.at(-1), rpy: [45, 0, 75] }, {
    mode: 'pose',
    toolOffsetRpy: [0, 0, 0],
    lockPosition: true,
  });
  near(rotRoll.rpy[0], 0, 1e-4);
  near(rotRoll.rpy[1], 0, 1e-4);

  // 6. Continuous multi-frame RPY orientation drag keeps end-effector XYZ position strictly locked
  const ur5 = PRESET_LIST.find(p => p.id === 'ur5');
  let dragJoints = ur5.joints;
  const anchorPos = forwardKinematics(dragJoints).positions.at(-1);
  for (let stepIdx = 1; stepIdx <= 25; stepIdx++) {
    const res = solveBoundedTarget(
      dragJoints,
      { position: anchorPos, rpy: [90 + stepIdx * 4, stepIdx * 2, -135 + stepIdx * 3] },
      { mode: 'pose', toolOffsetRpy: ur5.toolOffsetRpy, lockPosition: true }
    );
    assert.equal(res.blocked, false);
    dragJoints = res.joints;
    const stepPos = forwardKinematics(dragJoints).positions.at(-1);
    nearVector(stepPos, anchorPos, 1e-4);
  }

  // 7. Continuous multi-frame XYZ translation drag keeps end-effector RPY orientation strictly locked
  let transJoints = ur5.joints;
  const anchorRpy = computeToolFrame(forwardKinematics(transJoints).total, ur5.toolOffsetRpy).orientation.rpy;
  for (let stepIdx = 1; stepIdx <= 20; stepIdx++) {
    const curPos = forwardKinematics(transJoints).positions.at(-1);
    const res = solveBoundedTarget(
      transJoints,
      { position: [curPos[0] - 0.3, curPos[1] + 0.2, curPos[2] - 0.1], rpy: anchorRpy },
      { mode: 'position', toolOffsetRpy: ur5.toolOffsetRpy, lockOrientation: true }
    );
    if (!res.blocked) transJoints = res.joints;
    const stepToolRpy = computeToolFrame(forwardKinematics(transJoints).total, ur5.toolOffsetRpy).orientation.rpy;
    nearVector(stepToolRpy, anchorRpy, 0.05);
  }
  const scaraStart = forwardKinematics(scara.joints);
  const scaraPos0 = scaraStart.positions.at(-1);
  const scaraRpy0 = computeToolFrame(scaraStart.total, scara.toolOffsetRpy).orientation.rpy;
  const scaraTrans = solveBoundedTarget(
    scara.joints,
    { position: [scaraPos0[0] - 0.8, scaraPos0[1] + 0.6, scaraPos0[2] - 0.5], rpy: scaraRpy0 },
    { mode: 'position', toolOffsetRpy: scara.toolOffsetRpy, lockOrientation: true }
  );
  assert.equal(scaraTrans.blocked, false);
  const scaraEnd = forwardKinematics(scaraTrans.joints);
  nearVector(scaraEnd.positions.at(-1), [scaraPos0[0] - 0.8, scaraPos0[1] + 0.6, scaraPos0[2] - 0.5], 1e-3);
  nearVector(computeToolFrame(scaraEnd.total, scara.toolOffsetRpy).orientation.rpy, scaraRpy0, 0.05);
});

test('normalizeModelStyle clamps dimensions, validates link shapes, and tracks defaults', () => {
  assert.deepEqual(Object.keys(LINK_SHAPES), ['cylinder', 'square', 'hex', 'flat']);
  assert.equal(isDefaultModelStyle(DEFAULT_MODEL_STYLE), true);

  const clamped = normalizeModelStyle({
    linkShape: 'invalid',
    linkRadius: 99,
    revRadius: 0.001,
    revLength: NaN,
    prismWidth: 5,
    prismLength: -1,
  });
  assert.equal(clamped.linkShape, 'cylinder');
  assert.equal(clamped.linkRadius, 0.35);
  assert.equal(clamped.revRadius, 0.08);
  assert.equal(clamped.revLength, DEFAULT_MODEL_STYLE.revLength);
  assert.equal(clamped.prismWidth, 1.0);
  assert.equal(clamped.prismLength, 0.15);
  assert.equal(isDefaultModelStyle(clamped), false);

  const custom = normalizeModelStyle({ linkShape: 'hex', linkRadius: 0.12, revRadius: 0.45, revLength: 0.6, prismWidth: 0.5, prismLength: 0.8 });
  assert.deepEqual(custom, {
    linkShape: 'hex',
    linkRadius: 0.12,
    revRadius: 0.45,
    revLength: 0.6,
    prismWidth: 0.5,
    prismLength: 0.8,
  });

  // Per-joint and per-link customization inherits global defaults for untouched fields
  const withPerJoint = normalizeModelStyle({
    ...custom,
    perJoint: {
      1: { linkShape: 'square', revRadius: 99 },
      badKey: { linkRadius: 0.2 },
    },
  });
  assert.deepEqual(withPerJoint.perJoint, { 1: { linkShape: 'square', revRadius: 0.8 } });
  assert.equal(isDefaultModelStyle({ ...DEFAULT_MODEL_STYLE, perJoint: { 1: { revRadius: 0.5 } } }), false);

  const resolvedJ1 = resolveJointModelStyle(withPerJoint, 1);
  assert.equal(resolvedJ1.overridden, true);
  assert.equal(resolvedJ1.linkShape, 'square');
  assert.equal(resolvedJ1.revRadius, 0.8);
  assert.equal(resolvedJ1.linkRadius, 0.12);

  const resolvedJ2 = resolveJointModelStyle(withPerJoint, 2);
  assert.equal(resolvedJ2.overridden, false);
  assert.equal(resolvedJ2.linkShape, 'hex');
});

test('custom joint and link colors normalize, resolve per-joint/global fallbacks, and synchronize chart palette', () => {
  const defaultStyle = normalizeModelStyle({
    ...DEFAULT_MODEL_STYLE,
    revColor: DEFAULT_JOINT_COLORS.revColor,
    prismColor: 'invalid',
  });
  assert.equal(isDefaultModelStyle(defaultStyle), true);
  assert.equal(defaultStyle.revColor, undefined);
  const s1Default = resolveJointModelStyle(defaultStyle, 1, 'R', 0);
  const s2Default = resolveJointModelStyle(defaultStyle, 2, 'P', 1);
  assert.equal(s1Default.jointColor, JOINT_CHART_PALETTE[0]);
  assert.equal(s2Default.jointColor, JOINT_CHART_PALETTE[1]);
  assert.equal(resolveJointChartColor(defaultStyle, { id: 1, type: 'R' }, 0), s1Default.jointColor);
  assert.equal(resolveJointChartColor(defaultStyle, { id: 2, type: 'P' }, 1), s2Default.jointColor);

  const colored = normalizeModelStyle({
    ...DEFAULT_MODEL_STYLE,
    revColor: '#FF5500',
    linkColor: '#112233',
    perJoint: {
      2: { jointColor: '#AA33CC', linkColor: 'not-a-hex' },
    },
  });
  assert.equal(isDefaultModelStyle(colored), false);
  assert.equal(colored.revColor, '#ff5500');
  assert.equal(colored.linkColor, '#112233');
  assert.deepEqual(colored.perJoint, { 2: { jointColor: '#aa33cc' } });

  const s1 = resolveJointModelStyle(colored, 1, 'R');
  assert.equal(s1.jointColor, '#ff5500');
  assert.equal(s1.linkColor, '#112233');
  assert.equal(resolveJointChartColor(colored, { id: 1, type: 'R' }, 0), '#ff5500');

  const s2 = resolveJointModelStyle(colored, 2, 'P');
  assert.equal(s2.jointColor, '#aa33cc');
  assert.equal(s2.prismColor, '#aa33cc');
  assert.equal(s2.linkColor, '#112233');
  assert.equal(resolveJointChartColor(colored, { id: 2, type: 'P' }, 1), '#aa33cc');

  assert.equal(JOINT_CHART_PALETTE.length, 24);
  assert.equal(hslToHex(0, 100, 50), '#ff0000');
  assert.equal(hslToHex(120, 100, 50), '#00ff00');
  assert.equal(hslToHex(240, 100, 50), '#0000ff');
  assert.deepEqual(hexToHsl('#ff0000'), { h: 0, s: 100, l: 50 });
  assert.deepEqual(hexToHsl('#808080'), { h: 0, s: 0, l: 50 });
  const roundtrip = hexToHsl(hslToHex(210, 82, 62));
  assert.ok(Math.abs(roundtrip.h - 210) <= 1);
  assert.ok(Math.abs(roundtrip.s - 82) <= 2);
  assert.ok(Math.abs(roundtrip.l - 62) <= 2);
});

test('computeManipulabilityEllipsoid yields valid SVD radii, unit quaternion, and detects planar flattening', () => {
  assert.equal(computeManipulabilityEllipsoid(null), null);

  // Planar RPR has zero velocity capability along Z (s3 ≈ 0)
  const rprFk = forwardKinematics(DEFAULT_JOINTS);
  const rprEll = computeManipulabilityEllipsoid(rprFk.jacobian);
  assert.ok(rprEll);
  assert.equal(rprEll.singularValues.length, 3);
  assert.ok(rprEll.singularValues[0] >= rprEll.singularValues[1]);
  assert.ok(rprEll.singularValues[1] >= rprEll.singularValues[2]);
  near(rprEll.singularValues[2], 0, 1e-8);
  near(rprEll.manipulability, 0, 1e-8);
  assert.ok(rprEll.radii.every(r => r >= 0.04 && r <= 0.55));
  const qNormRpr = Math.hypot(...rprEll.quaternion);
  near(qNormRpr, 1, 1e-6);

  // 6-DOF UR5 in a general pose has non-zero 3D manipulability volume
  const ur5 = PRESET_LIST.find(p => p.id === 'ur5').joints;
  const ur5Fk = forwardKinematics(ur5);
  const ur5Ell = computeManipulabilityEllipsoid(ur5Fk.jacobian);
  assert.ok(ur5Ell);
  assert.ok(ur5Ell.singularValues[2] > 0.1);
  assert.ok(ur5Ell.manipulability > 0.5);
  near(Math.hypot(...ur5Ell.quaternion), 1, 1e-6);
});
