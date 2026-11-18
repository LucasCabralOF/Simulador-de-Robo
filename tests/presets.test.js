import test from 'node:test';
import assert from 'node:assert/strict';
import { PRESETS, PRESET_LIST } from '../src/presets.js';
import { DEFAULT_JOINTS, computeToolFrame, forwardKinematics, inverseKinematics, validateJoints } from '../src/kinematics.js';
import { computeCartesianBounds, generateWorkspaceCloud, heatColor } from '../src/workspace.js';
import { generateCpp, generatePython, generateRos2 } from '../src/codeGenerator.js';

test('all robot presets pass strict kinematic validation', () => {
  for (const preset of PRESET_LIST) {
    const errors = validateJoints(preset.joints);
    assert.deepEqual(errors, [], `Preset ${preset.name} has validation errors: ${errors.join(', ')}`);
    assert.equal(preset.joints.length, preset.dof, `DOF mismatch for ${preset.name}`);
  }
});

test('all robot presets compute valid forward kinematics', () => {
  for (const preset of PRESET_LIST) {
    const fk = forwardKinematics(preset.joints);
    assert.equal(fk.positions.length, preset.joints.length + 1);
    const end = fk.positions.at(-1);
    assert.ok(end.every(Number.isFinite), `Non-finite end-effector for ${preset.name}`);
    assert.ok(Number.isFinite(fk.orientation.determinant));
    assert.ok(Math.abs(fk.orientation.determinant - 1) < 1e-4, `Orientation det not 1 for ${preset.name}`);
  }
});

test('workspace cloud generates points, vertex colors (blue near singularity to green at high dexterity) and bounding volume', () => {
  const singularRgb = heatColor(0);
  const midRgb = heatColor(0.5);
  const healthyRgb = heatColor(1);
  assert.ok(singularRgb[2] > 0.9 && singularRgb[0] < 0.25, 'Singularidades devem tender ao azul');
  assert.ok(midRgb[1] > 0.75 && midRgb[2] > 0.75, 'Destreza intermediária deve passar pelo ciano');
  assert.ok(healthyRgb[1] > 0.85 && healthyRgb[0] < 0.25, 'Alta manipulabilidade deve tender ao verde');

  const scara = PRESETS.scara.joints;
  const cloud = generateWorkspaceCloud(scara, 200);
  assert.equal(cloud.count, 200);
  assert.equal(cloud.positions.length, 600);
  assert.equal(cloud.colors.length, 600);
  assert.ok(cloud.boundingVolume > 0, 'Bounding volume must be positive');
  assert.ok(cloud.bounds.dx > 0 && cloud.bounds.dy > 0 && cloud.bounds.dz >= 0);
  assert.ok(Number.isFinite(cloud.maxManipulability));
});

test('code generator creates valid C++, Python and ROS2 templates', () => {
  const puma = PRESETS.puma560.joints;
  const cpp = generateCpp(puma, 'Puma560');
  assert.ok(cpp.includes('void forwardKinematics'));
  assert.ok(cpp.includes('dhMatrix'));
  assert.ok(cpp.includes('float q[6]'));

  const py = generatePython(puma, 'Puma560');
  assert.ok(py.includes('def forward_kinematics(q):'));
  assert.ok(py.includes('import numpy as np'));

  const ros2 = generateRos2(puma, 'Puma560');
  assert.ok(ros2.includes('class RobotKinematicsNode'));
  assert.ok(ros2.includes('rclpy.init'));

  const toolCfg = { toolType: 'welder', toolOffsetRpy: [10, -45, 0], toolGrip: 80 };
  const cppTool = generateCpp(puma, 'Puma560', toolCfg);
  assert.ok(cppTool.includes('TOOL_TYPE = "welder"'));
  assert.ok(cppTool.includes('TOOL_OFFSET_RPY[3] = {10f, -45f, 0f}'));
  assert.ok(cppTool.includes('computeToolTcp'));

  const pyTool = generatePython(puma, 'Puma560', toolCfg);
  assert.ok(pyTool.includes("'type': 'welder'"));
  assert.ok(pyTool.includes('compute_tool_tcp'));

  const rosTool = generateRos2(puma, 'Puma560', toolCfg);
  assert.ok(rosTool.includes("self.declare_parameter('tool_type', 'welder')"));
  assert.ok(rosTool.includes("'/tcp_pose'"));
});

test('computeCartesianBounds estimates physical reach and auto-adjusts to joint limits', () => {
  const defaultBounds = computeCartesianBounds(DEFAULT_JOINTS);
  assert.ok(Math.abs(defaultBounds.min[0] - (-15)) < 1e-3, `Expected minX ~ -15, got ${defaultBounds.min[0]}`);
  assert.ok(Math.abs(defaultBounds.max[0] - 15) < 1e-3, `Expected maxX ~ 15, got ${defaultBounds.max[0]}`);
  assert.ok(Math.abs(defaultBounds.min[1] - (-15)) < 1e-3, `Expected minY ~ -15, got ${defaultBounds.min[1]}`);
  assert.ok(Math.abs(defaultBounds.max[1] - 15) < 1e-3, `Expected maxY ~ 15, got ${defaultBounds.max[1]}`);
  assert.equal(defaultBounds.span[2], 0);

  // Restrict prismatic joint max from 10 to 4 -> max reach shrinks from 15 to 9
  const restricted = DEFAULT_JOINTS.map((j, i) => i === 1 ? { ...j, max: 4, val: 2 } : j);
  const restrictedBounds = computeCartesianBounds(restricted);
  assert.ok(Math.abs(restrictedBounds.max[0] - 9) < 1e-3, `Expected restricted maxX ~ 9, got ${restrictedBounds.max[0]}`);
  assert.ok(Math.abs(restrictedBounds.min[0] - (-9)) < 1e-3, `Expected restricted minX ~ -9, got ${restrictedBounds.min[0]}`);

  // All presets produce valid finite bounds containing their initial FK position
  for (const preset of PRESET_LIST) {
    const b = computeCartesianBounds(preset.joints);
    const pos = forwardKinematics(preset.joints).positions.at(-1);
    for (let axis = 0; axis < 3; axis++) {
      assert.ok(b.min[axis] <= pos[axis] + 1e-6 && b.max[axis] >= pos[axis] - 1e-6);
    }
  }
});

test('computeCartesianBounds adapts slider limits to current robot pose and lateral slice', () => {
  // RPR at Y = 12 (max radius R = 15): X reach must adapt from [-15, 15] to ~[-9, 9]
  const atY12 = inverseKinematics(DEFAULT_JOINTS, { position: [0, 12, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  const endY12 = forwardKinematics(atY12.joints).positions.at(-1);
  const boundsY12 = computeCartesianBounds(atY12.joints, endY12, 'xy');
  assert.ok(Math.abs(boundsY12.max[0] - 9) < 0.1, `Expected X max ~ 9 at Y=12, got ${boundsY12.max[0]}`);
  assert.ok(Math.abs(boundsY12.min[0] - (-9)) < 0.1, `Expected X min ~ -9 at Y=12, got ${boundsY12.min[0]}`);

  // UR5 at lateral elevated pose [-7.79, -2.78, 3.40]: X/Y/Z bounds tighten strictly below global AABB
  const ur5 = PRESETS.ur5.joints;
  const globalUr5 = computeCartesianBounds(ur5);
  const lateralUr5 = inverseKinematics(ur5, { position: [-7.79, -2.78, 3.40], rpy: [0, 0, 0] }, { mode: 'position' });
  const endUr5 = forwardKinematics(lateralUr5.joints).positions.at(-1);
  const sliceUr5 = computeCartesianBounds(lateralUr5.joints, endUr5, 'position');
  assert.ok(sliceUr5.max[0] < globalUr5.max[0] - 0.3, `Expected contracted UR5 X max < ${globalUr5.max[0]}, got ${sliceUr5.max[0]}`);
  assert.ok(sliceUr5.max[1] < globalUr5.max[1] - 4.0, `Expected contracted UR5 Y max < ${globalUr5.max[1]}, got ${sliceUr5.max[1]}`);
  assert.ok(sliceUr5.max[2] < globalUr5.max[2] - 4.0, `Expected contracted UR5 Z max < ${globalUr5.max[2]}, got ${sliceUr5.max[2]}`);
  for (let axis = 0; axis < 3; axis++) {
    assert.ok(sliceUr5.min[axis] <= endUr5[axis] + 1e-6 && sliceUr5.max[axis] >= endUr5[axis] - 1e-6);
  }
});

test('PRP Espacial preset satisfies cylindrical analytical kinematics from P2 Question 3', () => {
  const prp = PRESETS.prp.joints;
  const fk = forwardKinematics(prp);
  const d1 = prp[0].val;
  const theta2Rad = (prp[1].val * Math.PI) / 180;
  const d3 = prp[2].val;
  const end = fk.positions.at(-1);

  assert.ok(Math.abs(end[0] - d3 * Math.cos(theta2Rad)) < 1e-9);
  assert.ok(Math.abs(end[1] - d3 * Math.sin(theta2Rad)) < 1e-9);
  assert.ok(Math.abs(end[2] - d1) < 1e-9);
});

test('all default robot presets orient the end-effector straight outward along the last connected link', () => {
  for (const preset of PRESET_LIST) {
    assert.ok(Array.isArray(preset.toolOffsetRpy) && preset.toolOffsetRpy.length === 3, `${preset.id} missing toolOffsetRpy`);
    const fk = forwardKinematics(preset.joints);
    const tool = computeToolFrame(fk.total, preset.toolOffsetRpy);
    let linkDir = null;
    for (let i = fk.positions.length - 1; i > 0; i--) {
      const dx = fk.positions[i][0] - fk.positions[i - 1][0];
      const dy = fk.positions[i][1] - fk.positions[i - 1][1];
      const dz = fk.positions[i][2] - fk.positions[i - 1][2];
      const len = Math.hypot(dx, dy, dz);
      if (len > 1e-6) {
        linkDir = [dx / len, dy / len, dz / len];
        break;
      }
    }
    assert.ok(linkDir, `${preset.id} has no non-zero link`);
    const dot = tool.approach[0] * linkDir[0] + tool.approach[1] * linkDir[1] + tool.approach[2] * linkDir[2];
    assert.ok(Math.abs(dot - 1) < 1e-6, `${preset.id} tool approach not parallel/straight out from last link (dot=${dot})`);
  }
});

test('computeCartesianBounds detects interior and boundary singularity intervals inside active axis spans', () => {
  const rprCenter = inverseKinematics(DEFAULT_JOINTS, { position: [5, 0, 0], rpy: [0, 0, 0] }, { mode: 'xy' });
  const rprEnd = forwardKinematics(rprCenter.joints).positions.at(-1);
  const rprBounds = computeCartesianBounds(rprCenter.joints, rprEnd, 'xy');
  assert.ok(Array.isArray(rprBounds.singularities) && rprBounds.singularities.length === 3);
  for (let axis = 0; axis < 2; axis++) {
    assert.ok(rprBounds.singularities[axis].length > 0, `Expected singularity marker on RPR axis ${axis}`);
    for (const marker of rprBounds.singularities[axis]) {
      assert.ok(Number.isFinite(marker.start) && Number.isFinite(marker.end));
      assert.ok(marker.start <= marker.end + 1e-9);
      assert.ok(marker.start >= rprBounds.min[axis] - 1e-2 && marker.end <= rprBounds.max[axis] + 1e-2);
      assert.ok(marker.value >= marker.start - 1e-2 && marker.value <= marker.end + 1e-2);
      assert.ok(['boundary', 'interior'].includes(marker.kind));
      assert.ok(typeof marker.label === 'string' && marker.label.length > 0);
    }
  }

  const ur5 = PRESETS.ur5.joints;
  const ur5End = forwardKinematics(ur5).positions.at(-1);
  const ur5Bounds = computeCartesianBounds(ur5, ur5End, 'position');
  assert.ok(ur5Bounds.singularities.some(list => list.length > 0), 'Expected UR5 Cartesian sliders to surface singularity intervals');
  for (let axis = 0; axis < 3; axis++) {
    for (const marker of ur5Bounds.singularities[axis]) {
      assert.ok(Number.isFinite(marker.start) && Number.isFinite(marker.end) && marker.start <= marker.end + 1e-9);
      assert.ok(marker.value >= ur5Bounds.min[axis] - 1e-2 && marker.value <= ur5Bounds.max[axis] + 1e-2);
    }
  }

  const scaraY0 = inverseKinematics(PRESETS.scara.joints, { position: [3, 0, 0.2], rpy: [0, 0, 0] }, { mode: 'position' });
  const scaraEnd = forwardKinematics(scaraY0.joints).positions.at(-1);
  const scaraBounds = computeCartesianBounds(scaraY0.joints, scaraEnd, 'position');
  const innerDeadZone = scaraBounds.singularities[0].find(m => m.kind === 'interior' && (m.end - m.start) > 1.0);
  assert.ok(innerDeadZone, 'Expected SCARA X slice at Y=0 to report the central unreachable singular interval');
});
