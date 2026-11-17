import test from 'node:test';
import assert from 'node:assert/strict';
import { PRESETS, PRESET_LIST } from '../src/presets.js';
import { DEFAULT_JOINTS, forwardKinematics, inverseKinematics, validateJoints } from '../src/kinematics.js';
import { computeCartesianBounds, generateWorkspaceCloud } from '../src/workspace.js';
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

test('workspace cloud generates points, vertex colors and bounding volume', () => {
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
