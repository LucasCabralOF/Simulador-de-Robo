import test from 'node:test';
import assert from 'node:assert/strict';
import { PRESETS, PRESET_LIST } from '../src/presets.js';
import { forwardKinematics, validateJoints } from '../src/kinematics.js';
import { generateWorkspaceCloud } from '../src/workspace.js';
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
});
