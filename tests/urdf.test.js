import test from 'node:test';
import assert from 'node:assert/strict';
import { exportToURDF, parseURDF } from '../src/urdfParser.js';
import { PRESETS } from '../src/presets.js';
import { normalizeJoints, forwardKinematics } from '../src/kinematics.js';

test('exportToURDF generates valid URDF XML structure with links and joints', () => {
  const scara = PRESETS.scara.joints;
  const urdf = exportToURDF(scara, 'SCARA_Industrial');

  assert.match(urdf, /<\?xml version="1.0" encoding="utf-8"\?>/);
  assert.match(urdf, /<robot name="scara_industrial">/);
  assert.match(urdf, /<link name="base_link">/);
  assert.match(urdf, /<joint name="joint_1" type="revolute">/);
  assert.match(urdf, /<joint name="joint_3" type="prismatic">/);
  assert.match(urdf, /<joint name="joint_ee_fixed" type="fixed">/);
  assert.match(urdf, /<\/robot>/);
});

test('round-trip export and parse URDF reconstructs kinematic parameters', () => {
  const original = PRESETS.scara.joints;
  const urdf = exportToURDF(original, 'SCARA');
  const parsed = parseURDF(urdf);

  assert.equal(parsed.length, original.length);
  assert.equal(parsed[0].type, original[0].type);
  assert.equal(parsed[2].type, 'P'); // 3rd joint of SCARA is prismatic
  assert.equal(parsed[0].a, original[0].a);
  assert.equal(parsed[1].a, original[1].a);

  // Validate that normalized joints produce non-divergent forward kinematics
  const normalized = normalizeJoints(parsed);
  const fk = forwardKinematics(normalized);
  assert.ok(Array.isArray(fk.positions));
  assert.equal(fk.positions.length, original.length + 1);
});

test('parseURDF handles Stanford and PUMA presets', () => {
  const puma = PRESETS.puma560.joints;
  const pumaUrdf = exportToURDF(puma, 'PUMA_560');
  const parsedPuma = parseURDF(pumaUrdf);
  assert.equal(parsedPuma.length, 6);
  assert.ok(parsedPuma.every(j => j.type === 'R'));

  const stanford = PRESETS.stanford.joints;
  const stanfordUrdf = exportToURDF(stanford, 'Stanford_Arm');
  const parsedStanford = parseURDF(stanfordUrdf);
  assert.equal(parsedStanford.length, 6);
  assert.equal(parsedStanford[2].type, 'P'); // Joint 3 of Stanford is prismatic
});

test('parseURDF throws helpful errors for empty or invalid XML', () => {
  assert.throws(() => parseURDF(''), /Arquivo URDF vazio ou inválido/);
  assert.throws(() => parseURDF('<something>other</something>'), /tag <robot> não encontrada/);
  assert.throws(() => parseURDF('<robot name="empty"><link name="base"/></robot>'), /Nenhuma junta móvel/);
});

test('URDF round-trip preserves toolType and toolOffsetRpy via non-enumerable toolConfig', () => {
  const joints = PRESETS.ur5.joints;
  const urdf = exportToURDF(joints, 'UR5_Welder', { toolType: 'welder', toolOffsetRpy: [0, -45, 15] });
  assert.match(urdf, /<tool_config type="welder" offset_rpy="0 -45 15"\/>/);
  assert.match(urdf, /<link name="tcp_link"\/>/);
  const parsed = parseURDF(urdf);
  assert.equal(parsed.length, 6);
  assert.deepEqual(parsed.toolConfig, { toolType: 'welder', toolOffsetRpy: [0, -45, 15] });
});

test('URDF round-trip preserves custom modelStyle link shape and joint sizes (including perJoint)', () => {
  const joints = PRESETS.rpr.joints;
  const modelStyle = {
    linkShape: 'square',
    linkRadius: 0.14,
    revRadius: 0.52,
    revLength: 0.65,
    prismWidth: 0.58,
    prismLength: 0.95,
    perJoint: {
      2: { linkShape: 'flat', linkRadius: 0.2, prismWidth: 0.72, prismLength: 1.1 },
    },
  };
  const urdf = exportToURDF(joints, 'Styled_RPR', { toolType: 'gripper', toolOffsetRpy: [0, 0, 0], modelStyle });
  assert.match(urdf, /<model_style link_shape="square" link_radius="0\.14" rev_radius="0\.52" rev_length="0\.65" prism_width="0\.58" prism_length="0\.95">/);
  assert.match(urdf, /<joint_style id="2" link_shape="flat" link_radius="0\.2" prism_width="0\.72" prism_length="1\.1"\/>/);
  assert.match(urdf, /<box size="0\.48 0\.22 /);
  const parsed = parseURDF(urdf);
  assert.deepEqual(parsed.toolConfig.modelStyle, modelStyle);
});
