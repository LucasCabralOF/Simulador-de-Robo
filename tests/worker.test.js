import test from 'node:test';
import assert from 'node:assert/strict';
import {
  isWorkerSupported,
  runSingularityAnalysisAsync,
  runWorkspaceCloudAsync,
  terminateWorkspaceWorker,
} from '../src/workers/workspaceClient.js';
import { analyzeRobotSingularities, generateWorkspaceCloud } from '../src/workspace.js';
import { forwardKinematics } from '../src/kinematics.js';
import { PRESETS } from '../src/presets.js';

test('workspaceClient executes workspace cloud cleanly with synchronous Node fallback', async () => {
  assert.equal(isWorkerSupported(), false);

  const cloud = await runWorkspaceCloudAsync(PRESETS.ur5.joints, 500, null);
  assert.ok(cloud.positions instanceof Float32Array);
  assert.ok(cloud.colors instanceof Float32Array);
  assert.ok(cloud.count > 0);
  assert.equal(cloud.positions.length, cloud.count * 3);
  assert.equal(cloud.colors.length, cloud.count * 3);
  assert.ok(Number.isFinite(cloud.boundingVolume) && cloud.boundingVolume > 0);
  assert.ok(Number.isFinite(cloud.maxManipulability) && cloud.maxManipulability > 0);

  const syncSample = generateWorkspaceCloud(PRESETS.ur5.joints, 500, null);
  assert.ok(Math.abs(cloud.bounds.maxX - syncSample.bounds.maxX) < 3.0);
});

test('workspaceClient computes asynchronous singularity reports matching direct analysis for UR5 and SCARA', async () => {
  for (const id of ['ur5', 'scara']) {
    const joints = PRESETS[id].joints;
    const endPos = forwardKinematics(joints).positions.at(-1);
    const asyncReport = await runSingularityAnalysisAsync(joints, { referencePos: endPos, mode: 'position' });
    const syncReport = analyzeRobotSingularities(joints, { referencePos: endPos, mode: 'position' });

    assert.equal(asyncReport.axes.length, syncReport.axes.length);
    assert.equal(asyncReport.currentDexterityPct, syncReport.currentDexterityPct);
    assert.equal(asyncReport.jointSweeps.length, syncReport.jointSweeps.length);
    assert.equal(asyncReport.criticalPoses.length, syncReport.criticalPoses.length);
    assert.deepEqual(asyncReport.criticalPoses, syncReport.criticalPoses);
  }
});

test('terminateWorkspaceWorker handles idle teardown safely without errors', () => {
  assert.doesNotThrow(() => terminateWorkspaceWorker());
});
