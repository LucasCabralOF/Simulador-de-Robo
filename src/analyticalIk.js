import { Euler, Matrix4 } from 'three';

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const GEOM_TOL = 1e-6;
const ANGLE_TOL = 1e-2;

export const ANALYTICAL_ARCH_LABELS = {
  scara: 'SCARA (RRPR)',
  puma560: 'PUMA 560 (6R)',
  ur5: 'UR5 (6R)',
};

export const POSTURE_AXES_BY_ARCH = {
  scara: [
    {
      key: 'elbow',
      label: 'Cotovelo',
      options: [
        { value: 'left', label: 'Esquerda (+q₂)' },
        { value: 'right', label: 'Direita (-q₂)' },
      ],
    },
  ],
  puma560: [
    {
      key: 'shoulder',
      label: 'Ombro',
      options: [
        { value: 'right', label: 'Direito' },
        { value: 'left', label: 'Esquerdo' },
      ],
    },
    {
      key: 'elbow',
      label: 'Cotovelo',
      options: [
        { value: 'up', label: 'Cima' },
        { value: 'down', label: 'Baixo' },
      ],
    },
    {
      key: 'wrist',
      label: 'Punho',
      options: [
        { value: 'up', label: 'Normal (+q₅)' },
        { value: 'down', label: 'Invertido (-q₅)' },
      ],
    },
  ],
  ur5: [
    {
      key: 'shoulder',
      label: 'Ombro',
      options: [
        { value: 'right', label: 'Direito' },
        { value: 'left', label: 'Esquerdo' },
      ],
    },
    {
      key: 'elbow',
      label: 'Cotovelo',
      options: [
        { value: 'up', label: 'Cima' },
        { value: 'down', label: 'Baixo' },
      ],
    },
    {
      key: 'wrist',
      label: 'Punho',
      options: [
        { value: 'up', label: 'Normal (+q₅)' },
        { value: 'down', label: 'Invertido (-q₅)' },
      ],
    },
  ],
};

function wrap180(deg) {
  let a = ((deg + 180) % 360 + 360) % 360 - 180;
  if (a <= -180) a += 360;
  return a;
}

function angleClose(a, b, tol = ANGLE_TOL) {
  return Math.abs(wrap180(a - b)) <= tol;
}

function dhTransform(a, alphaDeg, d, thetaDeg) {
  const th = thetaDeg * DEG;
  const al = alphaDeg * DEG;
  const c = Math.cos(th);
  const s = Math.sin(th);
  const ca = Math.cos(al);
  const sa = Math.sin(al);
  return new Matrix4().set(
    c, -s * ca, s * sa, a * c,
    s, c * ca, -c * sa, a * s,
    0, sa, ca, d,
    0, 0, 0, 1
  );
}

export function targetPoseMatrix(target) {
  const rpy = target?.rpy || [0, 0, 0];
  const pos = target?.position || [0, 0, 0];
  const m = new Matrix4().makeRotationFromEuler(
    new Euler((rpy[0] || 0) * DEG, (rpy[1] || 0) * DEG, (rpy[2] || 0) * DEG, 'ZYX')
  );
  m.setPosition(pos[0] || 0, pos[1] || 0, pos[2] || 0);
  return m;
}

function nearestEquivalentInRange(angleDeg, refDeg, minDeg, maxDeg) {
  const base = wrap180(angleDeg);
  let best = null;
  let bestDist = Infinity;
  for (let k = -2; k <= 2; k++) {
    const raw = base + k * 360;
    if (raw < minDeg - 1e-5 || raw > maxDeg + 1e-5) continue;
    const clamped = Math.max(minDeg, Math.min(maxDeg, raw));
    const dist = Math.abs(clamped - refDeg);
    if (dist < bestDist) {
      bestDist = dist;
      best = clamped;
    }
  }
  return best;
}

function clampPrismaticInRange(val, minVal, maxVal) {
  if (!Number.isFinite(val)) return null;
  if (val < minVal - 1e-5 || val > maxVal + 1e-5) return null;
  return Math.max(minVal, Math.min(maxVal, val));
}

function buildJointsWithValues(joints, values) {
  return joints.map((j, i) => ({
    ...j,
    val: Math.max(j.min, Math.min(j.max, values[i])),
    speed: 0,
  }));
}

export function formatPostureLabel(arch, posture) {
  if (!arch || !posture) return '';
  const axes = POSTURE_AXES_BY_ARCH[arch];
  if (!axes) return '';
  return axes
    .map(axis => {
      const opt = axis.options.find(o => o.value === posture[axis.key]);
      if (!opt) return null;
      return `${axis.label}: ${opt.label.split(' ')[0]}`;
    })
    .filter(Boolean)
    .join(' · ');
}

export function postureId(arch, posture) {
  const axes = POSTURE_AXES_BY_ARCH[arch] || [];
  return axes.map(a => `${a.key}:${posture?.[a.key] || '?'}`).join('|');
}

/**
 * Detects whether DH parameters match a supported closed-form robot family.
 * @param {import('./types.js').Joint[]} joints
 * @returns {'scara' | 'puma560' | 'ur5' | null}
 */
export function detectAnalyticalArchitecture(joints) {
  if (!Array.isArray(joints)) return null;

  if (joints.length === 4) {
    const [j0, j1, j2, j3] = joints;
    if (
      j0.type === 'R' && j1.type === 'R' && j2.type === 'P' && j3.type === 'R'
      && Math.abs(j0.a) > 1e-5 && Math.abs(j1.a) > 1e-5
      && angleClose(j0.alpha, 0)
      && (angleClose(j1.alpha, 0) || angleClose(j1.alpha, 180))
      && Math.abs(j2.a) < 1e-4 && angleClose(j2.alpha, 0)
      && Math.abs(j3.a) < 1e-4 && angleClose(j3.alpha, 0)
    ) {
      return 'scara';
    }
    return null;
  }

  if (joints.length === 6 && joints.every(j => j.type === 'R')) {
    const [j0, j1, j2, j3, j4, j5] = joints;
    const baseAndWristOk = (
      Math.abs(j0.a) < 1e-4 && angleClose(j0.alpha, 90)
      && Math.abs(j1.a) > 1e-5 && angleClose(j1.alpha, 0)
      && Math.abs(j3.a) < 1e-4 && angleClose(j3.alpha, 90)
      && Math.abs(j4.a) < 1e-4 && angleClose(j4.alpha, -90)
      && Math.abs(j5.a) < 1e-4 && angleClose(j5.alpha, 0)
    );
    if (!baseAndWristOk) return null;

    if (angleClose(j2.alpha, -90)) {
      return 'puma560';
    }
    if (Math.abs(j2.a) > 1e-5 && angleClose(j2.alpha, 0)) {
      return 'ur5';
    }
  }

  return null;
}

/**
 * Classifies the physical posture branch (shoulder, elbow, wrist) of a joint configuration.
 * @param {import('./types.js').Joint[]} joints
 * @param {'scara' | 'puma560' | 'ur5' | null} [arch]
 * @returns {{ shoulder?: string, elbow?: string, wrist?: string } | null}
 */
export function classifyPosture(joints, arch = detectAnalyticalArchitecture(joints)) {
  if (!arch || !Array.isArray(joints)) return null;

  if (arch === 'scara') {
    if (joints.length < 4) return null;
    const [j0, j1] = joints;
    const cross = j0.a * j1.a * Math.sin(j1.val * DEG);
    return { elbow: cross >= -1e-9 ? 'left' : 'right' };
  }

  if (arch === 'puma560') {
    if (joints.length < 6) return null;
    const [j0, j1, j2, j3, j4] = joints;
    const t01 = dhTransform(j0.a, j0.alpha, j0.d, j0.val);
    const t12 = dhTransform(j1.a, j1.alpha, j1.d, j1.val);
    const t23 = dhTransform(j2.a, j2.alpha, j2.d, j2.val);
    const t34 = dhTransform(j3.a, j3.alpha, j3.d, j3.val);
    const t02 = new Matrix4().multiplyMatrices(t01, t12);
    const t04 = new Matrix4().multiplyMatrices(t01, new Matrix4().multiplyMatrices(t12, new Matrix4().multiplyMatrices(t23, t34)));
    const e14 = t01.elements;
    const e2 = t02.elements;
    const e4 = t04.elements;
    // Frame 1 X-axis (c1, s1, 0)
    const c1 = e14[0], s1 = e14[1];
    const u = c1 * e4[12] + s1 * e4[13];
    const shoulder = u >= -1e-9 ? 'right' : 'left';

    // Frame 1 coordinates of elbow O2 and wrist center O4
    const xe = c1 * e2[12] + s1 * e2[13];
    const ye = e2[14] - j0.d;
    const px = c1 * e4[12] + s1 * e4[13];
    const py = e4[14] - j0.d;
    const sign = Math.abs(px) > 1e-5 ? Math.sign(px) : 1;
    const chordLift = (px * ye - py * xe) * sign;
    const elbow = chordLift >= -1e-9 ? 'up' : 'down';

    const wrist = Math.sin(j4.val * DEG) >= -1e-9 ? 'up' : 'down';
    return { shoulder, elbow, wrist };
  }

  if (arch === 'ur5') {
    if (joints.length < 6) return null;
    const [j0, j1, j2, j3, j4] = joints;
    const t01 = dhTransform(j0.a, j0.alpha, j0.d, j0.val);
    const t12 = dhTransform(j1.a, j1.alpha, j1.d, j1.val);
    const t23 = dhTransform(j2.a, j2.alpha, j2.d, j2.val);
    const t34 = dhTransform(j3.a, j3.alpha, j3.d, j3.val);
    const t45 = dhTransform(j4.a, j4.alpha, j4.d, j4.val);
    const t02 = new Matrix4().multiplyMatrices(t01, t12);
    const t03 = new Matrix4().multiplyMatrices(t02, t23);
    const t04 = new Matrix4().multiplyMatrices(t03, t34);
    const t05 = new Matrix4().multiplyMatrices(t04, t45);
    const e1 = t01.elements;
    const e2 = t02.elements;
    const e3 = t03.elements;
    const e5 = t05.elements;
    const c1 = e1[0], s1 = e1[1];
    const u = c1 * e5[12] + s1 * e5[13];
    const shoulder = u >= -1e-9 ? 'right' : 'left';

    const xe = c1 * e2[12] + s1 * e2[13];
    const ye = e2[14] - j0.d;
    const px = c1 * e3[12] + s1 * e3[13];
    const py = e3[14] - j0.d;
    const sign = Math.abs(px) > 1e-5 ? Math.sign(px) : 1;
    const chordLift = (px * ye - py * xe) * sign;
    const elbow = chordLift >= -1e-9 ? 'up' : 'down';

    const wrist = Math.sin(j4.val * DEG) >= -1e-9 ? 'up' : 'down';
    return { shoulder, elbow, wrist };
  }

  return null;
}

function solveScaraRaw(joints, target, mode) {
  const [j0, j1, j2, j3] = joints;
  const a1 = j0.a;
  const a2 = j1.a;
  const d1 = j0.d;
  const d2 = j1.d;
  const sz = Math.cos(j1.alpha * DEG) >= 0 ? 1 : -1;
  const theta3 = j2.theta;
  const d4 = j3.d;

  const pos = target.position || [0, 0, 0];
  const x = pos[0], y = pos[1], z = pos[2];

  let d3Val = j2.val;
  if (mode !== 'xy') {
    const rawD3 = (z - (d1 + d2)) / sz - d4;
    d3Val = clampPrismaticInRange(rawD3, j2.min, j2.max);
    if (d3Val === null) return [];
  } else {
    d3Val = clampPrismaticInRange(j2.val, j2.min, j2.max);
    if (d3Val === null) return [];
  }

  let yawDeg = null;
  if (mode === 'pose') {
    const tm = targetPoseMatrix(target).elements;
    // Column 2 = target Z axis; must equal [0, 0, sz]
    const zx = tm[8], zy = tm[9], zz = tm[10];
    if (Math.hypot(zx, zy, zz - sz) > 5e-3) {
      return [];
    }
    yawDeg = Math.atan2(tm[1], tm[0]) * RAD;
  }

  const r2 = x * x + y * y;
  const cos2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);
  if (Math.abs(cos2) > 1 + 1e-5) return [];
  const c2Clamped = Math.max(-1, Math.min(1, cos2));
  const q2Mag = Math.acos(c2Clamped) * RAD;
  const q2Options = q2Mag < 1e-4 ? [0] : [q2Mag, -q2Mag];

  const out = [];
  for (const q2Raw of q2Options) {
    const q2 = nearestEquivalentInRange(q2Raw, j1.val, j1.min, j1.max);
    if (q2 === null) continue;

    const A = a1 + a2 * Math.cos(q2 * DEG);
    const B = a2 * Math.sin(q2 * DEG);
    const q1Raw = A * A + B * B > 1e-12
      ? Math.atan2(A * y - B * x, A * x + B * y) * RAD
      : j0.val;
    const q1 = nearestEquivalentInRange(q1Raw, j0.val, j0.min, j0.max);
    if (q1 === null) continue;

    let q4 = nearestEquivalentInRange(j3.val, j3.val, j3.min, j3.max);
    if (mode === 'pose') {
      const q4Raw = sz * (yawDeg - (q1 + q2)) - theta3;
      q4 = nearestEquivalentInRange(q4Raw, j3.val, j3.min, j3.max);
    }
    if (q4 === null) continue;

    const candJoints = buildJointsWithValues(joints, [q1, q2, d3Val, q4]);
    const posture = classifyPosture(candJoints, 'scara');
    out.push({
      joints: candJoints,
      posture,
      architecture: 'scara',
      label: formatPostureLabel('scara', posture),
    });
  }
  return out;
}

function solvePuma560Raw(joints, target) {
  const [j0, j1, j2, j3, j4, j5] = joints;
  const d1 = j0.d;
  const a2 = j1.a;
  const d23 = j1.d + j2.d;
  const a3 = j2.a;
  const d4 = j3.d;
  const d6 = j5.d;

  const tm = targetPoseMatrix(target);
  const te = tm.elements;
  // Flange frame axes and origin
  const px = te[12], py = te[13], pz = te[14];
  const zx = te[8], zy = te[9], zz = te[10];

  // Wrist center Pw = P6 - d6 * z6
  const pwx = px - d6 * zx;
  const pwy = py - d6 * zy;
  const pwz = pz - d6 * zz;

  const rxy = Math.hypot(pwx, pwy);
  if (rxy < Math.abs(d23) - 1e-5) return [];
  const s1Mag = Math.sqrt(Math.max(0, rxy * rxy - d23 * d23));

  const shoulderChoices = [];
  if (rxy < 1e-6 && Math.abs(d23) < 1e-6) {
    shoulderChoices.push(j0.val);
  } else {
    const phi = Math.atan2(pwy, pwx);
    shoulderChoices.push((phi + Math.atan2(d23, s1Mag)) * RAD);
    if (s1Mag > 1e-5) {
      shoulderChoices.push((phi + Math.atan2(d23, -s1Mag)) * RAD);
    }
  }

  const rho2 = a3 * a3 + d4 * d4;
  const rho = Math.sqrt(rho2);
  const beta = Math.atan2(d4, a3);

  const out = [];
  for (const q1Raw of shoulderChoices) {
    const q1 = nearestEquivalentInRange(q1Raw, j0.val, j0.min, j0.max);
    if (q1 === null) continue;

    const c1 = Math.cos(q1 * DEG);
    const s1 = Math.sin(q1 * DEG);
    const wx = c1 * pwx + s1 * pwy;
    const wy = pwz - d1;

    const kVal = (wx * wx + wy * wy - a2 * a2 - rho2) / (2 * a2);
    if (Math.abs(kVal) > rho + 1e-5) continue;
    const s3Mag = Math.sqrt(Math.max(0, rho2 - kVal * kVal));
    const q3Options = s3Mag < 1e-5
      ? [(Math.atan2(0, kVal) - beta) * RAD]
      : [(Math.atan2(s3Mag, kVal) - beta) * RAD, (Math.atan2(-s3Mag, kVal) - beta) * RAD];

    for (const q3Raw of q3Options) {
      const q3 = nearestEquivalentInRange(q3Raw, j2.val, j2.min, j2.max);
      if (q3 === null) continue;

      const c3 = Math.cos(q3 * DEG);
      const s3 = Math.sin(q3 * DEG);
      const A = a2 + a3 * c3 - d4 * s3;
      const B = a3 * s3 + d4 * c3;
      const denom = A * A + B * B;
      const q2Raw = denom > 1e-12
        ? Math.atan2(A * wy - B * wx, A * wx + B * wy) * RAD
        : j1.val;
      const q2 = nearestEquivalentInRange(q2Raw, j1.val, j1.min, j1.max);
      if (q2 === null) continue;

      const t03 = dhTransform(j0.a, j0.alpha, j0.d, q1)
        .multiply(dhTransform(j1.a, j1.alpha, j1.d, q2))
        .multiply(dhTransform(j2.a, j2.alpha, j2.d, q3));
      const re = t03.elements;

      // R3_6 = (R0_3)^T * R_target
      const dot = (cA, cB) => re[cA * 4] * te[cB * 4] + re[cA * 4 + 1] * te[cB * 4 + 1] + re[cA * 4 + 2] * te[cB * 4 + 2];
      const r02 = dot(0, 2);
      const r12 = dot(1, 2);
      const r22 = dot(2, 2);
      const r20 = dot(2, 0);
      const r21 = dot(2, 1);
      const r00 = dot(0, 0);
      const r10 = dot(1, 0);

      const s5Mag = Math.hypot(r02, r12);
      const wristTriples = [];

      if (s5Mag > 1e-5) {
        wristTriples.push([
          Math.atan2(-r12, -r02) * RAD,
          Math.atan2(s5Mag, r22) * RAD,
          Math.atan2(-r21, r20) * RAD,
        ]);
        wristTriples.push([
          Math.atan2(r12, r02) * RAD,
          Math.atan2(-s5Mag, r22) * RAD,
          Math.atan2(r21, -r20) * RAD,
        ]);
      } else {
        // Wrist singularity: q5 = 0 or 180
        const q5Sing = r22 >= 0 ? 0 : 180;
        const combDeg = (r22 >= 0 ? Math.atan2(r10, r00) : Math.atan2(r10, -r00)) * RAD;
        for (let k = -1; k <= 1; k++) {
          const sumAngle = wrap180(combDeg) + k * 360;
          const loQ4 = r22 >= 0 ? Math.max(j3.min, sumAngle - j5.max) : Math.max(j3.min, j5.min - sumAngle);
          const hiQ4 = r22 >= 0 ? Math.min(j3.max, sumAngle - j5.min) : Math.min(j3.max, j5.max - sumAngle);
          if (loQ4 <= hiQ4 + 1e-5) {
            const q4Sing = Math.max(loQ4, Math.min(hiQ4, j3.val));
            const q6Sing = r22 >= 0 ? sumAngle - q4Sing : q4Sing + sumAngle;
            wristTriples.push([q4Sing, q5Sing, q6Sing]);
            break;
          }
        }
      }

      for (const [q4Raw, q5Raw, q6Raw] of wristTriples) {
        const q4 = nearestEquivalentInRange(q4Raw, j3.val, j3.min, j3.max);
        const q5 = nearestEquivalentInRange(q5Raw, j4.val, j4.min, j4.max);
        const q6 = nearestEquivalentInRange(q6Raw, j5.val, j5.min, j5.max);
        if (q4 === null || q5 === null || q6 === null) continue;

        const candJoints = buildJointsWithValues(joints, [q1, q2, q3, q4, q5, q6]);
        const posture = classifyPosture(candJoints, 'puma560');
        out.push({
          joints: candJoints,
          posture,
          architecture: 'puma560',
          label: formatPostureLabel('puma560', posture),
        });
      }
    }
  }
  return out;
}

function solveUr5Raw(joints, target) {
  const [j0, j1, j2, j3, j4, j5] = joints;
  const a2 = j1.a;
  const a3 = j2.a;
  const d234 = j1.d + j2.d + j3.d;
  const d6 = j5.d;

  const tm = targetPoseMatrix(target);
  const te = tm.elements;
  const xx = te[0], xy = te[1];
  const yx = te[4], yy = te[5];
  const zx = te[8], zy = te[9], zz = te[10];
  const px = te[12], py = te[13], pz = te[14];

  const p5x = px - d6 * zx;
  const p5y = py - d6 * zy;
  const p5z = pz - d6 * zz;

  const r5 = Math.hypot(p5x, p5y);
  if (r5 < Math.abs(d234) - 1e-5) return [];
  const s1Mag = Math.sqrt(Math.max(0, r5 * r5 - d234 * d234));

  const shoulderChoices = [];
  if (r5 < 1e-6 && Math.abs(d234) < 1e-6) {
    shoulderChoices.push(j0.val);
  } else {
    const phi = Math.atan2(p5y, p5x);
    shoulderChoices.push((phi + Math.atan2(d234, s1Mag)) * RAD);
    if (s1Mag > 1e-5) {
      shoulderChoices.push((phi + Math.atan2(d234, -s1Mag)) * RAD);
    }
  }

  const out = [];
  for (const q1Raw of shoulderChoices) {
    const q1 = nearestEquivalentInRange(q1Raw, j0.val, j0.min, j0.max);
    if (q1 === null) continue;

    const c1 = Math.cos(q1 * DEG);
    const s1 = Math.sin(q1 * DEG);

    const z1x = s1 * xx - c1 * xy;
    const z1y = s1 * yx - c1 * yy;
    const z1z = s1 * zx - c1 * zy;
    const s5Mag = Math.hypot(z1x, z1y);

    const wristPairs = [];
    if (s5Mag > 1e-5) {
      wristPairs.push([
        Math.atan2(s5Mag, z1z) * RAD,
        Math.atan2(-z1y, z1x) * RAD,
        false,
      ]);
      wristPairs.push([
        Math.atan2(-s5Mag, z1z) * RAD,
        Math.atan2(z1y, -z1x) * RAD,
        false,
      ]);
    } else {
      const q5Sing = z1z >= 0 ? 0 : 180;
      wristPairs.push([q5Sing, j5.val, true]);
    }

    const t1Inv = dhTransform(j0.a, j0.alpha, j0.d, q1).invert();
    const t1Target = new Matrix4().multiplyMatrices(t1Inv, tm);

    const tryWristPair = (q5Raw, q6Raw) => {
      const q5 = nearestEquivalentInRange(q5Raw, j4.val, j4.min, j4.max);
      const q6 = nearestEquivalentInRange(q6Raw, j5.val, j5.min, j5.max);
      if (q5 === null || q6 === null) return 0;

      const t56Inv = dhTransform(j4.a, j4.alpha, j4.d, q5)
        .multiply(dhTransform(j5.a, j5.alpha, j5.d, q6))
        .invert();
      const t14 = new Matrix4().multiplyMatrices(t1Target, t56Inv);
      const e14 = t14.elements;

      const x14 = e14[12];
      const y14 = e14[13];
      const th234 = Math.atan2(e14[1], e14[0]) * RAD;

      const r2 = x14 * x14 + y14 * y14;
      const cos3 = (r2 - a2 * a2 - a3 * a3) / (2 * a2 * a3);
      if (Math.abs(cos3) > 1 + 1e-5) return 0;
      const c3Clamped = Math.max(-1, Math.min(1, cos3));
      const q3Mag = Math.acos(c3Clamped) * RAD;
      const q3Options = q3Mag < 1e-4 ? [0] : [q3Mag, -q3Mag];

      let added = 0;
      for (const q3Raw of q3Options) {
        const q3 = nearestEquivalentInRange(q3Raw, j2.val, j2.min, j2.max);
        if (q3 === null) continue;

        const A = a2 + a3 * Math.cos(q3 * DEG);
        const B = a3 * Math.sin(q3 * DEG);
        const denom = A * A + B * B;
        const q2Raw = denom > 1e-12
          ? Math.atan2(A * y14 - B * x14, A * x14 + B * y14) * RAD
          : j1.val;
        const q2 = nearestEquivalentInRange(q2Raw, j1.val, j1.min, j1.max);
        if (q2 === null) continue;

        const q4Raw = th234 - q2 - q3;
        const q4 = nearestEquivalentInRange(q4Raw, j3.val, j3.min, j3.max);
        if (q4 === null) continue;

        const candJoints = buildJointsWithValues(joints, [q1, q2, q3, q4, q5, q6]);
        const posture = classifyPosture(candJoints, 'ur5');
        out.push({
          joints: candJoints,
          posture,
          architecture: 'ur5',
          label: formatPostureLabel('ur5', posture),
        });
        added++;
      }
      return added;
    };

    for (const [q5Raw, q6Raw, isSingular] of wristPairs) {
      let count = tryWristPair(q5Raw, q6Raw);
      if (isSingular && count === 0) {
        for (const fallbackQ6 of [-135, -90, -45, 0, 45, 90, 135, 180]) {
          count = tryWristPair(q5Raw, fallbackQ6);
          if (count > 0) break;
        }
      }
    }
  }

  // Keep reference to p5z to satisfy static checks if unused
  void p5z;
  return out;
}

function deduplicateBranches(candidates, arch) {
  const seen = [];
  for (const cand of candidates) {
    const id = postureId(arch, cand.posture);
    const samePose = seen.some(existing => (
      existing.id === id
      || existing.joints.every((j, idx) => Math.abs(j.val - cand.joints[idx].val) < 1e-3)
    ));
    if (!samePose) {
      seen.push({ ...cand, id });
    }
  }
  return seen;
}

/**
 * Generates raw analytical IK branch candidates within [min, max] angular limits.
 * @param {import('./types.js').Joint[]} joints
 * @param {import('./types.js').CartesianTarget} target
 * @param {import('./types.js').TaskMode} [mode]
 * @returns {Array<{ id: string, joints: import('./types.js').Joint[], posture: Record<string, string> | null, architecture: 'scara' | 'puma560' | 'ur5', label: string }>}
 */
export function generateAnalyticalCandidates(joints, target, mode = 'pose') {
  const arch = detectAnalyticalArchitecture(joints);
  if (!arch || !target || !Array.isArray(target.position)) return [];

  let raw = [];
  if (arch === 'scara') {
    raw = solveScaraRaw(joints, target, mode);
  } else if (arch === 'puma560') {
    if (mode !== 'pose') return [];
    raw = solvePuma560Raw(joints, target);
  } else if (arch === 'ur5') {
    if (mode !== 'pose') return [];
    raw = solveUr5Raw(joints, target);
  }

  return deduplicateBranches(raw, arch);
}

export function matchesPosturePreference(posture, preference) {
  if (!preference || preference === 'auto' || !posture) return true;
  return Object.entries(preference).every(([key, val]) => !val || val === 'auto' || posture[key] === val);
}

export function jointDisplacementScore(aJoints, bJoints, lengthRef = 5) {
  let sum = 0;
  for (let i = 0; i < aJoints.length; i++) {
    const d = Math.abs(aJoints[i].val - bJoints[i].val);
    sum += aJoints[i].type === 'R' ? (d * DEG) : (d / Math.max(lengthRef, 1e-4));
  }
  return sum;
}

export function maxRevoluteJumpDeg(aJoints, bJoints) {
  let maxJump = 0;
  for (let i = 0; i < aJoints.length; i++) {
    if (aJoints[i].type === 'R') {
      maxJump = Math.max(maxJump, Math.abs(aJoints[i].val - bJoints[i].val));
    }
  }
  return maxJump;
}

export { GEOM_TOL };
