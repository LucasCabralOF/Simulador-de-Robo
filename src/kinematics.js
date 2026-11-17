import { Euler, Matrix4, Quaternion, Vector3 } from 'three';
import { Matrix, SingularValueDecomposition, determinant } from 'ml-matrix';

export const DEG = Math.PI / 180;
export const DEFAULT_JOINTS = [
  { id: 1, type: 'R', a: 0, alpha: 90, d: 0, theta: 0, val: 45, min: -180, max: 180, speed: 0 },
  { id: 2, type: 'P', a: 0, alpha: -90, d: 0, theta: 0, val: 5, min: 0, max: 10, speed: 0 },
  { id: 3, type: 'R', a: 5, alpha: 0, d: 0, theta: 0, val: 30, min: -180, max: 180, speed: 0 },
];
export const TASK_ROWS = { xy: [0, 1], position: [0, 1, 2], pose: [0, 1, 2, 3, 4, 5] };
export const TOOL_TYPES = {
  gripper: {
    id: 'gripper',
    name: 'Garra Paralela (2 Dedos)',
    shortLabel: 'Garra Paralela',
    length: 0.28,
    radius: 0.16,
    actuationLabel: 'Abertura da garra (%)',
    description: 'Pinça paralela de 2 dedos para manipulação geral, montagem e pick-and-place.',
  },
  vacuum: {
    id: 'vacuum',
    name: 'Ventosa Pneumática (Vácuo)',
    shortLabel: 'Ventosa a Vácuo',
    length: 0.32,
    radius: 0.18,
    actuationLabel: 'Sucção / vácuo (%)',
    description: 'Copo pneumático a vácuo ideal para paletização, chapas e embalagens planas.',
  },
  welder: {
    id: 'welder',
    name: 'Tocha de Solda / Laser',
    shortLabel: 'Tocha de Solda',
    length: 0.38,
    radius: 0.14,
    actuationLabel: 'Potência do arco / laser (%)',
    description: 'Ferramenta direcional de precisão onde o vetor de orientação define o ângulo de ataque.',
  },
};
export const TOOL_LIST = Object.values(TOOL_TYPES);
export const DEFAULT_ENVIRONMENT = {
  floorEnabled: true,
  floorZ: -0.22,
  obstacle: {
    enabled: false,
    shape: 'sphere',
    position: [3.5, -2.5, 1.0],
    radius: 0.8,
  },
};
const norm = (v) => Math.hypot(...v);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export function validateJoints(joints) {
  if (!Array.isArray(joints) || joints.length < 1 || joints.length > 24) {
    return ['O modelo deve conter de 1 a 24 juntas.'];
  }
  const errors = [];
  const ids = new Set();
  joints.forEach((joint, index) => {
    const label = `Junta ${index + 1}`;
    if (!joint || typeof joint !== 'object') { errors.push(`${label}: dados inválidos.`); return; }
    if (!Number.isInteger(joint.id) || joint.id < 1 || ids.has(joint.id)) errors.push(`${label}: identificador inválido ou duplicado.`);
    ids.add(joint.id);
    if (!['R', 'P'].includes(joint.type)) errors.push(`${label}: tipo deve ser R ou P.`);
    for (const key of ['a', 'alpha', 'd', 'theta', 'val', 'min', 'max', 'speed']) {
      if (key === 'speed' && joint[key] === undefined) continue;
      if (!Number.isFinite(joint[key]) || Math.abs(joint[key]) > 1e6) errors.push(`${label}: ${key} deve ser finito, entre -10⁶ e 10⁶.`);
    }
    if (joint.min >= joint.max) errors.push(`${label}: mínimo deve ser menor que máximo.`);
    if (joint.val < joint.min || joint.val > joint.max) errors.push(`${label}: posição fora dos limites.`);
  });
  return errors;
}

export function normalizeJoints(joints) {
  const errors = validateJoints(joints);
  if (errors.length) throw new Error(errors.join(' '));
  return joints.map(({ id, type, a, alpha, d, theta, val, min, max, speed = 0 }) =>
    ({ id, type, a, alpha, d, theta, val, min, max, speed }));
}

export function matrixRows(matrix, size = 4) {
  return Array.from({ length: size }, (_, row) =>
    Array.from({ length: size }, (_, col) => matrix.elements[col * 4 + row]));
}

export function dhMatrix(joint) {
  const theta = (joint.type === 'R' ? joint.val : joint.theta) * DEG;
  const alpha = joint.alpha * DEG;
  const d = joint.type === 'P' ? joint.val : joint.d;
  const c = Math.cos(theta), s = Math.sin(theta), ca = Math.cos(alpha), sa = Math.sin(alpha);
  return new Matrix4().set(
    c, -s * ca, s * sa, joint.a * c,
    s, c * ca, -c * sa, joint.a * s,
    0, sa, ca, d,
    0, 0, 0, 1,
  );
}

export function rotationFromRPY(rpy) {
  return new Matrix4().makeRotationFromEuler(new Euler(...rpy.map(v => v * DEG), 'ZYX'));
}

function rotationVector(quaternion) {
  const q = quaternion.clone().normalize();
  if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
  const sine = Math.hypot(q.x, q.y, q.z);
  if (sine < 1e-12) return [0, 0, 0];
  const scale = 2 * Math.atan2(sine, clamp(q.w, -1, 1)) / sine;
  return [q.x * scale, q.y * scale, q.z * scale];
}

export function orientation(matrix) {
  const euler = new Euler().setFromRotationMatrix(matrix, 'ZYX');
  const vector = rotationVector(new Quaternion().setFromRotationMatrix(matrix));
  const angle = norm(vector);
  const rotation = matrixRows(matrix, 3);
  const r = new Matrix(rotation);
  const orthogonality = r.transpose().mmul(r).sub(Matrix.eye(3)).norm();
  return {
    rotation, rpy: [euler.x, euler.y, euler.z].map(v => v / DEG),
    gimbalLock: Math.abs(Math.cos(euler.y)) < 1e-6,
    axis: angle < 1e-12 ? null : vector.map(v => v / angle), angle: angle / DEG,
    determinant: determinant(r), orthogonality,
  };
}

export function computeToolFrame(fkTotal, toolOffsetRpy = [0, 0, 0], toolType = 'gripper') {
  const spec = TOOL_TYPES[toolType] || TOOL_TYPES.gripper;
  const safeOffset = Array.isArray(toolOffsetRpy) && toolOffsetRpy.length === 3
    ? toolOffsetRpy.map(v => (Number.isFinite(v) ? v : 0))
    : [0, 0, 0];
  const offsetMatrix = rotationFromRPY(safeOffset);
  const toolMatrix = fkTotal.clone().multiply(offsetMatrix);
  const tcpVec = new Vector3(spec.length, 0, 0).applyMatrix4(toolMatrix);
  const approachVec = new Vector3().setFromMatrixColumn(toolMatrix, 0).normalize();
  return {
    tool: spec,
    matrix: toolMatrix,
    tcpPosition: [tcpVec.x, tcpVec.y, tcpVec.z],
    approach: [approachVec.x, approachVec.y, approachVec.z],
    orientation: orientation(toolMatrix),
  };
}

export function flangeRpyFromToolTarget(targetToolRpy, toolOffsetRpy = [0, 0, 0]) {
  const safeOffset = Array.isArray(toolOffsetRpy) && toolOffsetRpy.length === 3
    ? toolOffsetRpy.map(v => (Number.isFinite(v) ? v : 0))
    : [0, 0, 0];
  if (safeOffset.every(v => Math.abs(v) < 1e-9)) return [...targetToolRpy];
  const flangeMatrix = rotationFromRPY(targetToolRpy).multiply(rotationFromRPY(safeOffset).transpose());
  return orientation(flangeMatrix).rpy;
}

export function forwardKinematics(joints, { includeOrientation = true } = {}) {
  const total = new Matrix4();
  const transforms = [total.clone()];
  const links = [];
  for (const joint of joints) {
    const link = dhMatrix(joint);
    links.push(link);
    total.multiply(link);
    transforms.push(total.clone());
  }
  const positions = transforms.map(m => new Vector3().setFromMatrixPosition(m));
  const end = positions.at(-1);
  const jacobian = Array.from({ length: 6 }, () => []);
  joints.forEach((joint, index) => {
    const axis = new Vector3().setFromMatrixColumn(transforms[index], 2);
    const linear = joint.type === 'P' ? axis : axis.clone().cross(end.clone().sub(positions[index]));
    const angular = joint.type === 'P' ? new Vector3() : axis;
    [...linear.toArray(), ...angular.toArray()].forEach((value, row) => jacobian[row].push(value));
  });
  return {
    total,
    transforms,
    links,
    positions: positions.map(p => p.toArray()),
    jacobian,
    orientation: includeOrientation ? orientation(total) : null,
  };
}

export function multiplyVector(matrix, vector) {
  return matrix.map(row => row.reduce((sum, value, i) => sum + value * vector[i], 0));
}

const dist3 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const JACKKNIFE_COS_THRESHOLD = Math.cos(8 * DEG);

export function pointSegmentDistance(p, a, b) {
  const ab = sub3(b, a);
  const ap = sub3(p, a);
  const abLen2 = dot3(ab, ab);
  const t = abLen2 < 1e-12 ? 0 : clamp(dot3(ap, ab) / abLen2, 0, 1);
  const closest = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
  return { distance: dist3(p, closest), closest };
}

export function pointAabbDistance(point, center, halfExtent) {
  const closest = [
    clamp(point[0], center[0] - halfExtent, center[0] + halfExtent),
    clamp(point[1], center[1] - halfExtent, center[1] + halfExtent),
    clamp(point[2], center[2] - halfExtent, center[2] + halfExtent),
  ];
  return { distance: dist3(point, closest), closest };
}

export function segmentAabbClosest(a, b, center, halfExtent) {
  const evalAt = (t) => {
    const pt = [
      a[0] + t * (b[0] - a[0]),
      a[1] + t * (b[1] - a[1]),
      a[2] + t * (b[2] - a[2]),
    ];
    const res = pointAabbDistance(pt, center, halfExtent);
    return { t, pt, distance: res.distance, closest: res.closest };
  };

  const len = dist3(a, b);
  const samples = len < 1e-9 ? 1 : Math.max(5, Math.min(48, Math.ceil(len / Math.max(0.1, halfExtent * 0.35))));
  let best = evalAt(0);
  for (let i = 1; i <= samples; i++) {
    const cand = evalAt(i / samples);
    if (cand.distance < best.distance) best = cand;
  }

  if (samples > 1) {
    let lo = Math.max(0, best.t - 1 / samples);
    let hi = Math.min(1, best.t + 1 / samples);
    for (let iter = 0; iter < 5; iter++) {
      const m1 = lo + (hi - lo) / 3;
      const m2 = hi - (hi - lo) / 3;
      const c1 = evalAt(m1);
      const c2 = evalAt(m2);
      if (c1.distance < best.distance) best = c1;
      if (c2.distance < best.distance) best = c2;
      if (c1.distance <= c2.distance) hi = m2;
      else lo = m1;
    }
  }

  return {
    distance: best.distance,
    closestOnBox: best.closest,
    closestOnSeg: best.pt,
  };
}

export function segmentSegmentDistance(p1, q1, p2, q2) {
  const d1 = sub3(q1, p1);
  const d2 = sub3(q2, p2);
  const r = sub3(p1, p2);
  const a = dot3(d1, d1);
  const e = dot3(d2, d2);
  const f = dot3(d2, r);
  let s = 0;
  let t = 0;

  if (a <= 1e-12 && e <= 1e-12) {
    s = 0;
    t = 0;
  } else if (a <= 1e-12) {
    s = 0;
    t = clamp(f / e, 0, 1);
  } else {
    const c = dot3(d1, r);
    if (e <= 1e-12) {
      t = 0;
      s = clamp(-c / a, 0, 1);
    } else {
      const b = dot3(d1, d2);
      const denom = a * e - b * b;
      s = denom !== 0 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp(-c / a, 0, 1);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c) / a, 0, 1);
      }
    }
  }

  const closestA = [p1[0] + s * d1[0], p1[1] + s * d1[1], p1[2] + s * d1[2]];
  const closestB = [p2[0] + t * d2[0], p2[1] + t * d2[1], p2[2] + t * d2[2]];
  return {
    distance: dist3(closestA, closestB),
    closestA,
    closestB,
    midpoint: [
      (closestA[0] + closestB[0]) / 2,
      (closestA[1] + closestB[1]) / 2,
      (closestA[2] + closestB[2]) / 2,
    ],
  };
}

export function extractRobotGeometry(joints, fk = forwardKinematics(joints), toolType = 'gripper') {
  const origins = fk.positions;
  const segments = [];
  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    const d = j.type === 'P' ? j.val : j.d;
    const oCurr = origins[i];
    const oNext = origins[i + 1];
    const axialEndVec = new Vector3(0, 0, d).applyMatrix4(fk.transforms[i]);
    const axialEnd = [axialEndVec.x, axialEndVec.y, axialEndVec.z];
    if (dist3(oCurr, axialEnd) > 0.08) {
      segments.push({ a: oCurr, b: axialEnd, jointIndex: i, kind: 'axial', label: `Elo ${i + 1}` });
    }
    if (Math.abs(j.a) > 0.08 && dist3(axialEnd, oNext) > 0.08) {
      segments.push({ a: axialEnd, b: oNext, jointIndex: i, kind: 'radial', label: `Elo ${i + 1}` });
    }
  }
  let gripTip = null;
  if (joints.length > 0) {
    const n = joints.length;
    const endOrigin = origins[n];
    const toolSpec = TOOL_TYPES[toolType] || TOOL_TYPES.gripper;
    const tipVec = new Vector3(toolSpec.length, 0, 0).applyMatrix4(fk.transforms[n]);
    gripTip = [tipVec.x, tipVec.y, tipVec.z];
    if (dist3(endOrigin, gripTip) > 0.05) {
      segments.push({ a: endOrigin, b: gripTip, jointIndex: n, kind: 'gripper', label: toolSpec.shortLabel });
    }
  }
  return { origins, segments, gripTip };
}

export function detectSelfCollision(joints, precomputedFk = null, toolType = 'gripper', environment = null) {
  if (!Array.isArray(joints) || joints.length === 0) {
    return { colliding: false, pairs: [], point: null, message: '' };
  }
  const fk = precomputedFk || forwardKinematics(joints);
  const { origins, segments, gripTip } = extractRobotGeometry(joints, fk, toolType);
  const n = joints.length;

  // Rules A & B: Segment-segment jackknife folding and 3D crossing
  for (let u = 0; u < segments.length; u++) {
    const sa = segments[u];
    for (let v = u + 1; v < segments.length; v++) {
      const sb = segments[v];
      if (sa.jointIndex === sb.jointIndex) continue;

      const endpoints = [
        [sa.a, sa.b, sb.a, sb.b],
        [sa.a, sa.b, sb.b, sb.a],
        [sa.b, sa.a, sb.a, sb.b],
        [sa.b, sa.a, sb.b, sb.a],
      ];
      let shared = null;
      let minEndDist = Infinity;
      for (const [ea, otherA, eb, otherB] of endpoints) {
        const dEnd = dist3(ea, eb);
        if (dEnd < minEndDist) minEndDist = dEnd;
        if (dEnd <= 0.15 && !shared) shared = [ea, otherA, eb, otherB];
      }

      if (shared) {
        if ((sa.kind === 'gripper' && sb.jointIndex === n - 1) || (sb.kind === 'gripper' && sa.jointIndex === n - 1)) {
          continue;
        }
        const [ea, otherA, eb, otherB] = shared;
        const lenA = dist3(ea, otherA);
        const lenB = dist3(eb, otherB);
        if (lenA > 0.25 && lenB > 0.25) {
          const dirA = sub3(otherA, ea);
          const dirB = sub3(otherB, eb);
          const cosPhi = dot3(dirA, dirB) / (lenA * lenB);
          if (cosPhi > JACKKNIFE_COS_THRESHOLD) {
            const bisector = [dirA[0] / lenA + dirB[0] / lenB, dirA[1] / lenA + dirB[1] / lenB, dirA[2] / lenA + dirB[2] / lenB];
            const bNorm = Math.hypot(...bisector) || 1;
            const offset = Math.min(lenA, lenB, 0.8) * 0.5;
            const point = [
              ea[0] + (bisector[0] / bNorm) * offset,
              ea[1] + (bisector[1] / bNorm) * offset,
              ea[2] + (bisector[2] / bNorm) * offset,
            ];
            return {
              colliding: true,
              pairs: [[sa.label, sb.label]],
              point,
              message: `Autocolisão detectada: ${sa.label} dobra sobre ${sb.label}.`,
            };
          }
        }
        continue;
      }

      const threshold = sa.kind === 'gripper' || sb.kind === 'gripper' ? 0.18 : 0.16;
      const segDist = segmentSegmentDistance(sa.a, sa.b, sb.a, sb.b);
      if (segDist.distance < threshold) {
        return {
          colliding: true,
          pairs: [[sa.label, sb.label]],
          point: segDist.midpoint,
          message: `Autocolisão detectada entre ${sa.label} e ${sb.label}.`,
        };
      }
    }
  }

  // Rule C: Link passing through a non-adjacent joint motor or Base
  for (let k = 0; k < n; k++) {
    const ok = origins[k];
    const nodeLabel = k === 0 ? 'Base' : `Junta ${k + 1}`;
    for (const s of segments) {
      if (s.jointIndex === k) continue;
      if (dist3(ok, s.a) <= 0.36 || dist3(ok, s.b) <= 0.36) continue;
      const { distance, closest } = pointSegmentDistance(ok, s.a, s.b);
      if (distance < 0.28) {
        return {
          colliding: true,
          pairs: [[s.label, nodeLabel]],
          point: closest,
          message: `Autocolisão detectada: ${s.label} colide com ${nodeLabel}.`,
        };
      }
    }
  }

  // Rule D: Distal joint/gripper colliding with proximal joint motor or Base
  const cumulativeSpan = [0];
  for (let i = 0; i < n; i++) {
    const dAbs = Math.abs(joints[i].type === 'P' ? joints[i].val : joints[i].d);
    const aAbs = Math.abs(joints[i].a);
    cumulativeSpan.push(cumulativeSpan[i] + dAbs + aAbs);
  }
  for (let a = 0; a < n; a++) {
    const aLabel = a === 0 ? 'Base' : `Junta ${a + 1}`;
    for (let b = a + 2; b <= n; b++) {
      if (cumulativeSpan[b] - cumulativeSpan[a] <= 0.65) continue;
      const bLabel = b === n ? 'Garra' : `Junta ${b + 1}`;
      if (dist3(origins[a], origins[b]) < 0.34) {
        return {
          colliding: true,
          pairs: [[bLabel, aLabel]],
          point: [
            (origins[a][0] + origins[b][0]) / 2,
            (origins[a][1] + origins[b][1]) / 2,
            (origins[a][2] + origins[b][2]) / 2,
          ],
          message: `Autocolisão detectada: ${bLabel} colide com ${aLabel}.`,
        };
      }
    }
    if (gripTip && cumulativeSpan[n] - cumulativeSpan[a] + 0.28 > 0.8 && n - a >= 2) {
      if (dist3(origins[a], gripTip) < 0.34) {
        return {
          colliding: true,
          pairs: [['Garra', aLabel]],
          point: [
            (origins[a][0] + gripTip[0]) / 2,
            (origins[a][1] + gripTip[1]) / 2,
            (origins[a][2] + gripTip[2]) / 2,
          ],
          message: `Autocolisão detectada: Garra colide com ${aLabel}.`,
        };
      }
    }
  }

  // Rule E: Floor / Workbench plane collision (Z = floorZ)
  if (environment?.floorEnabled) {
    const floorZ = Number.isFinite(environment.floorZ) ? environment.floorZ : -0.22;
    const linkMargin = 0.08;
    const tipMargin = 0.05;
    for (const s of segments) {
      const checkA = s.jointIndex >= 1;
      const checkB = s.jointIndex >= 1 || dist3(s.b, origins[0]) > 0.15;
      if (checkA && s.a[2] < floorZ + linkMargin) {
        return {
          colliding: true,
          pairs: [[s.label, 'Piso / Bancada']],
          point: [s.a[0], s.a[1], floorZ],
          message: `Colisão com o ambiente: ${s.label} atingiu o piso/bancada (Z = ${floorZ.toFixed(2)} u).`,
        };
      }
      if (checkB && s.b[2] < floorZ + linkMargin) {
        return {
          colliding: true,
          pairs: [[s.label, 'Piso / Bancada']],
          point: [s.b[0], s.b[1], floorZ],
          message: `Colisão com o ambiente: ${s.label} atingiu o piso/bancada (Z = ${floorZ.toFixed(2)} u).`,
        };
      }
    }
    if (gripTip && gripTip[2] < floorZ + tipMargin) {
      return {
        colliding: true,
        pairs: [['Ferramenta', 'Piso / Bancada']],
        point: [gripTip[0], gripTip[1], floorZ],
        message: `Colisão com o ambiente: Ferramenta atingiu o piso/bancada (Z = ${floorZ.toFixed(2)} u).`,
      };
    }
  }

  // Rule F: Configurable 3D Obstacle (sphere or box)
  const obs = environment?.obstacle;
  if (obs?.enabled && Array.isArray(obs.position) && obs.position.length === 3 && obs.position.every(Number.isFinite)) {
    const center = obs.position;
    const size = Math.max(0.1, Number(obs.radius) || 0.8);
    const isBox = obs.shape === 'box';
    const toolSpec = TOOL_TYPES[toolType] || TOOL_TYPES.gripper;

    for (const s of segments) {
      const margin = s.kind === 'gripper' ? Math.max(0.08, (toolSpec.radius || 0.14) * 0.75) : 0.08;
      if (isBox) {
        const { distance, closestOnBox, closestOnSeg } = segmentAabbClosest(s.a, s.b, center, size);
        if (distance < margin) {
          const pt = distance > 1e-6
            ? [
                (closestOnBox[0] + closestOnSeg[0]) / 2,
                (closestOnBox[1] + closestOnSeg[1]) / 2,
                (closestOnBox[2] + closestOnSeg[2]) / 2,
              ]
            : closestOnBox;
          return {
            colliding: true,
            pairs: [[s.label, 'Obstáculo 3D']],
            point: pt,
            message: `Colisão com o ambiente: ${s.label} colidiu com o Obstáculo 3D.`,
          };
        }
      } else {
        const { distance, closest } = pointSegmentDistance(center, s.a, s.b);
        if (distance < size + margin) {
          let pt = closest;
          if (distance > 1e-6) {
            const dir = sub3(closest, center);
            pt = [
              center[0] + (dir[0] / distance) * size,
              center[1] + (dir[1] / distance) * size,
              center[2] + (dir[2] / distance) * size,
            ];
          }
          return {
            colliding: true,
            pairs: [[s.label, 'Obstáculo 3D']],
            point: pt,
            message: `Colisão com o ambiente: ${s.label} colidiu com o Obstáculo 3D.`,
          };
        }
      }
    }

    for (let k = 1; k <= n; k++) {
      const hub = origins[k];
      const hubLabel = k === n ? toolSpec.shortLabel : `Junta ${k + 1}`;
      const margin = 0.16;
      if (isBox) {
        const { distance, closest } = pointAabbDistance(hub, center, size);
        if (distance < margin) {
          return {
            colliding: true,
            pairs: [[hubLabel, 'Obstáculo 3D']],
            point: closest,
            message: `Colisão com o ambiente: ${hubLabel} colidiu com o Obstáculo 3D.`,
          };
        }
      } else {
        const d = dist3(hub, center);
        if (d < size + margin) {
          const dir = d > 1e-6 ? sub3(hub, center) : [0, 0, 1];
          const lenD = d > 1e-6 ? d : 1;
          return {
            colliding: true,
            pairs: [[hubLabel, 'Obstáculo 3D']],
            point: [
              center[0] + (dir[0] / lenD) * size,
              center[1] + (dir[1] / lenD) * size,
              center[2] + (dir[2] / lenD) * size,
            ],
            message: `Colisão com o ambiente: ${hubLabel} colidiu com o Obstáculo 3D.`,
          };
        }
      }
    }
  }

  return { colliding: false, pairs: [], point: null, message: '' };
}

export function sweepJointsCollisionFree(prevJoints, nextJoints, toolType = 'gripper', environment = null) {
  if (!Array.isArray(prevJoints) || !Array.isArray(nextJoints) || prevJoints.length !== nextJoints.length) {
    return { blocked: false, joints: nextJoints, collision: null };
  }
  const prevCheck = detectSelfCollision(prevJoints, null, toolType, environment);
  if (prevCheck.colliding) {
    const nextCheck = detectSelfCollision(nextJoints, null, toolType, environment);
    if (!nextCheck.colliding) {
      return { blocked: false, joints: nextJoints, collision: null };
    }
    return { blocked: true, joints: prevJoints, collision: nextCheck };
  }

  let requiredSteps = 8;
  for (let i = 0; i < prevJoints.length; i++) {
    const dq = Math.abs(nextJoints[i].val - prevJoints[i].val);
    const stepUnit = prevJoints[i].type === 'R' ? 2 : 0.1;
    requiredSteps = Math.max(requiredSteps, Math.ceil(dq / stepUnit));
  }
  const steps = Math.min(90, requiredSteps);

  const interpolate = frac => prevJoints.map((j, i) => ({
    ...nextJoints[i],
    val: j.val + (nextJoints[i].val - j.val) * frac,
  }));

  let safeFrac = 0;
  for (let k = 1; k <= steps; k++) {
    const frac = k / steps;
    const sample = interpolate(frac);
    const col = detectSelfCollision(sample, null, toolType, environment);
    if (col.colliding) {
      let lo = safeFrac;
      let hi = frac;
      let lastCollision = col;
      for (let iter = 0; iter < 8; iter++) {
        const mid = (lo + hi) / 2;
        const midSample = interpolate(mid);
        const midCol = detectSelfCollision(midSample, null, toolType, environment);
        if (midCol.colliding) {
          hi = mid;
          lastCollision = midCol;
        } else {
          lo = mid;
        }
      }
      return {
        blocked: true,
        joints: interpolate(lo),
        collision: lastCollision,
      };
    }
    safeFrac = frac;
  }

  return { blocked: false, joints: nextJoints, collision: null };
}

// Normalize linear task rows and prismatic coordinates with the same reference length.
function taskSystem(joints, jacobian, mode, length) {
  if (!TASK_ROWS[mode] || !Number.isFinite(length) || length <= 0) throw new Error('Tarefa ou comprimento de referência inválido.');
  const rows = TASK_ROWS[mode];
  const scales = joints.map(j => j.type === 'P' ? length : 1);
  const matrix = rows.map(row => jacobian[row].map((v, col) => v * scales[col] / (row < 3 ? length : 1)));
  return { matrix, rows, scales };
}

function decomposition(matrix) {
  const svd = new SingularValueDecomposition(new Matrix(matrix), { autoTranspose: true });
  const threshold = Math.max(1e-12, svd.diagonal[0] * 1e-8);
  return { svd, threshold };
}

export function jacobianDiagnostics(joints, jacobian, mode = 'position', length = 5) {
  const { matrix, rows } = taskSystem(joints, jacobian, mode, length);
  const { svd, threshold } = decomposition(matrix);
  const singularValues = svd.diagonal;
  const rank = singularValues.filter(v => v > threshold).length;
  const maxRank = Math.min(rows.length, joints.length);
  return {
    rank, maxRank, dimensions: rows.length, deficient: rank < maxRank,
    fullTask: rank === rows.length, singularValues,
    condition: rank < maxRank ? Infinity : singularValues[0] / singularValues.at(-1),
    determinant: rows.length === joints.length ? determinant(new Matrix(rows.map(row => jacobian[row]))) : null,
  };
}

export function inverseVelocity(joints, jacobian, desired, { mode = 'position', length = 5, damping = 0.02 } = {}) {
  if (!Array.isArray(desired) || desired.length !== 6 || !desired.every(Number.isFinite) || !Number.isFinite(damping) || damping < 0) {
    throw new Error('Velocidade ou amortecimento inválido.');
  }
  const { matrix, rows, scales } = taskSystem(joints, jacobian, mode, length);
  const { svd, threshold } = decomposition(matrix);
  const error = rows.map(row => desired[row] / (row < 3 ? length : 1));
  const weights = svd.diagonal.map(s => s <= threshold ? 0 : s / (s * s + damping * damping));
  const normalized = svd.rightSingularVectors.mmul(Matrix.diag(weights))
    .mmul(svd.leftSingularVectors.transpose()).mmul(Matrix.columnVector(error)).to1DArray();
  const rates = normalized.map((v, index) => v * scales[index]);
  const achieved = multiplyVector(jacobian, rates);
  const residual = desired.map((v, i) => v - achieved[i]);
  return { rates, achieved, residual, error: norm(rows.map(row => residual[row] / (row < 3 ? length : 1))) };
}

export function integrateJoints(joints, rates, dt, environment = null, toolType = 'gripper') {
  if (!Number.isFinite(dt) || dt <= 0 || rates.length !== joints.length || !rates.every(Number.isFinite)) throw new Error('Passo de integração inválido.');
  const limited = [];
  const updated = joints.map((joint, i) => {
    const next = joint.val + rates[i] * dt / (joint.type === 'R' ? DEG : 1);
    const val = clamp(next, joint.min, joint.max);
    if (val !== next) limited.push(i + 1);
    return { ...joint, val };
  });
  const swept = sweepJointsCollisionFree(joints, updated, toolType, environment);
  return { joints: swept.joints, limited, collision: swept.collision };
}

export function poseError(fk, target) {
  const translation = target.position.map((v, i) => v - fk.positions.at(-1)[i]);
  const desired = new Quaternion().setFromRotationMatrix(rotationFromRPY(target.rpy));
  const current = new Quaternion().setFromRotationMatrix(fk.total);
  return [...translation, ...rotationVector(desired.multiply(current.invert()))];
}

function solveConstrainedRates(joints, jacobian, error, options) {
  const blocked = new Array(joints.length).fill(false);
  let rates = inverseVelocity(joints, jacobian, error, options).rates;
  for (let pass = 0; pass < joints.length; pass++) {
    let addedBlock = false;
    for (let i = 0; i < joints.length; i++) {
      if (blocked[i]) continue;
      const atMin = joints[i].val <= joints[i].min + 1e-9 && rates[i] < -1e-12;
      const atMax = joints[i].val >= joints[i].max - 1e-9 && rates[i] > 1e-12;
      if (atMin || atMax) {
        blocked[i] = true;
        addedBlock = true;
      }
    }
    if (!addedBlock) break;
    if (blocked.every(Boolean)) return rates.map(() => 0);
    const maskedJacobian = jacobian.map(row => row.map((v, col) => (blocked[col] ? 0 : v)));
    rates = inverseVelocity(joints, maskedJacobian, error, options).rates.map((v, col) => (blocked[col] ? 0 : v));
  }
  return rates;
}

export function inverseKinematics(input, target, { mode = 'position', length = 5, damping = 0.02, maxIterations = 160, tolerance = 1e-5, toolType = 'gripper', environment = null } = {}) {
  const initial = normalizeJoints(input);
  if (!target || target.position?.length !== 3 || target.rpy?.length !== 3 || ![...target.position, ...target.rpy].every(Number.isFinite)) throw new Error('Alvo inválido.');
  if (!TASK_ROWS[mode] || !Number.isFinite(length) || length <= 0 || !Number.isFinite(damping) || damping < 0 || !Number.isInteger(maxIterations) || maxIterations < 1 || !Number.isFinite(tolerance) || tolerance <= 0) throw new Error('Opções do solver inválidas.');
  const rows = TASK_ROWS[mode];
  const score = error => norm(rows.map(row => error[row] / (row < 3 ? length : 1)));
  let best = null;
  let iterations = 0;
  // Deterministic restarts allow escape from stationary seeds, including a straight arm.
  for (let seed = 0; seed < 4; seed++) {
    let joints = initial.map((j, i) => ({ ...j, val: seed === 0 ? j.val : seed === 1 ? (j.min + j.max) / 2 : j.min + (j.max - j.min) * (0.5 + 0.28 * Math.sin((i + 1) * (seed + 1))) }));
    if (detectSelfCollision(joints, null, toolType, environment).colliding) {
      if (seed > 0) continue;
      if (best === null) {
        best = { joints, error: Infinity, residual: [Infinity, Infinity, Infinity, 0, 0, 0], converged: false };
      }
      continue;
    }
    for (let iteration = 0; iteration < maxIterations; iteration++) {
      iterations++;
      const fk = forwardKinematics(joints);
      const error = poseError(fk, target);
      const errorNorm = score(error);
      if (!best || errorNorm < best.error) best = { joints, error: errorNorm, residual: error, converged: errorNorm <= tolerance };
      if (best.converged) return { ...best, iterations };
      const rates = solveConstrainedRates(joints, fk.jacobian, error, { mode, length, damping });
      if (rates.every(v => v === 0)) break;
      const largestStep = Math.max(...rates.map((v, i) => {
        const span = Math.max(joints[i].max - joints[i].min, 1e-4);
        const maxStep = joints[i].type === 'R' ? 0.25 : Math.max(1e-4, Math.min(length, span) * 0.25);
        return Math.abs(v) / maxStep;
      }), 1);
      let improved = false;
      for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625]) {
        const rawCandidate = joints.map((joint, i) => ({
          ...joint,
          val: clamp(joint.val + (rates[i] * (fraction / largestStep)) / (joint.type === 'R' ? DEG : 1), joint.min, joint.max),
        }));
        if (detectSelfCollision(rawCandidate, null, toolType, environment).colliding) continue;
        const swept = sweepJointsCollisionFree(joints, rawCandidate, toolType, environment);
        const candidate = swept.blocked ? swept.joints : rawCandidate;
        if (detectSelfCollision(candidate, null, toolType, environment).colliding) continue;
        const nextResidual = poseError(forwardKinematics(candidate), target);
        const nextError = score(nextResidual);
        if (nextError < errorNorm - 1e-13) {
          joints = candidate; improved = true;
          if (nextError < best.error) best = { joints, error: nextError, residual: nextResidual, converged: nextError <= tolerance };
          if (best.converged) return { ...best, iterations };
          break;
        }
      }
      if (!improved) break;
    }
  }
  return {
    ...(best || { joints: initial, error: Infinity, residual: [Infinity, Infinity, Infinity, 0, 0, 0], converged: false }),
    iterations,
  };
}
