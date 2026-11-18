import { Euler, Matrix4, Quaternion, Vector3 } from 'three';
import { Matrix, SingularValueDecomposition, determinant } from 'ml-matrix';
import {
  ANALYTICAL_ARCH_LABELS,
  POSTURE_AXES_BY_ARCH,
  classifyPosture,
  detectAnalyticalArchitecture,
  formatPostureLabel,
  generateAnalyticalCandidates,
  jointDisplacementScore,
  matchesPosturePreference,
  maxRevoluteJumpDeg,
} from './analyticalIk.js';

export {
  ANALYTICAL_ARCH_LABELS,
  POSTURE_AXES_BY_ARCH,
  classifyPosture,
  detectAnalyticalArchitecture,
  formatPostureLabel,
};

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
export const DEFAULT_TOOL_OFFSET_RPY = [0, 0, 0];
export const LINK_SHAPES = {
  cylinder: { id: 'cylinder', label: 'Cilíndrica' },
  square: { id: 'square', label: 'Quadrada' },
  hex: { id: 'hex', label: 'Hexagonal' },
  flat: { id: 'flat', label: 'Barra chata' },
};
export const LINK_SHAPE_LIST = Object.values(LINK_SHAPES);
export const MODEL_STYLE_BOUNDS = {
  linkRadius: [0.02, 0.35],
  revRadius: [0.08, 0.8],
  revLength: [0.1, 1.2],
  prismWidth: [0.1, 1.0],
  prismLength: [0.15, 1.5],
};
export const DEFAULT_MODEL_STYLE = {
  linkShape: 'cylinder',
  linkRadius: 0.065,
  revRadius: 0.3,
  revLength: 0.4,
  prismWidth: 0.35,
  prismLength: 0.6,
};
export const DEFAULT_JOINT_COLORS = {
  revColor: '#e65c93',
  prismColor: '#42b1e1',
  linkColor: '#70d9b0',
};
export const JOINT_CHART_PALETTE = [
  '#e65c93',
  '#4da6ff',
  '#f3b74f',
  '#62d5b5',
  '#b59cff',
  '#ff9868',
  '#7cd96e',
  '#e58fd8',
  '#22d3ee',
  '#fb7185',
  '#facc15',
  '#34d399',
  '#a78bfa',
  '#fb923c',
  '#38bdf8',
  '#f472b6',
  '#a3e635',
  '#c084fc',
  '#f87171',
  '#2dd4bf',
  '#818cf8',
  '#e879f9',
  '#4ade80',
  '#fbbf24',
];
export function normalizeHexColor(val) {
  if (typeof val !== 'string') return null;
  const trimmed = val.trim().toLowerCase();
  return /^#[0-9a-f]{6}$/.test(trimmed) ? trimmed : null;
}
export function hslToHex(h, s = 82, l = 62) {
  const hue = ((Number(h) % 360) + 360) % 360;
  const sat = Math.max(0, Math.min(100, Number(s) || 0)) / 100;
  const light = Math.max(0, Math.min(100, Number(l) || 0)) / 100;
  const k = (n) => (n + hue / 30) % 12;
  const a = sat * Math.min(light, 1 - light);
  const f = (n) => light - a * Math.max(-1, Math.min(k(n) - 3, 9 - k(n), 1));
  const toHex = (x) => Math.round(Math.max(0, Math.min(1, x)) * 255).toString(16).padStart(2, '0');
  return `#${toHex(f(0))}${toHex(f(8))}${toHex(f(4))}`;
}
export function hexToHsl(hex) {
  const norm = normalizeHexColor(hex);
  if (!norm) return { h: 0, s: 80, l: 60 };
  const r = parseInt(norm.slice(1, 3), 16) / 255;
  const g = parseInt(norm.slice(3, 5), 16) / 255;
  const b = parseInt(norm.slice(5, 7), 16) / 255;
  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);
  const l = (max + min) / 2;
  const d = max - min;
  if (d === 0) {
    return { h: 0, s: 0, l: Math.round(l * 100) };
  }
  const s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
  let h = 0;
  if (max === r) h = (g - b) / d + (g < b ? 6 : 0);
  else if (max === g) h = (b - r) / d + 2;
  else h = (r - g) / d + 4;
  return {
    h: Math.round((h * 60 + 360) % 360),
    s: Math.round(s * 100),
    l: Math.round(l * 100),
  };
}
export function normalizeModelStyle(raw) {
  const src = raw && typeof raw === 'object' ? raw : {};
  const linkShape = typeof src.linkShape === 'string' && LINK_SHAPES[src.linkShape]
    ? src.linkShape
    : DEFAULT_MODEL_STYLE.linkShape;
  const pick = (obj, key, fallback) => {
    const [lo, hi] = MODEL_STYLE_BOUNDS[key];
    const n = Number(obj[key]);
    const clamped = Number.isFinite(n) ? clamp(n, lo, hi) : fallback;
    return Math.round(clamped * 10000) / 10000;
  };
  const result = {
    linkShape,
    linkRadius: pick(src, 'linkRadius', DEFAULT_MODEL_STYLE.linkRadius),
    revRadius: pick(src, 'revRadius', DEFAULT_MODEL_STYLE.revRadius),
    revLength: pick(src, 'revLength', DEFAULT_MODEL_STYLE.revLength),
    prismWidth: pick(src, 'prismWidth', DEFAULT_MODEL_STYLE.prismWidth),
    prismLength: pick(src, 'prismLength', DEFAULT_MODEL_STYLE.prismLength),
  };
  const revColor = normalizeHexColor(src.revColor);
  if (revColor && revColor !== DEFAULT_JOINT_COLORS.revColor) result.revColor = revColor;
  const prismColor = normalizeHexColor(src.prismColor);
  if (prismColor && prismColor !== DEFAULT_JOINT_COLORS.prismColor) result.prismColor = prismColor;
  const linkColor = normalizeHexColor(src.linkColor);
  if (linkColor && linkColor !== DEFAULT_JOINT_COLORS.linkColor) result.linkColor = linkColor;
  if (src.perJoint && typeof src.perJoint === 'object' && !Array.isArray(src.perJoint)) {
    const cleaned = {};
    for (const [k, v] of Object.entries(src.perJoint)) {
      if (!/^\d+$/.test(k) || !v || typeof v !== 'object') continue;
      const entry = {};
      if (typeof v.linkShape === 'string' && LINK_SHAPES[v.linkShape]) {
        entry.linkShape = v.linkShape;
      }
      for (const dimKey of Object.keys(MODEL_STYLE_BOUNDS)) {
        if (v[dimKey] !== undefined && v[dimKey] !== null && v[dimKey] !== '' && Number.isFinite(Number(v[dimKey]))) {
          entry[dimKey] = pick(v, dimKey, result[dimKey]);
        }
      }
      const jointColor = normalizeHexColor(v.jointColor);
      if (jointColor) entry.jointColor = jointColor;
      const jointLinkColor = normalizeHexColor(v.linkColor);
      if (jointLinkColor) entry.linkColor = jointLinkColor;
      if (Object.keys(entry).length > 0) cleaned[k] = entry;
    }
    if (Object.keys(cleaned).length > 0) {
      result.perJoint = cleaned;
    }
  }
  return result;
}
export function resolveJointModelStyle(modelStyle, jointId, jointType = 'R', jointIndex = null) {
  const base = normalizeModelStyle(modelStyle);
  const override = base.perJoint?.[String(jointId)];
  const indexedPaletteColor = Number.isInteger(jointIndex) && jointIndex >= 0
    ? JOINT_CHART_PALETTE[jointIndex % JOINT_CHART_PALETTE.length]
    : null;
  const revColor = override?.jointColor || base.revColor || indexedPaletteColor || DEFAULT_JOINT_COLORS.revColor;
  const prismColor = override?.jointColor || base.prismColor || indexedPaletteColor || DEFAULT_JOINT_COLORS.prismColor;
  const customJointColor = override?.jointColor || (jointType === 'P' ? base.prismColor : base.revColor) || null;
  const jointColor = jointType === 'P' ? prismColor : revColor;
  const linkColor = override?.linkColor || base.linkColor || DEFAULT_JOINT_COLORS.linkColor;
  if (!override) {
    return {
      linkShape: base.linkShape,
      linkRadius: base.linkRadius,
      revRadius: base.revRadius,
      revLength: base.revLength,
      prismWidth: base.prismWidth,
      prismLength: base.prismLength,
      revColor,
      prismColor,
      jointColor,
      customJointColor,
      linkColor,
      overridden: false,
    };
  }
  return {
    linkShape: override.linkShape || base.linkShape,
    linkRadius: override.linkRadius ?? base.linkRadius,
    revRadius: override.revRadius ?? base.revRadius,
    revLength: override.revLength ?? base.revLength,
    prismWidth: override.prismWidth ?? base.prismWidth,
    prismLength: override.prismLength ?? base.prismLength,
    revColor,
    prismColor,
    jointColor,
    customJointColor,
    linkColor,
    overridden: true,
  };
}
export function isDefaultModelStyle(style) {
  const s = normalizeModelStyle(style);
  return !s.perJoint
    && !s.revColor
    && !s.prismColor
    && !s.linkColor
    && s.linkShape === DEFAULT_MODEL_STYLE.linkShape
    && Math.abs(s.linkRadius - DEFAULT_MODEL_STYLE.linkRadius) < 1e-6
    && Math.abs(s.revRadius - DEFAULT_MODEL_STYLE.revRadius) < 1e-6
    && Math.abs(s.revLength - DEFAULT_MODEL_STYLE.revLength) < 1e-6
    && Math.abs(s.prismWidth - DEFAULT_MODEL_STYLE.prismWidth) < 1e-6
    && Math.abs(s.prismLength - DEFAULT_MODEL_STYLE.prismLength) < 1e-6;
}
export function resolveJointChartColor(modelStyle, joint, index = 0) {
  return resolveJointModelStyle(modelStyle, joint?.id, joint?.type || 'R', index).jointColor;
}
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

export function computeToolFrame(fkTotal, toolOffsetRpy = DEFAULT_TOOL_OFFSET_RPY, toolType = 'gripper', toolScale = 1) {
  const spec = TOOL_TYPES[toolType] || TOOL_TYPES.gripper;
  const safeScale = Number.isFinite(Number(toolScale)) ? clamp(Number(toolScale), 0.4, 4.0) : 1;
  const safeOffset = Array.isArray(toolOffsetRpy) && toolOffsetRpy.length === 3
    ? toolOffsetRpy.map(v => (Number.isFinite(v) ? v : 0))
    : DEFAULT_TOOL_OFFSET_RPY;
  const offsetMatrix = rotationFromRPY(safeOffset);
  const toolMatrix = fkTotal.clone().multiply(offsetMatrix);
  const tcpVec = new Vector3(spec.length * safeScale, 0, 0).applyMatrix4(toolMatrix);
  const approachVec = new Vector3().setFromMatrixColumn(toolMatrix, 0).normalize();
  return {
    tool: spec,
    scale: safeScale,
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

const _vAxis = new Vector3();
const _vEnd = new Vector3();
const _vJointPos = new Vector3();
const _vDelta = new Vector3();
const _vLinear = new Vector3();
const _vScratch = new Vector3();

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
  const positions = transforms.map(m => [m.elements[12], m.elements[13], m.elements[14]]);
  const endArr = positions.at(-1);
  _vEnd.set(endArr[0], endArr[1], endArr[2]);
  const jacobian = Array.from({ length: 6 }, () => []);
  for (let index = 0; index < joints.length; index++) {
    const joint = joints[index];
    _vAxis.setFromMatrixColumn(transforms[index], 2);
    if (joint.type === 'P') {
      jacobian[0].push(_vAxis.x);
      jacobian[1].push(_vAxis.y);
      jacobian[2].push(_vAxis.z);
      jacobian[3].push(0);
      jacobian[4].push(0);
      jacobian[5].push(0);
    } else {
      const p = positions[index];
      _vJointPos.set(p[0], p[1], p[2]);
      _vDelta.subVectors(_vEnd, _vJointPos);
      _vLinear.crossVectors(_vAxis, _vDelta);
      jacobian[0].push(_vLinear.x);
      jacobian[1].push(_vLinear.y);
      jacobian[2].push(_vLinear.z);
      jacobian[3].push(_vAxis.x);
      jacobian[4].push(_vAxis.y);
      jacobian[5].push(_vAxis.z);
    }
  }
  return {
    total,
    transforms,
    links,
    positions,
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
    _vScratch.set(0, 0, d).applyMatrix4(fk.transforms[i]);
    const axialEnd = [_vScratch.x, _vScratch.y, _vScratch.z];
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
    _vScratch.set(toolSpec.length, 0, 0).applyMatrix4(fk.transforms[n]);
    gripTip = [_vScratch.x, _vScratch.y, _vScratch.z];
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

/**
 * Computes static joint efforts (torques for R, forces for P) via tau = J^T * F
 * @param {Array} joints - Robot joint definitions
 * @param {Array} jacobian - 6xn geometric Jacobian matrix
 * @param {Array} wrench - [Fx, Fy, Fz, Mx, My, Mz] end-effector wrench
 */
export function staticJointEfforts(joints, jacobian, wrench) {
  if (!Array.isArray(wrench) || wrench.length !== 6 || !wrench.every(Number.isFinite)) {
    throw new Error('Vetor de força/momento inválido.');
  }
  const efforts = joints.map((_, col) => {
    let sum = 0;
    for (let row = 0; row < 6; row++) {
      sum += (jacobian[row]?.[col] ?? 0) * wrench[row];
    }
    return sum;
  });
  const labels = joints.map((j, i) => (j.type === 'R' ? `T${i + 1}` : `F${i + 1}`));
  const units = joints.map(j => (j.type === 'R' ? 'N·u' : 'N'));
  return { efforts, labels, units };
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

function normalizePosturePreference(posture) {
  if (!posture || posture === 'auto' || typeof posture !== 'object') return null;
  const entries = Object.entries(posture).filter(([, v]) => v && v !== 'auto');
  return entries.length ? Object.fromEntries(entries) : null;
}

function isNearJointSingularity(joints, arch) {
  if (!arch || !Array.isArray(joints)) return false;
  if (arch === 'scara' && joints[1]) {
    return Math.abs(Math.sin(joints[1].val * DEG)) < 0.08;
  }
  if ((arch === 'puma560' || arch === 'ur5') && joints[2] && joints[4]) {
    return Math.abs(Math.sin(joints[2].val * DEG)) < 0.08
      || Math.abs(Math.sin(joints[4].val * DEG)) < 0.08;
  }
  return false;
}

function evaluateAnalyticalBranches(initial, flangeTarget, {
  solveMode = 'pose',
  errorMode = solveMode,
  length = 5,
  tolerance = 1e-5,
  toolType = 'gripper',
  environment = null,
} = {}) {
  const candidates = generateAnalyticalCandidates(initial, flangeTarget, solveMode);
  const evalRows = TASK_ROWS[errorMode] ?? 3;
  const accepted = [];
  for (const cand of candidates) {
    if (validateJoints(cand.joints).length !== 0) continue;
    if (detectSelfCollision(cand.joints, null, toolType, environment).colliding) continue;
    const fk = forwardKinematics(cand.joints);
    const residual = poseError(fk, flangeTarget);
    const err = Math.hypot(...residual.slice(0, evalRows));
    if (err > Math.max(tolerance * 10, 1e-3)) continue;
    accepted.push({
      id: cand.id,
      joints: normalizeJoints(cand.joints).map(j => ({ ...j, speed: 0 })),
      posture: cand.posture,
      architecture: cand.architecture,
      label: cand.label,
      error: err,
      residual,
      converged: err <= Math.max(tolerance, 1e-4),
      iterations: 0,
      solver: 'analytical',
      distance: jointDisplacementScore(cand.joints, initial, length),
    });
  }
  accepted.sort((a, b) => a.distance - b.distance);
  return accepted;
}

function collectFallbackPositionBranches(initial, flangeTarget, existingBranches, {
  length = 5,
  tolerance = 1e-5,
  toolType = 'gripper',
  environment = null,
} = {}) {
  const seenIds = new Set(existingBranches.map(b => b.id));
  const pos = flangeTarget.position || [0, 0, 0];
  const radialYaw = Math.atan2(pos[1], pos[0]) / DEG;
  const extraRpys = [
    [0, 0, radialYaw],
    [180, 0, radialYaw],
    [90, 0, radialYaw],
    [-90, 0, radialYaw],
    [0, 90, 0],
    [0, -90, 0],
    [0, 0, 0],
    [180, 0, 0],
  ];
  const extras = [];
  for (const rpy of extraRpys) {
    const solved = evaluateAnalyticalBranches(initial, { position: pos, rpy }, {
      solveMode: 'pose',
      errorMode: 'position',
      length,
      tolerance,
      toolType,
      environment,
    });
    for (const branch of solved) {
      if (seenIds.has(branch.id)) continue;
      seenIds.add(branch.id);
      extras.push({ ...branch, orientationAdjusted: true });
    }
  }
  extras.sort((a, b) => a.distance - b.distance);
  return extras;
}

export function listPostureBranches(input, target, {
  mode = 'position',
  length = 5,
  tolerance = 1e-5,
  toolType = 'gripper',
  toolOffsetRpy = null,
  environment = null,
} = {}) {
  const initial = normalizeJoints(input);
  const architecture = detectAnalyticalArchitecture(initial);
  if (!architecture || !target || !Array.isArray(target.position)) {
    return { architecture: null, currentPosture: null, branches: [], axes: [] };
  }
  const currentPosture = classifyPosture(initial, architecture);
  const flangeTarget = toolOffsetRpy
    ? { ...target, rpy: flangeRpyFromToolTarget(target.rpy || [0, 0, 0], toolOffsetRpy) }
    : target;

  const primarySolveMode = architecture === 'scara' ? mode : 'pose';
  const primaryBranches = evaluateAnalyticalBranches(initial, flangeTarget, {
    solveMode: primarySolveMode,
    errorMode: primarySolveMode,
    length,
    tolerance,
    toolType,
    environment,
  }).map(b => ({ ...b, orientationAdjusted: false }));

  let branches = primaryBranches;
  if ((architecture === 'ur5' || architecture === 'puma560') && mode === 'position' && primaryBranches.length < 8) {
    const fallbackExtras = collectFallbackPositionBranches(initial, flangeTarget, primaryBranches, {
      length,
      tolerance,
      toolType,
      environment,
    });
    branches = [...primaryBranches, ...fallbackExtras];
  }

  return {
    architecture,
    currentPosture,
    axes: POSTURE_AXES_BY_ARCH[architecture] || [],
    branches,
  };
}

export function inverseKinematics(input, target, {
  mode = 'position',
  length = 5,
  damping = 0.02,
  maxIterations = 160,
  tolerance = 1e-5,
  maxSeeds = 4,
  toolType = 'gripper',
  environment = null,
  posture = null,
  strictPosture = null,
  preferAnalytical = true,
  continuityToleranceDeg = null,
} = {}) {
  const initial = normalizeJoints(input);
  if (!target || target.position?.length !== 3 || target.rpy?.length !== 3 || ![...target.position, ...target.rpy].every(Number.isFinite)) throw new Error('Alvo inválido.');
  if (!TASK_ROWS[mode] || !Number.isFinite(length) || length <= 0 || !Number.isFinite(damping) || damping < 0 || !Number.isInteger(maxIterations) || maxIterations < 1 || !Number.isFinite(tolerance) || tolerance <= 0 || !Number.isInteger(maxSeeds) || maxSeeds < 1) throw new Error('Opções do solver inválidas.');
  const rows = TASK_ROWS[mode];
  const score = error => norm(rows.map(row => error[row] / (row < 3 ? length : 1)));
  let best = null;
  const arch = preferAnalytical ? detectAnalyticalArchitecture(initial) : null;
  const posturePref = normalizePosturePreference(posture);
  const hasExplicitPosture = Boolean(posturePref);
  const enforceStrict = strictPosture ?? hasExplicitPosture;
  const initialNearSing = arch ? isNearJointSingularity(initial, arch) : false;

  const filterContinuous = branches => {
    if (!Number.isFinite(continuityToleranceDeg) || initialNearSing) return branches;
    return branches.filter(b => maxRevoluteJumpDeg(initial, b.joints) <= continuityToleranceDeg);
  };

  if (arch) {
    if (mode === 'pose' || arch === 'scara') {
      const exactBranches = filterContinuous(evaluateAnalyticalBranches(initial, target, {
        solveMode: mode,
        errorMode: mode,
        length,
        tolerance,
        toolType,
        environment,
      }));
      if (hasExplicitPosture) {
        const matching = exactBranches.filter(b => matchesPosturePreference(b.posture, posturePref));
        if (matching.length > 0) {
          return { ...matching[0], branches: exactBranches };
        }
        if (enforceStrict && exactBranches.length > 0) {
          const curFk = forwardKinematics(initial);
          const curRes = poseError(curFk, target);
          return {
            joints: initial,
            error: score(curRes),
            residual: curRes,
            converged: false,
            iterations: 0,
            solver: 'analytical',
            architecture: arch,
            posture: classifyPosture(initial, arch),
            branches: exactBranches,
            postureUnavailable: true,
          };
        }
      } else if (exactBranches.length > 0) {
        return { ...exactBranches[0], branches: exactBranches };
      }
    } else if ((arch === 'ur5' || arch === 'puma560') && mode === 'position') {
      const exactBranches = filterContinuous(evaluateAnalyticalBranches(initial, target, {
        solveMode: 'pose',
        errorMode: 'pose',
        length,
        tolerance,
        toolType,
        environment,
      }));
      if (hasExplicitPosture) {
        let matching = exactBranches.filter(b => matchesPosturePreference(b.posture, posturePref));
        let allBranches = exactBranches;
        if (matching.length === 0) {
          const extras = filterContinuous(collectFallbackPositionBranches(initial, target, exactBranches, {
            length,
            tolerance,
            toolType,
            environment,
          }));
          allBranches = [...exactBranches, ...extras];
          matching = allBranches.filter(b => matchesPosturePreference(b.posture, posturePref));
        }
        if (matching.length > 0) {
          return { ...matching[0], branches: allBranches };
        }
      } else if (exactBranches.length > 0 && (!Number.isFinite(continuityToleranceDeg) || maxSeeds > 1 || exactBranches[0].distance < 0.75)) {
        return { ...exactBranches[0], branches: exactBranches };
      }
    }
  }

  const finalizeDlsResult = (resultObj) => {
    const out = resultObj || { joints: initial, error: Infinity, residual: [Infinity, Infinity, Infinity, 0, 0, 0], converged: false };
    const outPosture = arch ? classifyPosture(out.joints, arch) : null;
    const postureOk = !hasExplicitPosture || !enforceStrict || !arch || matchesPosturePreference(outPosture, posturePref);
    return {
      ...out,
      converged: Boolean(out.converged && postureOk),
      iterations,
      solver: 'dls',
      architecture: arch,
      posture: outPosture,
      ...(postureOk ? {} : { postureUnavailable: true }),
    };
  };

  let iterations = 0;
  // Deterministic restarts allow escape from stationary seeds, including a straight arm.
  for (let seed = 0; seed < maxSeeds; seed++) {
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
      if (best.converged) {
        const finalized = finalizeDlsResult(best);
        if (finalized.converged) return finalized;
      }
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
          if (best.converged) {
            const finalized = finalizeDlsResult(best);
            if (finalized.converged) return finalized;
          }
          break;
        }
      }
      if (!improved) break;
    }
  }
  return finalizeDlsResult(best);
}

/**
 * Solves IK warm-started from `inputJoints` for interactive 3D gizmo dragging,
 * returning the actual achievable position and tool RPY clamped to joint limits,
 * workspace boundaries, and collision constraints (floor / obstacle / self-collision).
 */
export function solveBoundedTarget(inputJoints, target, {
  mode = 'position',
  length = 5,
  damping = 0.02,
  maxIterations = 40,
  toolType = 'gripper',
  toolOffsetRpy = DEFAULT_TOOL_OFFSET_RPY,
  environment = null,
  lockPosition = false,
  lockOrientation = false,
  posture = null,
} = {}) {
  const current = normalizeJoints(inputJoints).map(j => ({ ...j, speed: 0 }));
  const currentFk = forwardKinematics(current);
  const currentEnd = currentFk.positions.at(-1);
  const currentTool = computeToolFrame(currentFk.total, toolOffsetRpy, toolType);
  const flangeTarget = {
    position: target.position,
    rpy: flangeRpyFromToolTarget(target.rpy, toolOffsetRpy),
  };
  const effectiveMode = lockOrientation ? 'pose' : mode;
  const effectiveLength = lockPosition
    ? Math.min(length, 0.35)
    : (lockOrientation ? Math.max(length, 10) : length);
  const continuityToleranceDeg = (lockPosition || lockOrientation) ? 95 : null;

  let res = inverseKinematics(current, flangeTarget, {
    mode: effectiveMode,
    length: effectiveLength,
    damping,
    maxIterations,
    maxSeeds: 1,
    toolType,
    environment,
    posture,
    strictPosture: false,
    continuityToleranceDeg,
  });

  // If warm-start made zero progress at a singular straight-arm interior pose, try a small local perturbation.
  if (!res.converged && res.iterations <= 2 && res.error > 1e-4
    && current.every((j, i) => Math.abs(j.val - res.joints[i].val) < 1e-5)) {
    for (let idx = current.length - 1; idx >= 0; idx--) {
      if (current[idx].type !== 'R') continue;
      for (const delta of [2, -2]) {
        const candidateVal = clamp(current[idx].val + delta, current[idx].min, current[idx].max);
        if (Math.abs(candidateVal - current[idx].val) < 0.1) continue;
        const nudged = current.map((j, k) => (k === idx ? { ...j, val: candidateVal } : j));
        const sweptSeed = sweepJointsCollisionFree(current, nudged, toolType, environment);
        if (sweptSeed.blocked) continue;
        const attempt = inverseKinematics(sweptSeed.joints, flangeTarget, {
          mode: effectiveMode,
          length: effectiveLength,
          damping,
          maxIterations,
          maxSeeds: 1,
          toolType,
          environment,
        });
        if (attempt.error < res.error - 1e-6 && !sweepJointsCollisionFree(current, attempt.joints, toolType, environment).blocked) {
          res = attempt;
          break;
        }
      }
      if (res.converged || res.error < 1e-4) break;
    }
  }

  const isSafeCandidate = (candJoints) => candJoints
    && validateJoints(candJoints).length === 0
    && !detectSelfCollision(candJoints, null, toolType, environment).colliding;

  let candidate = isSafeCandidate(res.joints)
    ? normalizeJoints(res.joints).map(j => ({ ...j, speed: 0 }))
    : null;

  if (lockOrientation) {
    const qAnchor = new Quaternion().setFromRotationMatrix(rotationFromRPY(flangeTarget.rpy)).normalize();
    const rotDriftRad = (candJoints) => {
      const qCand = new Quaternion().setFromRotationMatrix(forwardKinematics(candJoints, { includeOrientation: false }).total).normalize();
      return 2 * Math.acos(clamp(Math.abs(qAnchor.dot(qCand)), 0, 1));
    };
    const isOrientationOk = (candJoints) => candJoints
      && isSafeCandidate(candJoints)
      && rotDriftRad(candJoints) <= 1e-3
      && !sweepJointsCollisionFree(current, candJoints, toolType, environment).blocked;

    if (!candidate || !isOrientationOk(candidate)) {
      let lo = 0;
      let hi = 1;
      let best = current;
      const delta = [
        target.position[0] - currentEnd[0],
        target.position[1] - currentEnd[1],
        target.position[2] - currentEnd[2],
      ];
      for (let stepIdx = 0; stepIdx < 7; stepIdx++) {
        const mid = (lo + hi) / 2;
        const midPos = [
          currentEnd[0] + delta[0] * mid,
          currentEnd[1] + delta[1] * mid,
          currentEnd[2] + delta[2] * mid,
        ];
        const attempt = inverseKinematics(best, { position: midPos, rpy: flangeTarget.rpy }, {
          mode: 'pose',
          length: effectiveLength,
          damping,
          maxIterations: 35,
          maxSeeds: 1,
          toolType,
          environment,
        });
        if (attempt.converged && isOrientationOk(attempt.joints)) {
          best = normalizeJoints(attempt.joints).map(j => ({ ...j, speed: 0 }));
          lo = mid;
        } else {
          hi = mid;
        }
      }

      if (lo === 0) {
        // Diagonal move blocked immediately; try sliding smoothly along each individual axis while keeping orientation fixed
        let bestSlideDist = 0;
        for (let axis = 0; axis < 3; axis++) {
          if (Math.abs(delta[axis]) < 1e-6) continue;
          let sLo = 0;
          let sHi = 1;
          let axisBest = current;
          for (let stepIdx = 0; stepIdx < 5; stepIdx++) {
            const mid = (sLo + sHi) / 2;
            const slidePos = [
              currentEnd[0] + (axis === 0 ? delta[0] * mid : 0),
              currentEnd[1] + (axis === 1 ? delta[1] * mid : 0),
              currentEnd[2] + (axis === 2 ? delta[2] * mid : 0),
            ];
            const attempt = inverseKinematics(axisBest, { position: slidePos, rpy: flangeTarget.rpy }, {
              mode: 'pose',
              length: effectiveLength,
              damping,
              maxIterations: 32,
              maxSeeds: 1,
              toolType,
              environment,
            });
            if (attempt.converged && isOrientationOk(attempt.joints)) {
              axisBest = normalizeJoints(attempt.joints).map(j => ({ ...j, speed: 0 }));
              sLo = mid;
            } else {
              sHi = mid;
            }
          }
          const moved = Math.abs(delta[axis] * sLo);
          if (sLo > 0 && moved > bestSlideDist) {
            bestSlideDist = moved;
            best = axisBest;
          }
        }
      }

      if (best === current) {
        return {
          joints: current,
          position: [...currentEnd],
          rpy: [...target.rpy],
          converged: false,
          iterations: res.iterations,
          error: res.error,
          residual: res.residual,
          blocked: true,
        };
      }
      candidate = best;
    }
  }

  if (candidate) {
    if (lockPosition) {
      let candPos = forwardKinematics(candidate, { includeOrientation: false }).positions.at(-1);
      let drift = Math.hypot(
        candPos[0] - target.position[0],
        candPos[1] - target.position[1],
        candPos[2] - target.position[2]
      );
      if (drift > 5e-5) {
        const polish = inverseKinematics(candidate, flangeTarget, {
          mode: 'position',
          length: Math.max(1, length),
          damping: Math.min(damping, 0.01),
          maxIterations: 20,
          maxSeeds: 1,
          tolerance: 1e-6,
          toolType,
          environment,
        });
        if (isSafeCandidate(polish.joints)) {
          const polishedNorm = normalizeJoints(polish.joints).map(j => ({ ...j, speed: 0 }));
          const polishPos = forwardKinematics(polishedNorm, { includeOrientation: false }).positions.at(-1);
          const polishDrift = Math.hypot(
            polishPos[0] - target.position[0],
            polishPos[1] - target.position[1],
            polishPos[2] - target.position[2]
          );
          if (polishDrift < drift) {
            candidate = polishedNorm;
            candPos = polishPos;
            drift = polishDrift;
          }
        }
      }
      if (drift > 1e-3 || sweepJointsCollisionFree(current, candidate, toolType, environment).blocked) {
        return {
          joints: current,
          position: [...target.position],
          rpy: [...currentTool.orientation.rpy],
          converged: false,
          iterations: res.iterations,
          error: res.error,
          residual: res.residual,
          blocked: true,
        };
      }
    }

    const solvedFk = forwardKinematics(candidate);
    const rawPos = solvedFk.positions.at(-1);
    const achievedPos = (!lockOrientation && mode === 'xy') ? [rawPos[0], rawPos[1], currentEnd[2]] : rawPos;
    const achievedTool = computeToolFrame(solvedFk.total, toolOffsetRpy, toolType);
    return {
      joints: candidate,
      position: achievedPos,
      rpy: achievedTool.orientation.rpy,
      converged: lockOrientation ? true : res.converged,
      iterations: res.iterations,
      error: res.error,
      residual: res.residual,
      blocked: false,
      solver: res.solver || 'dls',
      architecture: res.architecture || detectAnalyticalArchitecture(candidate),
      posture: classifyPosture(candidate, res.architecture || detectAnalyticalArchitecture(candidate)),
    };
  }

  return {
    joints: current,
    position: [...(lockPosition ? target.position : currentEnd)],
    rpy: [...(lockOrientation ? target.rpy : currentTool.orientation.rpy)],
    converged: false,
    iterations: res.iterations,
    error: res.error,
    residual: res.residual,
    blocked: true,
  };
}

/**
 * Computes the 3D translational manipulability ellipsoid (radii + SO(3) orientation)
 * from the top 3 rows of the geometric Jacobian via SVD.
 */
export function computeManipulabilityEllipsoid(jacobian) {
  if (!Array.isArray(jacobian) || jacobian.length < 3 || !Array.isArray(jacobian[0]) || jacobian[0].length === 0) {
    return null;
  }
  const cols = jacobian[0].length;
  const paddedCols = Math.max(3, cols);
  const jv = [0, 1, 2].map(r => {
    const row = Array.isArray(jacobian[r]) ? jacobian[r] : [];
    const out = new Array(paddedCols).fill(0);
    for (let c = 0; c < cols; c++) {
      const val = row[c];
      out[c] = Number.isFinite(val) ? val : 0;
    }
    return out;
  });

  const svd = new SingularValueDecomposition(new Matrix(jv), { autoTranspose: true });
  const s = [svd.diagonal[0] ?? 0, svd.diagonal[1] ?? 0, svd.diagonal[2] ?? 0];
  const U = svd.leftSingularVectors;
  const sign = determinant(U) < 0 ? -1 : 1;

  const rotMat = new Matrix4().set(
    U.get(0, 0), U.get(0, 1), U.get(0, 2) * sign, 0,
    U.get(1, 0), U.get(1, 1), U.get(1, 2) * sign, 0,
    U.get(2, 0), U.get(2, 1), U.get(2, 2) * sign, 0,
    0, 0, 0, 1
  );
  const q = new Quaternion().setFromRotationMatrix(rotMat).normalize();
  const maxS = Math.max(s[0], 1e-6);
  const radii = s.map(v => Math.max(0.04, Math.min(0.55, (v / maxS) * 0.42)));

  return {
    singularValues: s,
    radii,
    quaternion: [q.x, q.y, q.z, q.w],
    manipulability: s[0] * s[1] * s[2],
  };
}
