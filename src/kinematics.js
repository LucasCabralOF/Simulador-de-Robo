import { Euler, Matrix4, Quaternion, Vector3 } from 'three';
import { Matrix, SingularValueDecomposition, determinant } from 'ml-matrix';

export const DEG = Math.PI / 180;
export const DEFAULT_JOINTS = [
  { id: 1, type: 'R', a: 0, alpha: 90, d: 0, theta: 0, val: 45, min: -180, max: 180, speed: 0 },
  { id: 2, type: 'P', a: 0, alpha: -90, d: 0, theta: 0, val: 5, min: 0, max: 10, speed: 0 },
  { id: 3, type: 'R', a: 5, alpha: 0, d: 0, theta: 0, val: 30, min: -180, max: 180, speed: 0 },
];
export const TASK_ROWS = { xy: [0, 1], position: [0, 1, 2], pose: [0, 1, 2, 3, 4, 5] };
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

export function forwardKinematics(joints) {
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
  return { total, transforms, links, positions: positions.map(p => p.toArray()), jacobian, orientation: orientation(total) };
}

export function multiplyVector(matrix, vector) {
  return matrix.map(row => row.reduce((sum, value, i) => sum + value * vector[i], 0));
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

export function integrateJoints(joints, rates, dt) {
  if (!Number.isFinite(dt) || dt <= 0 || rates.length !== joints.length || !rates.every(Number.isFinite)) throw new Error('Passo de integração inválido.');
  const limited = [];
  const updated = joints.map((joint, i) => {
    const next = joint.val + rates[i] * dt / (joint.type === 'R' ? DEG : 1);
    const val = clamp(next, joint.min, joint.max);
    if (val !== next) limited.push(i + 1);
    return { ...joint, val };
  });
  return { joints: updated, limited };
}

export function poseError(fk, target) {
  const translation = target.position.map((v, i) => v - fk.positions.at(-1)[i]);
  const desired = new Quaternion().setFromRotationMatrix(rotationFromRPY(target.rpy));
  const current = new Quaternion().setFromRotationMatrix(fk.total);
  return [...translation, ...rotationVector(desired.multiply(current.invert()))];
}

export function inverseKinematics(input, target, { mode = 'position', length = 5, damping = 0.02, maxIterations = 160, tolerance = 1e-5 } = {}) {
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
    for (let iteration = 0; iteration < maxIterations; iteration++) {
      iterations++;
      const fk = forwardKinematics(joints);
      const error = poseError(fk, target);
      const errorNorm = score(error);
      if (!best || errorNorm < best.error) best = { joints, error: errorNorm, residual: error, converged: errorNorm <= tolerance };
      if (best.converged) return { ...best, iterations };
      const { rates } = inverseVelocity(joints, fk.jacobian, error, { mode, length, damping });
      const largestStep = Math.max(...rates.map((v, i) => Math.abs(v) / (joints[i].type === 'R' ? 0.25 : length * 0.25)), 1);
      let improved = false;
      for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625]) {
        const candidate = integrateJoints(joints, rates, fraction / largestStep).joints;
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
  return { ...best, iterations };
}
