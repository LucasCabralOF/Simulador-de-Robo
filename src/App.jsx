import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import RobotScene from './RobotScene';
import SceneToolbar from './SceneToolbar';
import { NumberField } from './ui';
import { DEG, DEFAULT_ENVIRONMENT, DEFAULT_MODEL_STYLE, DEFAULT_TOOL_OFFSET_RPY, TOOL_TYPES, computeToolFrame, detectSelfCollision, flangeRpyFromToolTarget, forwardKinematics, integrateJoints, inverseKinematics, inverseVelocity, jacobianDiagnostics, multiplyVector, normalizeJoints, normalizeModelStyle, rotationFromRPY, solveBoundedTarget, staticJointEfforts, sweepJointsCollisionFree, validateJoints } from './kinematics';
import { PRESETS } from './presets';
import { computeCartesianBounds, generateWorkspaceCloud } from './workspace';
import { generateCpp, generatePython, generateRos2 } from './codeGenerator';
import { defaultWaypointsForJoints, evaluateTrajectory, sampleTrajectoryPath, syncWaypointsToJoints } from './trajectory';
import { exportToURDF, parseURDF } from './urdfParser';
import { parseStlBuffer } from './cadLoader';
import { fmt } from './format';
import ModelTab from './tabs/ModelTab';
import ProjectsTab from './tabs/ProjectsTab';
import ForwardTab from './tabs/ForwardTab';
import DifferentialTab from './tabs/DifferentialTab';
import InverseTab from './tabs/InverseTab';
import TrajectoryTab from './tabs/TrajectoryTab';
import WorkspaceTab from './tabs/WorkspaceTab';
import ExportTab from './tabs/ExportTab';
import ReportTab from './tabs/ReportTab';
import './App.css';

const API = (import.meta.env.VITE_API_URL || '/api').replace(/\/$/, '');

const INITIAL_PRESET = PRESETS.ur5;
const INITIAL_TOOL_OFFSET = Array.isArray(INITIAL_PRESET.toolOffsetRpy) ? [...INITIAL_PRESET.toolOffsetRpy] : [...DEFAULT_TOOL_OFFSET_RPY];
const INITIAL_FK = forwardKinematics(INITIAL_PRESET.joints);
const INITIAL_TOOL_FRAME = computeToolFrame(INITIAL_FK.total, INITIAL_TOOL_OFFSET, 'gripper');

async function request(path = '', options = {}) {
  const res = await fetch(`${API}/simulations${path}`, options);
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || `Falha na API (${res.status}).`);
  return data;
}

function App() {
  const [joints, setJoints] = useState(() => INITIAL_PRESET.joints.map(j => ({ ...j })));
  const jointsRef = useRef(joints);
  useEffect(() => { jointsRef.current = joints; }, [joints]);
  const [tab, setTab] = useState('model');
  const [activePanel, setActivePanel] = useState(null);
  const [simulations, setSimulations] = useState([]);
  const [simName, setSimName] = useState('');
  const [busy, setBusy] = useState(false);
  const [apiError, setApiError] = useState('');
  const [message, setMessage] = useState('');
  const [collisionAlert, setCollisionAlert] = useState(null);
  const [mode, setMode] = useState('position');
  const [length, setLength] = useState(5);
  const [damping, setDamping] = useState(0.02);
  const [dt, setDt] = useState(0.1);
  const [desiredVelocity, setDesiredVelocity] = useState([0, 0, 0, 0, 0, 0]);
  const [wrench, setWrench] = useState([0, 0, 0, 0, 0, 0]);
  const [payloadMass, setPayloadMass] = useState(2);
  const [gravity, setGravity] = useState(10);
  const [target, setTarget] = useState(() => ({
    position: [...INITIAL_FK.positions.at(-1)],
    rpy: [...INITIAL_TOOL_FRAME.orientation.rpy],
  }));
  const [solution, setSolution] = useState(null);
  const [autoSolveIk, setAutoSolveIk] = useState(false);
  const [ikWaypointDuration, setIkWaypointDuration] = useState(2.0);
  const [showFrames, setShowFrames] = useState(true);
  const [viewKey, setViewKey] = useState(0);
  const [workspaceCloud, setWorkspaceCloud] = useState(null);
  const [showWorkspace, setShowWorkspace] = useState(true);
  const [sampleCount, setSampleCount] = useState(2500);
  const [computingCloud, setComputingCloud] = useState(false);
  const [codeTarget, setCodeTarget] = useState('cpp');
  const [copied, setCopied] = useState(false);
  const [waypoints, setWaypoints] = useState(() => defaultWaypointsForJoints(INITIAL_PRESET.joints));
  const [isPlaying, setIsPlaying] = useState(false);
  const [playTime, setPlayTime] = useState(0);
  const playTimeRef = useRef(playTime);
  useEffect(() => { playTimeRef.current = playTime; }, [playTime]);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);
  const [isLooping, setIsLooping] = useState(false);
  const [interpType, setInterpType] = useState('quintic');
  const [angleUnit, setAngleUnit] = useState('deg');
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [shareUrl, setShareUrl] = useState('');
  const [shareLoading, setShareLoading] = useState(false);
  const [shareCopied, setShareCopied] = useState(false);
  const [customStlGeometry, setCustomStlGeometry] = useState(null);
  const [stlFileName, setStlFileName] = useState('');
  const [toolType, setToolType] = useState('gripper');
  const toolTypeRef = useRef(toolType);
  useEffect(() => { toolTypeRef.current = toolType; }, [toolType]);
  const [defaultToolOffset, setDefaultToolOffset] = useState(() => [...INITIAL_TOOL_OFFSET]);
  const [toolOffsetRpy, setToolOffsetRpy] = useState(() => [...INITIAL_TOOL_OFFSET]);
  const [toolGrip, setToolGrip] = useState(50);
  const [modelStyle, setModelStyle] = useState(() => ({ ...DEFAULT_MODEL_STYLE }));
  const [modelTarget, setModelTarget] = useState('global');
  const [environment, setEnvironment] = useState(() => ({
    ...DEFAULT_ENVIRONMENT,
    obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
  }));
  const environmentRef = useRef(environment);
  useEffect(() => { environmentRef.current = environment; }, [environment]);
  const [showGizmo, setShowGizmo] = useState(true);
  const [showObstacleGizmo, setShowObstacleGizmo] = useState(true);
  const [gizmoMode, setGizmoMode] = useState('translate');
  const [isGizmoDragging, setIsGizmoDragging] = useState(false);
  const lastCartesianBoundsRef = useRef(null);
  const [reportMeta, setReportMeta] = useState({
    studentName: 'Lucas Engenharia',
    courseName: 'Robótica Industrial e Cinemática',
    institution: 'Universidade / Faculdade de Engenharia',
    robotName: '',
  });
  const fk = useMemo(() => forwardKinematics(joints), [joints]);
  const toolFrame = useMemo(() => computeToolFrame(fk.total, toolOffsetRpy, toolType), [fk.total, toolOffsetRpy, toolType]);
  const currentEnd = fk.positions.at(-1);
  const liveCollision = useMemo(
    () => detectSelfCollision(joints, fk, toolType, environment),
    [joints, fk, toolType, environment]
  );
  const activeCollision = collisionAlert || (liveCollision.colliding ? liveCollision : null);
  const jointStructureKey = useMemo(
    () => joints.map(j => `${j.type}:${j.a}:${j.alpha}:${j.d}:${j.theta}:${j.min}:${j.max}`).join('|'),
    [joints]
  );
  const globalCartesianBounds = useMemo(
    () => computeCartesianBounds(joints, null, 'position', environment),
    // eslint-disable-next-line react-hooks/exhaustive-deps -- global bounds depend only on kinematic structure/limits and environment, not joint.val
    [jointStructureKey, environment]
  );
  const cartesianBounds = useMemo(() => {
    if (tab !== 'inverse') return globalCartesianBounds;
    if (isGizmoDragging && lastCartesianBoundsRef.current) {
      return lastCartesianBoundsRef.current;
    }
    const next = computeCartesianBounds(joints, currentEnd, mode, environment);
    lastCartesianBoundsRef.current = next;
    return next;
  }, [isGizmoDragging, tab, joints, currentEnd, mode, environment, globalCartesianBounds]);
  const diagnostics = useMemo(
    () => ((tab === 'differential' || tab === 'report') ? jacobianDiagnostics(joints, fk.jacobian, mode, length) : null),
    [tab, joints, fk, mode, length]
  );
  const velocity = useMemo(
    () => (tab === 'differential' ? multiplyVector(fk.jacobian, joints.map(j => j.speed)) : null),
    [tab, fk, joints]
  );
  const differential = useMemo(
    () => (tab === 'differential' ? inverseVelocity(joints, fk.jacobian, desiredVelocity, { mode, length, damping }) : null),
    [tab, joints, fk, desiredVelocity, mode, length, damping]
  );
  const statics = useMemo(
    () => (tab === 'differential' ? staticJointEfforts(joints, fk.jacobian, wrench) : null),
    [tab, joints, fk, wrench]
  );

  async function refresh(signal) {
    try { setSimulations(await request('', { signal })); setApiError(''); }
    catch (error) { if (error.name !== 'AbortError') setApiError('API indisponível. ' + error.message); }
  }
  useEffect(() => {
    const controller = new AbortController();
    request('', { signal: controller.signal }).then(data => {
      setSimulations(data);
      setApiError('');
    }).catch(error => {
      if (error.name !== 'AbortError') setApiError('API indisponível. ' + error.message);
    });

    const params = new URLSearchParams(window.location.search);
    const slug = params.get('share');
    if (slug) {
      fetch(`${API}/share/${slug}`, { signal: controller.signal })
        .then(res => res.json())
        .then(resData => {
          if (resData.data && validateJoints(resData.data).length === 0) {
            const normalized = normalizeJoints(resData.data);
            const nextFk = forwardKinematics(normalized);
            const loadedTool = resData.toolConfig || null;
            const nextToolType = loadedTool?.toolType || 'gripper';
            const nextOffset = loadedTool?.toolOffsetRpy || [...DEFAULT_TOOL_OFFSET_RPY];
            const nextGrip = loadedTool?.toolGrip ?? 50;
            const nextToolFrame = computeToolFrame(nextFk.total, nextOffset, nextToolType);
            setJoints(normalized);
            setToolType(nextToolType);
            setToolOffsetRpy(nextOffset);
            setToolGrip(nextGrip);
            setModelStyle(normalizeModelStyle(loadedTool?.modelStyle));
            setSolution(null);
            setCollisionAlert(null);
            setTarget({ position: nextFk.positions.at(-1), rpy: nextToolFrame.orientation.rpy });
            if (Math.abs(nextFk.positions.at(-1)[2]) > 1e-4) setMode('position');
            setIsPlaying(false);
            setPlayTime(0);
            setWaypoints(
              Array.isArray(resData.waypoints) && resData.waypoints.length > 0
                ? syncWaypointsToJoints(resData.waypoints, normalized)
                : defaultWaypointsForJoints(normalized)
            );
            setViewKey(k => k + 1);
            setMessage(`Modelo compartilhado "${resData.name}" carregado.`);
          }
        })
        .catch(() => {});
    }

    return () => controller.abort();
  }, []);

  function applyEnvironment(nextEnv) {
    setEnvironment(nextEnv);
    setWorkspaceCloud(null);
    setSolution(null);
    const nextCol = detectSelfCollision(joints, fk, toolType, nextEnv);
    if (nextCol.colliding) {
      setCollisionAlert(nextCol);
      setMessage(nextCol.message);
    } else {
      setCollisionAlert(null);
      if (collisionAlert) setMessage('');
    }
  }

  function placeObstacleInFront() {
    const size = Math.max(0.1, Number(environment.obstacle.radius) || 0.8);
    const offset = size + 0.8;
    const pos = [0, 1, 2].map(k => Math.round((currentEnd[k] + toolFrame.approach[k] * offset) * 100) / 100);
    applyEnvironment({
      ...environment,
      obstacle: {
        ...environment.obstacle,
        enabled: true,
        position: pos,
      },
    });
  }

  function handleObstacleDrag(nextPos) {
    applyEnvironment({
      ...environment,
      obstacle: {
        ...environment.obstacle,
        position: nextPos,
      },
    });
  }

  function solveInverse(targetToSolve = target, modeToUse = mode, jointsToUse = joints, maxIterations = undefined) {
    setIsPlaying(false);
    const adjustedTarget = {
      ...targetToSolve,
      rpy: flangeRpyFromToolTarget(targetToSolve.rpy, toolOffsetRpy),
    };
    const res = inverseKinematics(jointsToUse, adjustedTarget, {
      mode: modeToUse,
      length,
      damping,
      ...(maxIterations ? { maxIterations } : {}),
      toolType,
      environment,
    });
    if (res.converged && validateJoints(res.joints).length === 0) {
      const normalized = normalizeJoints(res.joints).map(j => ({ ...j, speed: 0 }));
      setJoints(normalized);
      setSolution({ ...res, joints: normalized, applied: true });
      setCollisionAlert(null);
      setMessage('');
    } else {
      setSolution({ ...res, applied: false });
      setMessage('');
    }
    return res;
  }

  function syncTargetAndModeForJoints(nextJoints, forceMode = null, customTool = null) {
    const nextFk = forwardKinematics(nextJoints);
    const tType = customTool?.toolType || toolType;
    const tOffset = customTool?.toolOffsetRpy || toolOffsetRpy;
    const nextTool = computeToolFrame(nextFk.total, tOffset, tType);
    setTarget({ position: nextFk.positions.at(-1), rpy: nextTool.orientation.rpy });
    setSolution(null);
    if (forceMode) {
      setMode(forceMode);
    } else if (Math.abs(nextFk.positions.at(-1)[2]) > 1e-4) {
      setMode('position');
    }
  }

  function commitJoints(next) {
    const errors = validateJoints(next);
    if (errors.length) { setMessage(errors.join(' ')); return false; }
    setJoints(normalizeJoints(next)); setSolution(null); setMessage(''); setCollisionAlert(null);
    return true;
  }
  function updateJoint(id, updates, options = {}) {
    const isRangeEdit = updates.min !== undefined || updates.max !== undefined;
    const isValMotion = updates.val !== undefined && !isRangeEdit
      && updates.a === undefined && updates.alpha === undefined
      && updates.d === undefined && updates.theta === undefined;
    let clampedInfo = null;
    const next = joints.map((j, idx) => {
      if (j.id !== id) return j;
      const nextMin = updates.min !== undefined ? updates.min : j.min;
      const nextMax = updates.max !== undefined ? updates.max : j.max;
      const candidateVal = updates.val !== undefined ? updates.val : j.val;
      let nextVal = candidateVal;
      if (isRangeEdit && Number.isFinite(nextMin) && Number.isFinite(nextMax) && nextMin < nextMax) {
        nextVal = Math.max(nextMin, Math.min(nextMax, candidateVal));
        if (nextVal !== candidateVal) {
          const unit = j.type === 'R' ? '°' : ' u';
          clampedInfo = `J${idx + 1} (${fmt(nextVal, j.type === 'R' ? 1 : 2)}${unit})`;
        }
      }
      return { ...j, ...updates, val: nextVal };
    });

    if (isValMotion) {
      const errors = validateJoints(next);
      if (errors.length) { setMessage(errors.join(' ')); return false; }
      const swept = sweepJointsCollisionFree(joints, normalizeJoints(next), toolType, environment);
      if (swept.blocked) {
        const unchanged = swept.joints.every((j, idx) => Math.abs(j.val - joints[idx].val) < 1e-9);
        commitJoints(swept.joints);
        setCollisionAlert(swept.collision);
        setMessage(`${swept.collision.message} Movimento bloqueado na fronteira segura.`);
        return options.continuous ? true : !unchanged;
      }
      return commitJoints(next);
    }

    if (!commitJoints(next)) return false;

    const normalized = normalizeJoints(next);
    const col = detectSelfCollision(normalized, null, toolType, environment);
    if (col.colliding) {
      setCollisionAlert(col);
      setMessage(col.message);
    }

    if (isRangeEdit) {
      setWorkspaceCloud(null);
      setWaypoints(prev => syncWaypointsToJoints(prev, normalized));
      const nextBounds = computeCartesianBounds(normalized, null, 'position', environment);
      const nextTarget = {
        ...target,
        position: target.position.map((v, axis) => Math.max(nextBounds.min[axis], Math.min(nextBounds.max[axis], v))),
      };
      setTarget(nextTarget);
      if (options.fromInverse && autoSolveIk) {
        solveInverse(nextTarget, mode, normalized);
      } else if (clampedInfo) {
        setMessage(`Intervalo atualizado; posição ajustada para ${clampedInfo}.`);
      }
    }
    return true;
  }
  function changeSetting(setter, value) { setter(value); setSolution(null); }
  function applyOrientationPreset(nextRpy) {
    const nextTarget = { ...target, rpy: nextRpy };
    if (mode !== 'pose') setMode('pose');
    setTarget(nextTarget);
    solveInverse(nextTarget, 'pose');
  }
  function changeTarget(key, index, value) {
    const nextTarget = { ...target, [key]: target[key].map((v, i) => i === index ? value : v) };
    const nextMode = key === 'rpy' && mode !== 'pose' ? 'pose' : mode;
    if (nextMode !== mode) setMode(nextMode);
    setTarget(nextTarget);
    if (autoSolveIk) {
      solveInverse(nextTarget, nextMode);
    } else {
      setSolution(null);
    }
  }
  function slideTarget(key, index, value) {
    const fixedTarget = { ...target, [key]: target[key].map((v, i) => (i === index ? value : v)) };
    const nextMode = key === 'rpy' && mode !== 'pose' ? 'pose' : mode;
    if (nextMode !== mode) setMode(nextMode);
    setTarget(fixedTarget);
    const res = solveInverse(fixedTarget, nextMode);
    if (res.converged || key !== 'position') return;

    // Never apply an unconverged least-squares step that drifts fixed transverse coordinates;
    // binary-search strictly along axis `index` while keeping fixed axes exact.
    let lo = currentEnd[index];
    let hi = value;
    let bestRes = null;

    for (let stepIdx = 0; stepIdx < 8; stepIdx++) {
      const mid = (lo + hi) / 2;
      const candTarget = {
        ...fixedTarget,
        position: fixedTarget.position.map((v, i) => (i === index ? mid : v)),
        rpy: flangeRpyFromToolTarget(fixedTarget.rpy, toolOffsetRpy),
      };
      const attempt = inverseKinematics(bestRes ? bestRes.joints : joints, candTarget, {
        mode: nextMode,
        length,
        damping,
        maxIterations: 45,
        toolType,
        environment,
      });
      if (
        attempt.converged
        && validateJoints(attempt.joints).length === 0
        && !detectSelfCollision(attempt.joints, null, toolType, environment).colliding
      ) {
        bestRes = attempt;
        lo = mid;
      } else {
        hi = mid;
      }
    }

    if (bestRes) {
      const normalized = normalizeJoints(bestRes.joints).map(j => ({ ...j, speed: 0 }));
      const achievedPos = forwardKinematics(normalized).positions.at(-1);
      setJoints(normalized);
      setTarget(prev => ({
        ...prev,
        position: fixedTarget.position.map((v, i) => (i === index ? achievedPos[index] : v)),
      }));
      setCollisionAlert(null);
      setSolution({ ...bestRes, joints: normalized, applied: true });
      setMessage('');
    } else {
      setTarget(prev => ({
        ...prev,
        position: fixedTarget.position.map((v, i) => (i === index ? currentEnd[index] : v)),
      }));
    }
  }
  function handleGizmoDragStart() {
    jointsRef.current = joints;
    setIsGizmoDragging(true);
  }
  function handleGizmoDragEnd() {
    setIsGizmoDragging(false);
    const activeFk = forwardKinematics(jointsRef.current);
    const activeEnd = activeFk.positions.at(-1).map(v => Math.round(v * 1000) / 1000);
    const activeToolRpy = computeToolFrame(activeFk.total, toolOffsetRpy, toolType).orientation.rpy.map(v => Math.round(v * 10) / 10);
    setTarget(prev => ({
      ...prev,
      position: activeEnd,
      rpy: activeToolRpy,
    }));
  }
  function handleGizmoDrag({ position, rpy }) {
    const activeJoints = jointsRef.current;
    const activeFk = forwardKinematics(activeJoints);
    const activeEnd = activeFk.positions.at(-1);

    if (position) {
      const requestedPos = position.map((v, i) => (mode === 'xy' && i === 2 ? activeEnd[2] : v));
      const bounded = solveBoundedTarget(
        activeJoints,
        { position: requestedPos, rpy: target.rpy },
        { mode, length, damping, maxIterations: 40, toolType, toolOffsetRpy, environment }
      );
      const nextPos = bounded.position.map((v, i) =>
        (mode === 'xy' && i === 2 ? activeEnd[2] : Math.round(v * 1000) / 1000)
      );
      if (!bounded.blocked) {
        jointsRef.current = bounded.joints;
        setJoints(bounded.joints);
        setCollisionAlert(null);
      }
      setSolution({ ...bounded, applied: bounded.converged && !bounded.blocked });
      setMessage('');
      setTarget(prev => ({ ...prev, position: nextPos }));
      return { position: nextPos };
    }

    if (rpy) {
      if (mode !== 'pose') setMode('pose');
      const bounded = solveBoundedTarget(
        activeJoints,
        { position: activeEnd, rpy },
        { mode: 'pose', length, damping, maxIterations: 40, toolType, toolOffsetRpy, environment }
      );
      const nextPos = bounded.position.map(v => Math.round(v * 1000) / 1000);
      const nextRpy = bounded.rpy.map(v => Math.round(v * 10) / 10);
      if (!bounded.blocked) {
        jointsRef.current = bounded.joints;
        setJoints(bounded.joints);
        setCollisionAlert(null);
      }
      setSolution({ ...bounded, applied: bounded.converged && !bounded.blocked });
      setMessage('');
      setTarget({ position: nextPos, rpy: nextRpy });
      return { position: nextPos, rpy: nextRpy };
    }
    return undefined;
  }
  async function persist(action) {
    setBusy(true); setApiError('');
    try { await action(); }
    catch (error) { setApiError(error.message); }
    finally { setBusy(false); }
  }
  function step() {
    const result = integrateJoints(joints, joints.map(j => j.speed), dt, environment, toolType);
    commitJoints(result.joints);
    if (result.collision) {
      setCollisionAlert(result.collision);
      setMessage(`Passo interrompido para evitar colisão: ${result.collision.message}`);
    } else {
      setMessage(result.limited.length ? `Limite atingido nas juntas ${result.limited.join(', ')}.` : `Passo de ${dt} s aplicado.`);
    }
  }

  function applyPreset(presetId) {
    const p = PRESETS[presetId];
    if (!p) return;
    const presetOffset = Array.isArray(p.toolOffsetRpy) ? [...p.toolOffsetRpy] : [...DEFAULT_TOOL_OFFSET_RPY];
    commitJoints(p.joints);
    setDefaultToolOffset(presetOffset);
    setToolOffsetRpy(presetOffset);
    setModelTarget('global');
    setModelStyle(prev => normalizeModelStyle({ ...prev, perJoint: undefined }));
    syncTargetAndModeForJoints(p.joints, presetId === 'rpr' ? 'xy' : 'position', { toolOffsetRpy: presetOffset });
    setViewKey(k => k + 1);
    setWorkspaceCloud(null);
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints(defaultWaypointsForJoints(p.joints));
    setMessage(`Robô ${p.name} carregado.`);
  }

  const structuralJointsKey = joints
    .map(j => `${j.id}:${j.type}:${j.a}:${j.alpha}:${j.d}:${j.theta}:${j.min}:${j.max}`)
    .join('|');
  const structuralJoints = useMemo(
    () => structuralJointsKey.split('|').filter(Boolean).map(entry => {
      const [id, type, a, alpha, d, theta, min, max] = entry.split(':');
      return {
        id: Number(id),
        type,
        a: Number(a),
        alpha: Number(alpha),
        d: Number(d),
        theta: Number(theta),
        val: Number(type === 'P' ? d : theta),
        min: Number(min),
        max: Number(max),
        speed: 0,
      };
    }),
    [structuralJointsKey]
  );

  const trajectoryData = useMemo(() => {
    return sampleTrajectoryPath(waypoints, structuralJoints, undefined, interpType, environment);
  }, [waypoints, structuralJoints, interpType, environment]);

  function seekTrajectory(time) {
    if (waypoints.length < 2) {
      setPlayTime(time);
      return;
    }
    const evaluated = evaluateTrajectory(waypoints, time, interpType, joints, environment);
    if (!evaluated) {
      setPlayTime(time);
      return;
    }
    const candidate = joints.map((j, i) => {
      const rawVel = evaluated.velocities[i] !== undefined ? evaluated.velocities[i] : 0;
      return {
        ...j,
        val: evaluated.q[i] !== undefined ? evaluated.q[i] : j.val,
        speed: j.type === 'R' ? rawVel * DEG : rawVel,
      };
    });
    const swept = sweepJointsCollisionFree(joints, candidate, toolType, environment);
    if (swept.blocked) {
      setJoints(swept.joints.map(j => ({ ...j, speed: 0 })));
      setCollisionAlert(swept.collision);
      setMessage(`Trajetória interrompida: ${swept.collision.message}`);
      return;
    }
    setPlayTime(time);
    setJoints(candidate);
    setCollisionAlert(null);
    setMessage('');
  }

  useEffect(() => {
    if (!isPlaying) return;
    let lastTime = performance.now();
    let animId;

    const tick = (now) => {
      const deltaSec = (now - lastTime) / 1000;
      lastTime = now;

      const total = trajectoryData.totalDuration;
      if (total <= 0) {
        setIsPlaying(false);
        return;
      }
      let next = playTimeRef.current + deltaSec * playbackSpeed;
      let stoppedAtEnd = false;
      if (next >= total) {
        if (isLooping) {
          next = next % total;
        } else {
          next = total;
          stoppedAtEnd = true;
        }
      }
      const evaluated = evaluateTrajectory(waypoints, next, interpType, jointsRef.current, environmentRef.current);
      if (evaluated) {
        const currentJoints = jointsRef.current;
        const candidate = currentJoints.map((j, i) => {
          const rawVel = stoppedAtEnd ? 0 : (evaluated.velocities[i] !== undefined ? evaluated.velocities[i] : 0);
          return {
            ...j,
            val: evaluated.q[i] !== undefined ? evaluated.q[i] : j.val,
            speed: j.type === 'R' ? rawVel * DEG : rawVel,
          };
        });
        const swept = sweepJointsCollisionFree(currentJoints, candidate, toolTypeRef.current, environmentRef.current);
        if (swept.blocked) {
          setJoints(swept.joints.map(j => ({ ...j, speed: 0 })));
          setIsPlaying(false);
          setCollisionAlert(swept.collision);
          setMessage(`Trajetória pausada por colisão: ${swept.collision.message}`);
          return;
        }
        jointsRef.current = candidate;
        setJoints(candidate);
      }
      playTimeRef.current = next;
      setPlayTime(next);
      if (stoppedAtEnd) {
        setIsPlaying(false);
        return;
      }

      animId = requestAnimationFrame(tick);
    };

    animId = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animId);
  }, [isPlaying, playbackSpeed, isLooping, trajectoryData.totalDuration, waypoints, interpType]);

  function addWaypointFromPose(qVec, name, durationVal = 2.0) {
    const count = waypoints.length;
    const wpDuration = count === 0 ? 0 : Math.max(0.1, Number(durationVal) || 2.0);
    const newWp = {
      id: Date.now() + Math.floor(Math.random() * 1000),
      name: name || `Ponto ${count + 1}`,
      q: qVec.map((val, i) => {
        const j = joints[i];
        return j ? Math.max(j.min, Math.min(j.max, val)) : val;
      }),
      duration: wpDuration,
    };
    setWaypoints(prev => [...prev, newWp]);
    setMessage(`Ponto "${newWp.name}" salvo na trajetória.`);
  }

  function addCurrentWaypoint() {
    addWaypointFromPose(joints.map(j => j.val), `Ponto ${waypoints.length + 1}`, 2.0);
  }

  function removeWaypoint(id) {
    if (waypoints.length <= 1) return;
    setWaypoints(prev => prev.filter(w => w.id !== id));
  }

  function moveToWaypoint(wp) {
    setIsPlaying(false);
    const candidate = joints.map((j, i) => ({
      ...j,
      val: wp.q[i] !== undefined ? wp.q[i] : j.val,
      speed: 0,
    }));
    const swept = sweepJointsCollisionFree(joints, candidate, toolType, environment);
    if (swept.blocked) {
      commitJoints(swept.joints);
      setCollisionAlert(swept.collision);
      setMessage(`Movimento bloqueado antes da colisão: ${swept.collision.message}`);
      return;
    }
    commitJoints(candidate);
    setMessage(`Manipulador posicionado em "${wp.name}".`);
  }

  function updateWaypointDuration(id, duration) {
    setWaypoints(prev => prev.map(w => w.id === id ? { ...w, duration: Math.max(0.1, duration) } : w));
  }

  function computeWorkspace() {
    setComputingCloud(true);
    setTimeout(() => {
      try {
        const cloud = generateWorkspaceCloud(joints, sampleCount, environment);
        setWorkspaceCloud(cloud);
        setShowWorkspace(true);
        setMessage(`Nuvem de trabalho calculada com ${cloud.count} amostras.`);
      } catch (err) {
        setMessage('Falha ao calcular espaço de trabalho: ' + err.message);
      } finally {
        setComputingCloud(false);
      }
    }, 20);
  }

  const generatedCode = useMemo(() => {
    if (tab !== 'export') return '';
    const robotName = `Manipulador_${joints.map(j => j.type).join('')}_${joints.length}DOF`;
    const toolConfig = { toolType, toolOffsetRpy, toolGrip, modelStyle };
    if (codeTarget === 'cpp') return generateCpp(joints, robotName, toolConfig);
    if (codeTarget === 'python') return generatePython(joints, robotName, toolConfig);
    if (codeTarget === 'urdf') return exportToURDF(joints, robotName, toolConfig);
    return generateRos2(joints, robotName, toolConfig);
  }, [tab, joints, codeTarget, toolType, toolOffsetRpy, toolGrip, modelStyle]);

  function downloadUrdf() {
    const urdfContent = exportToURDF(joints, `Manipulador_${joints.map(j => j.type).join('')}`, { toolType, toolOffsetRpy, toolGrip, modelStyle });
    const blob = new Blob([urdfContent], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `robot_${joints.map(j => j.type).join('')}.urdf`;
    a.click();
    URL.revokeObjectURL(url);
    setMessage('Arquivo .urdf baixado com sucesso.');
  }

  function handleUrdfUpload(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const parsed = parseURDF(ev.target.result);
        if (commitJoints(parsed)) {
          const normalized = normalizeJoints(parsed);
          const importedTool = parsed.toolConfig || null;
          if (importedTool) {
            setToolType(importedTool.toolType || 'gripper');
            setToolOffsetRpy(importedTool.toolOffsetRpy || [0, 0, 0]);
            if (importedTool.modelStyle) setModelStyle(normalizeModelStyle(importedTool.modelStyle));
          }
          syncTargetAndModeForJoints(normalized, 'position', importedTool);
          setIsPlaying(false);
          setPlayTime(0);
          setWaypoints(defaultWaypointsForJoints(normalized));
          setViewKey(k => k + 1);
          setMessage(`Robô importado via URDF com sucesso (${parsed.length} juntas).`);
        }
      } catch (err) {
        setMessage('Erro ao importar URDF: ' + err.message);
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  }

  function handleStlUpload(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const geom = parseStlBuffer(ev.target.result);
        setCustomStlGeometry(geom);
        setStlFileName(file.name);
        setMessage(`Malha STL "${file.name}" carregada no efetuador.`);
      } catch (err) {
        setMessage('Erro ao processar arquivo STL: ' + err.message);
      }
    };
    reader.readAsArrayBuffer(file);
    e.target.value = '';
  }

  function removeStlMesh() {
    if (customStlGeometry) customStlGeometry.dispose();
    setCustomStlGeometry(null);
    setStlFileName('');
    setMessage('Malha STL removida. Efetuador padrão restaurado.');
  }

  function copyCode() {
    navigator.clipboard.writeText(generatedCode).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  }

  async function handleShare() {
    setShareLoading(true);
    try {
      const res = await request('/share', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: simName.trim() || `Robô ${joints.map(j => j.type).join('')}`,
          data: joints,
          waypoints,
          toolConfig: { toolType, toolOffsetRpy, toolGrip, modelStyle },
        }),
      });
      const fullUrl = `${window.location.origin}${window.location.pathname}?share=${res.slug}`;
      setShareUrl(fullUrl);
      setMessage('Link público gerado com sucesso!');
      await refresh();
    } catch (err) {
      setApiError('Erro ao compartilhar: ' + err.message);
    } finally {
      setShareLoading(false);
    }
  }

  function copyShareUrl() {
    if (!shareUrl) return;
    navigator.clipboard.writeText(shareUrl).then(() => {
      setShareCopied(true);
      setTimeout(() => setShareCopied(false), 2000);
    });
  }

  const targetDistance = mode === 'xy'
    ? Math.hypot(target.position[0] - currentEnd[0], target.position[1] - currentEnd[1])
    : Math.hypot(target.position[0] - currentEnd[0], target.position[1] - currentEnd[1], target.position[2] - currentEnd[2]);

  const toolAngularErrorDeg = useMemo(() => {
    if (tab !== 'inverse') return 0;
    const rCurr = toolFrame.orientation.rotation;
    const d = rotationFromRPY(target.rpy).elements;
    const trace =
      rCurr[0][0] * d[0] + rCurr[1][0] * d[1] + rCurr[2][0] * d[2]
      + rCurr[0][1] * d[4] + rCurr[1][1] * d[5] + rCurr[2][1] * d[6]
      + rCurr[0][2] * d[8] + rCurr[1][2] * d[9] + rCurr[2][2] * d[10];
    const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
    return Math.acos(cosTheta) / DEG;
  }, [tab, toolFrame, target.rpy]);

  const numeric = useCallback(
    (fieldLabel, value, onCommit, extra = {}) => (
      <NumberField label={fieldLabel} value={value} onCommit={onCommit} onError={setMessage} {...extra} />
    ),
    []
  );

  function resetDefaultRobot() {
    commitJoints(INITIAL_PRESET.joints);
    setDefaultToolOffset([...INITIAL_TOOL_OFFSET]);
    setToolOffsetRpy([...INITIAL_TOOL_OFFSET]);
    syncTargetAndModeForJoints(INITIAL_PRESET.joints, 'position', { toolOffsetRpy: INITIAL_TOOL_OFFSET });
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints(defaultWaypointsForJoints(INITIAL_PRESET.joints));
    setViewKey(k => k + 1);
  }

  const handleSelectTab = useCallback((id) => {
    if (activePanel === id) {
      setActivePanel(null);
      return;
    }
    if (tab !== id) setMessage('');
    setTab(id);
    setActivePanel(id);
  }, [activePanel, tab]);

  const handleTogglePanel = useCallback((id) => {
    setActivePanel(prev => (prev === id ? null : id));
  }, []);

  const handleClosePanel = useCallback(() => {
    setActivePanel(null);
  }, []);

  const panelContent = activePanel === 'projects' ? (
    <ProjectsTab
      joints={joints}
      applyPreset={applyPreset}
      handleUrdfUpload={handleUrdfUpload}
      toolType={toolType}
      toolOffsetRpy={toolOffsetRpy}
      toolGrip={toolGrip}
      modelStyle={modelStyle}
      resetDefaultRobot={resetDefaultRobot}
      commitJoints={commitJoints}
      setIsPlaying={setIsPlaying}
      setPlayTime={setPlayTime}
      setWaypoints={setWaypoints}
      setViewKey={setViewKey}
      simName={simName}
      setSimName={setSimName}
      busy={busy}
      persist={persist}
      request={request}
      waypoints={waypoints}
      refresh={refresh}
      setMessage={setMessage}
      handleShare={handleShare}
      shareLoading={shareLoading}
      shareUrl={shareUrl}
      copyShareUrl={copyShareUrl}
      shareCopied={shareCopied}
      apiError={apiError}
      simulations={simulations}
      setToolTypeLoaded={setToolType}
      setToolOffsetRpyLoaded={setToolOffsetRpy}
      setToolGripLoaded={setToolGrip}
      setModelStyleLoaded={setModelStyle}
      syncTargetAndModeForJoints={syncTargetAndModeForJoints}
    />
  ) : activePanel === 'model' ? (
    <ModelTab
      joints={joints}
      resetDefaultRobot={resetDefaultRobot}
      updateJoint={updateJoint}
      commitJoints={commitJoints}
      setIsPlaying={setIsPlaying}
      setWaypoints={setWaypoints}
      environment={environment}
      applyEnvironment={applyEnvironment}
      placeObstacleInFront={placeObstacleInFront}
      showObstacleGizmo={showObstacleGizmo}
      setShowObstacleGizmo={setShowObstacleGizmo}
      numeric={numeric}
    />
  ) : activePanel === 'forward' ? (
    <ForwardTab joints={joints} fk={fk} />
  ) : activePanel === 'differential' ? (
    <DifferentialTab
      joints={joints}
      fk={fk}
      mode={mode}
      setMode={setMode}
      length={length}
      setLength={setLength}
      damping={damping}
      setDamping={setDamping}
      onChangeSetting={changeSetting}
      diagnostics={diagnostics}
      velocity={velocity}
      differential={differential}
      statics={statics}
      dt={dt}
      setDt={setDt}
      step={step}
      desiredVelocity={desiredVelocity}
      setDesiredVelocity={setDesiredVelocity}
      payloadMass={payloadMass}
      setPayloadMass={setPayloadMass}
      gravity={gravity}
      setGravity={setGravity}
      wrench={wrench}
      setWrench={setWrench}
      updateJoint={updateJoint}
      commitJoints={commitJoints}
      setMessage={setMessage}
      numeric={numeric}
    />
  ) : activePanel === 'inverse' ? (
    <InverseTab
      joints={joints}
      fk={fk}
      currentEnd={currentEnd}
      mode={mode}
      setMode={setMode}
      length={length}
      setLength={setLength}
      damping={damping}
      setDamping={setDamping}
      onChangeSetting={changeSetting}
      toolType={toolType}
      setToolType={setToolType}
      toolFrame={toolFrame}
      showGizmo={showGizmo}
      setShowGizmo={setShowGizmo}
      gizmoMode={gizmoMode}
      setGizmoMode={setGizmoMode}
      cartesianBounds={cartesianBounds}
      target={target}
      setTarget={setTarget}
      targetDistance={targetDistance}
      toolAngularErrorDeg={toolAngularErrorDeg}
      autoSolveIk={autoSolveIk}
      setAutoSolveIk={setAutoSolveIk}
      ikWaypointDuration={ikWaypointDuration}
      setIkWaypointDuration={setIkWaypointDuration}
      solution={solution}
      setSolution={setSolution}
      message={message}
      solveInverse={solveInverse}
      changeTarget={changeTarget}
      slideTarget={slideTarget}
      applyOrientationPreset={applyOrientationPreset}
      updateJoint={updateJoint}
      addWaypointFromPose={addWaypointFromPose}
      waypoints={waypoints}
      trajectoryData={trajectoryData}
      isPlaying={isPlaying}
      setIsPlaying={setIsPlaying}
      playTime={playTime}
      setPlayTime={setPlayTime}
      moveToWaypoint={moveToWaypoint}
      removeWaypoint={removeWaypoint}
      environment={environment}
      applyEnvironment={applyEnvironment}
      placeObstacleInFront={placeObstacleInFront}
      showObstacleGizmo={showObstacleGizmo}
      setShowObstacleGizmo={setShowObstacleGizmo}
      numeric={numeric}
    />
  ) : activePanel === 'trajectory' ? (
    <TrajectoryTab
      joints={joints}
      waypoints={waypoints}
      trajectoryData={trajectoryData}
      playTime={playTime}
      setPlayTime={setPlayTime}
      isPlaying={isPlaying}
      setIsPlaying={setIsPlaying}
      playbackSpeed={playbackSpeed}
      setPlaybackSpeed={setPlaybackSpeed}
      isLooping={isLooping}
      setIsLooping={setIsLooping}
      interpType={interpType}
      setInterpType={setInterpType}
      angleUnit={angleUnit}
      setAngleUnit={setAngleUnit}
      showTrajectory={showTrajectory}
      setShowTrajectory={setShowTrajectory}
      seekTrajectory={seekTrajectory}
      addCurrentWaypoint={addCurrentWaypoint}
      moveToWaypoint={moveToWaypoint}
      removeWaypoint={removeWaypoint}
      updateWaypointDuration={updateWaypointDuration}
    />
  ) : activePanel === 'workspace' ? (
    <WorkspaceTab
      sampleCount={sampleCount}
      setSampleCount={setSampleCount}
      computingCloud={computingCloud}
      computeWorkspace={computeWorkspace}
      workspaceCloud={workspaceCloud}
      showWorkspace={showWorkspace}
      setShowWorkspace={setShowWorkspace}
    />
  ) : activePanel === 'export' ? (
    <ExportTab
      codeTarget={codeTarget}
      setCodeTarget={setCodeTarget}
      generatedCode={generatedCode}
      downloadUrdf={downloadUrdf}
      copyCode={copyCode}
      copied={copied}
    />
  ) : activePanel === 'report' ? (
    <ReportTab
      joints={joints}
      fk={fk}
      toolType={toolType}
      toolOffsetRpy={toolOffsetRpy}
      toolGrip={toolGrip}
      toolFrame={toolFrame}
      diagnostics={diagnostics}
      reportMeta={reportMeta}
      setReportMeta={setReportMeta}
    />
  ) : null;

  return <main className="app-container app-fullscreen">
    <div className="canvas-container">
      <RobotScene
        joints={joints}
        showFrames={showFrames}
        viewKey={viewKey}
        workspaceCloud={workspaceCloud}
        showWorkspace={showWorkspace}
        trajectoryPathPoints={trajectoryData.pathPoints}
        trajectoryWaypointCoords={trajectoryData.waypointCoords}
        showTrajectory={showTrajectory}
        customStlGeometry={customStlGeometry}
        toolType={toolType}
        toolOffsetRpy={toolOffsetRpy}
        toolGrip={toolGrip}
        modelStyle={modelStyle}
        ikTarget={tab === 'inverse' ? [target.position[0], target.position[1], mode === 'xy' ? currentEnd[2] : target.position[2]] : null}
        ikRpy={target.rpy}
        gizmoEnabled={tab === 'inverse' && showGizmo}
        gizmoMode={gizmoMode}
        taskMode={mode}
        onGizmoDrag={handleGizmoDrag}
        onGizmoDragStart={handleGizmoDragStart}
        onGizmoDragEnd={handleGizmoDragEnd}
        collisionPoint={activeCollision?.point || null}
        environment={environment}
        obstacleGizmoEnabled={environment.obstacle.enabled && showObstacleGizmo}
        onDragObstacle={handleObstacleDrag}
        onSelectObstacle={() => setShowObstacleGizmo(true)}
      />
      <SceneToolbar
        joints={joints}
        tab={tab}
        activePanel={activePanel}
        onSelectTab={handleSelectTab}
        onTogglePanel={handleTogglePanel}
        onClosePanel={handleClosePanel}
        message={message}
        activeCollision={activeCollision}
        showFrames={showFrames}
        setShowFrames={setShowFrames}
        environment={environment}
        applyEnvironment={applyEnvironment}
        showGizmo={showGizmo}
        setShowGizmo={setShowGizmo}
        hasTrajectory={trajectoryData.pathPoints.length > 0}
        showTrajectory={showTrajectory}
        setShowTrajectory={setShowTrajectory}
        hasWorkspace={Boolean(workspaceCloud)}
        showWorkspace={showWorkspace}
        setShowWorkspace={setShowWorkspace}
        customStlGeometry={customStlGeometry}
        stlFileName={stlFileName}
        handleStlUpload={handleStlUpload}
        removeStlMesh={removeStlMesh}
        onFrameView={() => setViewKey(k => k + 1)}
        toolType={toolType}
        setToolType={setToolType}
        toolOffsetRpy={toolOffsetRpy}
        setToolOffsetRpy={setToolOffsetRpy}
        defaultToolOffset={defaultToolOffset}
        toolGrip={toolGrip}
        setToolGrip={setToolGrip}
        toolFrame={toolFrame}
        modelStyle={modelStyle}
        setModelStyle={setModelStyle}
        modelTarget={modelTarget}
        setModelTarget={setModelTarget}
        numeric={numeric}
      >
        {panelContent}
      </SceneToolbar>
      <div className="scene-readout">
        {activeCollision && <div className="collision-readout-pill">Bloqueio de colisão: {activeCollision.message}</div>}
        <span>Efetuador ({TOOL_TYPES[toolType].shortLabel}) · RPY [{fmt(toolFrame.orientation.rpy[0], 1)}°, {fmt(toolFrame.orientation.rpy[1], 1)}°, {fmt(toolFrame.orientation.rpy[2], 1)}°]</span>
        <output data-testid="end-position">X {fmt(fk.positions.at(-1)[0], 2)} · Y {fmt(fk.positions.at(-1)[1], 2)} · Z {fmt(fk.positions.at(-1)[2], 2)} u</output>
      </div>
    </div>
  </main>;
}

export default App;
