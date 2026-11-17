import { useEffect, useMemo, useRef, useState } from 'react';
import { Activity, Check, CheckCheck, Copy, Crosshair, Download, Focus, Pause, Play, Plus, Printer, Repeat, RotateCcw, Save, Settings2, Share2, Square, Trash2, Upload } from 'lucide-react';
import RobotScene from './RobotScene';
import TrajectoryCharts from './TrajectoryCharts';
import { Button, Checkbox, FileUploadButton, MatrixView, Navbar, Notice, NumberField, SegmentedTabs, Select, Slider } from './ui';
import { DEG, DEFAULT_ENVIRONMENT, DEFAULT_JOINTS, TASK_ROWS, TOOL_LIST, TOOL_TYPES, computeToolFrame, detectSelfCollision, flangeRpyFromToolTarget, forwardKinematics, integrateJoints, inverseKinematics, inverseVelocity, jacobianDiagnostics, matrixRows, multiplyVector, normalizeJoints, rotationFromRPY, sweepJointsCollisionFree, validateJoints } from './kinematics';
import { PRESETS, PRESET_LIST } from './presets';
import { computeCartesianBounds, generateWorkspaceCloud } from './workspace';
import { generateCpp, generatePython, generateRos2 } from './codeGenerator';
import { defaultWaypointsForJoints, evaluateTrajectory, sampleTrajectoryPath, syncWaypointsToJoints } from './trajectory';
import { openAcademicReport } from './reportGenerator';
import { exportToURDF, parseURDF } from './urdfParser';
import { parseStlBuffer } from './cadLoader';
import './App.css';

const API = (import.meta.env.VITE_API_URL || '/api').replace(/\/$/, '');
const fmt = (n, digits = 4) => !Number.isFinite(n) ? '∞' : (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits);
const vectorText = (v) => v.map(n => fmt(n)).join(', ');
const velocityLabels = ['vx (u/s)', 'vy (u/s)', 'vz (u/s)', 'ωx (rad/s)', 'ωy (rad/s)', 'ωz (rad/s)'];

async function request(path = '', options = {}) {
  const res = await fetch(`${API}/simulations${path}`, options);
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || `Falha na API (${res.status}).`);
  return data;
}

function App() {
  const [joints, setJoints] = useState(() => DEFAULT_JOINTS.map(j => ({ ...j })));
  const jointsRef = useRef(joints);
  useEffect(() => { jointsRef.current = joints; }, [joints]);
  const [tab, setTab] = useState('model');
  const [simulations, setSimulations] = useState([]);
  const [simName, setSimName] = useState('');
  const [busy, setBusy] = useState(false);
  const [apiError, setApiError] = useState('');
  const [message, setMessage] = useState('');
  const [collisionAlert, setCollisionAlert] = useState(null);
  const [mode, setMode] = useState('xy');
  const [length, setLength] = useState(5);
  const [damping, setDamping] = useState(0.02);
  const [dt, setDt] = useState(0.1);
  const [desiredVelocity, setDesiredVelocity] = useState([0, 0, 0, 0, 0, 0]);
  const [target, setTarget] = useState({ position: [5, 0, 0], rpy: [0, 0, 0] });
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
  const [waypoints, setWaypoints] = useState(() => defaultWaypointsForJoints(DEFAULT_JOINTS));
  const [isPlaying, setIsPlaying] = useState(false);
  const [playTime, setPlayTime] = useState(0);
  const playTimeRef = useRef(playTime);
  useEffect(() => { playTimeRef.current = playTime; }, [playTime]);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);
  const [isLooping, setIsLooping] = useState(false);
  const [interpType, setInterpType] = useState('quintic');
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [shareUrl, setShareUrl] = useState('');
  const [shareLoading, setShareLoading] = useState(false);
  const [shareCopied, setShareCopied] = useState(false);
  const [customStlGeometry, setCustomStlGeometry] = useState(null);
  const [stlFileName, setStlFileName] = useState('');
  const [toolType, setToolType] = useState('gripper');
  const toolTypeRef = useRef(toolType);
  useEffect(() => { toolTypeRef.current = toolType; }, [toolType]);
  const [toolOffsetRpy, setToolOffsetRpy] = useState([0, 0, 0]);
  const [toolGrip, setToolGrip] = useState(50);
  const [environment, setEnvironment] = useState(() => ({
    ...DEFAULT_ENVIRONMENT,
    obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
  }));
  const environmentRef = useRef(environment);
  useEffect(() => { environmentRef.current = environment; }, [environment]);
  const [showGizmo, setShowGizmo] = useState(true);
  const [gizmoMode, setGizmoMode] = useState('translate');
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
  const cartesianBounds = useMemo(
    () => (tab === 'inverse'
      ? computeCartesianBounds(joints, currentEnd, mode, environment)
      : computeCartesianBounds(joints, null, 'position', environment)),
    [tab, joints, currentEnd, mode, environment]
  );
  const diagnostics = useMemo(() => jacobianDiagnostics(joints, fk.jacobian, mode, length), [joints, fk, mode, length]);
  const velocity = multiplyVector(fk.jacobian, joints.map(j => j.speed));
  const differential = inverseVelocity(joints, fk.jacobian, desiredVelocity, { mode, length, damping });

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
            const nextOffset = loadedTool?.toolOffsetRpy || [0, 0, 0];
            const nextGrip = loadedTool?.toolGrip ?? 50;
            const nextToolFrame = computeToolFrame(nextFk.total, nextOffset, nextToolType);
            setJoints(normalized);
            setToolType(nextToolType);
            setToolOffsetRpy(nextOffset);
            setToolGrip(nextGrip);
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

  function solveInverse(targetToSolve = target, modeToUse = mode, jointsToUse = joints) {
    setIsPlaying(false);
    const adjustedTarget = {
      ...targetToSolve,
      rpy: flangeRpyFromToolTarget(targetToSolve.rpy, toolOffsetRpy),
    };
    const res = inverseKinematics(jointsToUse, adjustedTarget, { mode: modeToUse, length, damping, toolType, environment });
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
  function handleGizmoDrag({ position, rpy }) {
    if (position) {
      const globalBounds = computeCartesianBounds(joints, null, 'position', environment);
      const clampedPos = position.map((v, i) => {
        if (mode === 'xy' && i === 2) return currentEnd[2];
        return Math.round(Math.max(globalBounds.min[i], Math.min(globalBounds.max[i], v)) * 1000) / 1000;
      });
      const changedAxes = [0, 1, 2].filter(i => Math.abs(clampedPos[i] - target.position[i]) > 1e-4);
      const nextTarget = { ...target, position: clampedPos };
      setTarget(nextTarget);
      const res = solveInverse(nextTarget, mode);
      if (res.converged) return clampedPos;

      if (changedAxes.length === 1) {
        const axisIdx = changedAxes[0];
        let lo = currentEnd[axisIdx];
        let hi = clampedPos[axisIdx];
        let bestRes = null;
        for (let s = 0; s < 6; s++) {
          const mid = (lo + hi) / 2;
          const candTarget = {
            ...nextTarget,
            position: nextTarget.position.map((v, i) => (i === axisIdx ? mid : v)),
            rpy: flangeRpyFromToolTarget(nextTarget.rpy, toolOffsetRpy),
          };
          const attempt = inverseKinematics(bestRes ? bestRes.joints : joints, candTarget, {
            mode,
            length,
            damping,
            maxIterations: 40,
            toolType,
            environment,
          });
          if (attempt.converged && validateJoints(attempt.joints).length === 0 && !detectSelfCollision(attempt.joints, null, toolType, environment).colliding) {
            bestRes = attempt;
            lo = mid;
          } else {
            hi = mid;
          }
        }
        if (bestRes) {
          const normalized = normalizeJoints(bestRes.joints).map(j => ({ ...j, speed: 0 }));
          const achievedPos = forwardKinematics(normalized).positions.at(-1);
          const lockedPos = nextTarget.position.map((v, i) => (i === axisIdx ? achievedPos[axisIdx] : v));
          setJoints(normalized);
          setTarget({ ...nextTarget, position: lockedPos });
          setCollisionAlert(null);
          setSolution({ ...bestRes, joints: normalized, applied: true });
          return lockedPos;
        }
        setTarget(prev => ({ ...prev, position: currentEnd }));
        return currentEnd;
      }

      if (res.joints && validateJoints(res.joints).length === 0) {
        const normalized = normalizeJoints(res.joints).map(j => ({ ...j, speed: 0 }));
        if (!detectSelfCollision(normalized, null, toolType, environment).colliding) {
          const achievedPos = forwardKinematics(normalized).positions.at(-1);
          setJoints(normalized);
          setTarget({ ...nextTarget, position: achievedPos });
          setCollisionAlert(null);
          setSolution({ ...res, joints: normalized, applied: false });
          return achievedPos;
        }
      }
      return currentEnd;
    }

    if (rpy) {
      const nextTarget = { ...target, rpy };
      if (mode !== 'pose') setMode('pose');
      setTarget(nextTarget);
      solveInverse(nextTarget, 'pose');
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
    commitJoints(p.joints);
    syncTargetAndModeForJoints(p.joints, presetId === 'rpr' ? 'xy' : 'position');
    setViewKey(k => k + 1);
    setWorkspaceCloud(null);
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints(defaultWaypointsForJoints(p.joints));
    setMessage(`Robô ${p.name} carregado.`);
  }

  const trajectoryData = useMemo(() => {
    return sampleTrajectoryPath(waypoints, joints, undefined, interpType, environment);
  }, [waypoints, joints, interpType, environment]);

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
    const robotName = `Manipulador_${joints.map(j => j.type).join('')}_${joints.length}DOF`;
    const toolConfig = { toolType, toolOffsetRpy, toolGrip };
    if (codeTarget === 'cpp') return generateCpp(joints, robotName, toolConfig);
    if (codeTarget === 'python') return generatePython(joints, robotName, toolConfig);
    if (codeTarget === 'urdf') return exportToURDF(joints, robotName, toolConfig);
    return generateRos2(joints, robotName, toolConfig);
  }, [joints, codeTarget, toolType, toolOffsetRpy, toolGrip]);

  function downloadUrdf() {
    const urdfContent = exportToURDF(joints, `Manipulador_${joints.map(j => j.type).join('')}`, { toolType, toolOffsetRpy, toolGrip });
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
          toolConfig: { toolType, toolOffsetRpy, toolGrip },
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
    const rCurr = toolFrame.orientation.rotation;
    const d = rotationFromRPY(target.rpy).elements;
    const trace =
      rCurr[0][0] * d[0] + rCurr[1][0] * d[1] + rCurr[2][0] * d[2]
      + rCurr[0][1] * d[4] + rCurr[1][1] * d[5] + rCurr[2][1] * d[6]
      + rCurr[0][2] * d[8] + rCurr[1][2] * d[9] + rCurr[2][2] * d[10];
    const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
    return Math.acos(cosTheta) / DEG;
  }, [toolFrame, target.rpy]);

  const numeric = (label, value, onCommit, extra = {}) => <NumberField label={label} value={value} onCommit={onCommit} onError={setMessage} {...extra} />;
  const environmentSection = (
    <section>
      <div className="section-title">
        <h2>Ambiente e Colisão Externa (Piso / Obstáculo 3D)</h2>
        <Button
          icon
          title="Restaurar ambiente padrão"
          aria-label="Restaurar ambiente padrão"
          disabled={environment.floorEnabled && Math.abs(environment.floorZ - DEFAULT_ENVIRONMENT.floorZ) < 1e-6 && !environment.obstacle.enabled}
          onClick={() => applyEnvironment({
            ...DEFAULT_ENVIRONMENT,
            obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
          })}
        >
          <RotateCcw size={16} />
        </Button>
      </div>
      <div className="env-card">
        <div className="env-row">
          <Checkbox
            checked={environment.floorEnabled}
            onChange={floorEnabled => applyEnvironment({ ...environment, floorEnabled })}
          >
            Piso / Bancada ativo
          </Checkbox>
          {numeric('Nível do piso Z (u)', environment.floorZ, z => applyEnvironment({ ...environment, floorZ: z }), { min: -20, max: 20 })}
        </div>
        <div className="env-obstacle-header">
          <Checkbox
            checked={environment.obstacle.enabled}
            onChange={enabled => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, enabled } })}
          >
            Obstáculo 3D ativo
          </Checkbox>
          <Button onClick={placeObstacleInFront}>
            Posicionar à frente do efetuador
          </Button>
        </div>
        <SegmentedTabs
          ariaLabel="Forma do obstáculo 3D"
          activeId={environment.obstacle.shape}
          onChange={shape => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, shape } })}
          items={[
            { id: 'sphere', label: 'Esfera 3D' },
            { id: 'box', label: 'Caixa 3D (Cubo)' },
          ]}
        />
        <div className="env-obstacle-grid">
          {['X', 'Y', 'Z'].map((axis, idx) => (
            <div key={axis}>
              {numeric(`Obstáculo ${axis} (u)`, environment.obstacle.position[idx], val => applyEnvironment({
                ...environment,
                obstacle: {
                  ...environment.obstacle,
                  position: environment.obstacle.position.map((p, i) => (i === idx ? val : p)),
                },
              }), { min: -30, max: 30 })}
            </div>
          ))}
          <div>
            {numeric('Raio / Tamanho (u)', environment.obstacle.radius, radius => applyEnvironment({
              ...environment,
              obstacle: { ...environment.obstacle, radius },
            }), { min: 0.1, max: 10 })}
          </div>
        </div>
      </div>
    </section>
  );
  const taskSettings = <div className="task-settings">
    <label>Tarefa<Select
      ariaLabel="Tarefa"
      value={mode}
      onChange={val => changeSetting(setMode, val)}
      options={[
        { value: 'xy', label: 'Posição XY' },
        { value: 'position', label: 'Posição XYZ' },
        { value: 'pose', label: 'Pose XYZ + orientação' },
      ]}
    /></label>
    {numeric('Comprimento de referência (u)', length, v => changeSetting(setLength, v), { min: 0.001, max: 1e4 })}
    {numeric('Amortecimento λ', damping, v => changeSetting(setDamping, v), { min: 0, max: 10 })}
  </div>;

  return <main className="app-container">
    <aside className="sidebar">
      <header className="title-container"><Settings2 size={25} /><div><h1>Cinemática DH</h1><span className="muted">Manipulador {joints.map(j => j.type).join('')} · {joints.length} juntas</span></div></header>
      <Navbar
        ariaLabel="Análises"
        activeId={tab}
        onChange={id => { setTab(id); setMessage(''); }}
        items={[
          { id: 'model', label: 'Modelo' },
          { id: 'forward', label: 'Direta' },
          { id: 'differential', label: 'Diferencial' },
          { id: 'inverse', label: 'Inversa' },
          { id: 'trajectory', label: 'Trajetória' },
          { id: 'workspace', label: 'Alcance 3D' },
          { id: 'export', label: 'Código' },
          { id: 'report', label: 'Relatório' },
        ]}
      />
      {message && <Notice role="status" variant={activeCollision ? 'error' : undefined}>{message}</Notice>}

      {tab === 'model' && <>
        <section>
          <div className="section-title"><h2>Presets industriais</h2></div>
          <div className="preset-row">
            <Select
              ariaLabel="Selecionar preset industrial"
              value=""
              placeholder="Carregar arquitetura consagrada..."
              onChange={val => { if (val) applyPreset(val); }}
              options={PRESET_LIST.map(p => ({ value: p.id, label: `${p.name} (${p.dof} DOF)` }))}
            />
          </div>
          <div className="model-actions">
            <FileUploadButton accept=".urdf,.xml" onChange={handleUrdfUpload} inputAriaLabel="Upload de arquivo URDF">
              <Upload size={14} /> Importar robô (URDF)
            </FileUploadButton>
          </div>
        </section>
        <section>
          <div className="section-title">
            <h2>Ferramenta do efetuador (TCP)</h2>
            <Button
              icon
              title="Restaurar ferramenta padrão"
              aria-label="Restaurar ferramenta padrão"
              disabled={toolType === 'gripper' && toolOffsetRpy.every(v => Math.abs(v) < 1e-6) && toolGrip === 50}
              onClick={() => { setToolType('gripper'); setToolOffsetRpy([0, 0, 0]); setToolGrip(50); }}
            >
              <RotateCcw size={16} />
            </Button>
          </div>
          <SegmentedTabs
            ariaLabel="Tipo de ferramenta"
            activeId={toolType}
            onChange={setToolType}
            items={TOOL_LIST.map(t => ({ id: t.id, label: t.shortLabel }))}
          />
          <p className="muted">{TOOL_TYPES[toolType].description}</p>
          <div className="ik-slider-card tool-config-card">
            <div className="ik-slider-header">
              <div className="ik-slider-meta">
                <strong>{TOOL_TYPES[toolType].actuationLabel}</strong>
                <span className="muted">TCP: X {fmt(toolFrame.tcpPosition[0], 2)} · Y {fmt(toolFrame.tcpPosition[1], 2)} · Z {fmt(toolFrame.tcpPosition[2], 2)} u</span>
              </div>
              <output className="ik-joint-reading">{fmt(toolGrip, 0)}%</output>
            </div>
            <Slider
              ariaLabel={TOOL_TYPES[toolType].actuationLabel}
              ariaValueText={`${fmt(toolGrip, 0)}%`}
              min={0}
              max={100}
              step={1}
              value={toolGrip}
              onChange={val => setToolGrip(Math.round(val))}
            />
          </div>
          <div className="tool-offset-grid">
            {['Roll', 'Pitch', 'Yaw'].map((axis, idx) => (
              <div key={axis}>
                {numeric(`Offset ${axis} (°)`, toolOffsetRpy[idx], v => setToolOffsetRpy(prev => prev.map((cur, i) => (i === idx ? v : cur))), { min: -180, max: 180 })}
              </div>
            ))}
          </div>
          <div className="ik-summary">
            <div className="ik-summary-row">
              <span>Orientação do TCP (RPY)</span>
              <output>{fmt(toolFrame.orientation.rpy[0], 1)}° · {fmt(toolFrame.orientation.rpy[1], 1)}° · {fmt(toolFrame.orientation.rpy[2], 1)}°</output>
            </div>
          </div>
        </section>
        <section><div className="section-title"><h2>Atuadores</h2><Button icon title="Restaurar RPR das aulas" aria-label="Restaurar RPR das aulas" onClick={() => { commitJoints(DEFAULT_JOINTS); syncTargetAndModeForJoints(DEFAULT_JOINTS, 'xy'); setIsPlaying(false); setPlayTime(0); setWaypoints(defaultWaypointsForJoints(DEFAULT_JOINTS)); setViewKey(k => k + 1); }}><RotateCcw size={17} /></Button></div>
          {joints.map((joint, index) => {
            const unit = joint.type === 'R' ? '°' : 'u';
            return <div className="actuator" key={joint.id}>
              <div className="control-header">
                <div className="joint-label-group">
                  <strong>Junta {index + 1} · {joint.type}</strong>
                  <span className={`joint-type-badge ${joint.type === 'P' ? 'is-prismatic' : 'is-rotational'}`}>
                    {joint.type === 'P' ? 'Prismática' : 'Rotativa'} · Δ {fmt(joint.max - joint.min, joint.type === 'R' ? 0 : 1)} {unit}
                  </span>
                </div>
                {numeric(`q${index + 1} (${unit})`, joint.val, val => updateJoint(joint.id, { val }), { min: joint.min, max: joint.max })}
              </div>
              <Slider
                ariaLabel={`Atuador ${index + 1}`}
                ariaValueText={`${fmt(joint.val, joint.type === 'R' ? 1 : 2)} ${unit}`}
                min={joint.min}
                max={joint.max}
                value={joint.val}
                onChange={val => updateJoint(joint.id, { val }, { continuous: true })}
              />
              <div className="limit-fields">
                {numeric(`Mín. ${index + 1} (${unit})`, joint.min, min => updateJoint(joint.id, { min }))}
                {numeric(`Máx. ${index + 1} (${unit})`, joint.max, max => updateJoint(joint.id, { max }))}
              </div>
            </div>;
          })}
        </section>
        {environmentSection}
        <section><div className="section-title"><h2>Denavit-Hartenberg padrão</h2><Button icon title="Adicionar junta" aria-label="Adicionar junta" disabled={joints.length >= 24} onClick={() => {
          const nextJoints = [...joints, { id: Math.max(...joints.map(j => j.id)) + 1, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 0, min: -180, max: 180, speed: 0 }];
          if (commitJoints(nextJoints)) {
            setIsPlaying(false);
            setWaypoints(prev => syncWaypointsToJoints(prev, normalizeJoints(nextJoints)));
          }
        }}><Plus size={18} /></Button></div>
          <div className="table-scroll"><table className="dh-table"><thead><tr><th>i</th><th>Tipo</th><th>a (u)</th><th>α (°)</th><th>θ (°)</th><th>d (u)</th><th></th></tr></thead>
            <tbody>{joints.map((j, i) => <tr key={j.id}><td>{i + 1}</td><td><Select
              compact
              ariaLabel={`Tipo ${i + 1}`}
              value={j.type}
              options={[{ value: 'R', label: 'R' }, { value: 'P', label: 'P' }]}
              onChange={nextType => {
                const nextJoints = joints.map(item => item.id === j.id ? { ...item, type: nextType, val: 0, speed: 0, min: nextType === 'R' ? -180 : 0, max: nextType === 'R' ? 180 : 10 } : item);
                if (commitJoints(nextJoints)) {
                  setIsPlaying(false);
                  setWaypoints(prev => syncWaypointsToJoints(prev, normalizeJoints(nextJoints)));
                }
              }}
            /></td>
              {['a', 'alpha', 'theta', 'd'].map(key => <td key={key}>{numeric(`${key}${i + 1}`, (key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? j.val : j[key], v => updateJoint(j.id, { [(key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? 'val' : key]: v }), { compact: true })}</td>)}
              <td><Button icon variant="danger" title={`Remover junta ${i + 1}`} aria-label={`Remover junta ${i + 1}`} disabled={joints.length === 1} onClick={() => {
                const nextJoints = joints.filter(item => item.id !== j.id);
                if (commitJoints(nextJoints)) {
                  setIsPlaying(false);
                  const normalized = normalizeJoints(nextJoints);
                  setWaypoints(prev => syncWaypointsToJoints(
                    prev.map(wp => ({ ...wp, q: Array.isArray(wp.q) ? wp.q.filter((_, idx) => idx !== i) : [] })),
                    normalized
                  ));
                }
              }}><Trash2 size={15} /></Button></td></tr>)}</tbody>
          </table></div>
        </section>
        <section><h2>Simulações</h2><form className="save-row" onSubmit={e => { e.preventDefault(); void persist(async () => { await request('', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: simName.trim(), data: joints, waypoints, toolConfig: { toolType, toolOffsetRpy, toolGrip } }) }); setSimName(''); await refresh(); setMessage('Simulação salva.'); }); }}>
          <input className="ui-input" aria-label="Nome da simulação" placeholder="Nome da simulação" value={simName} maxLength={120} onChange={e => setSimName(e.target.value)} required />
          <Button type="submit" variant="primary" disabled={busy || !simName.trim()}><Save size={16} />Salvar</Button></form>
          <div className="action-row">
            <Button disabled={busy || shareLoading} onClick={handleShare}>
              <Share2 size={16} /> Compartilhar link público
            </Button>
          </div>
          {shareUrl && <div className="share-box">
            <strong>Link público de compartilhamento:</strong>
            <div className="share-url-row">
              <input className="ui-input" readOnly value={shareUrl} aria-label="Link de compartilhamento" />
              <Button onClick={copyShareUrl}>
                {shareCopied ? <CheckCheck size={14} /> : <Copy size={14} />}
                {shareCopied ? 'Copiado' : 'Copiar'}
              </Button>
            </div>
          </div>}
          {apiError && <Notice variant="error" role="alert">{apiError}<Button onClick={() => void refresh()}>Reconectar</Button></Notice>}
          {!apiError && simulations.length === 0 && <p className="muted">Nenhuma simulação salva.</p>}
          <ul className="saved-list">{simulations.map(sim => <li key={sim.id}><span>{sim.name}</span><Button icon disabled={busy} title={`Carregar ${sim.name}`} aria-label={`Carregar ${sim.name}`} onClick={() => void persist(async () => {
            const data = await request(`/${sim.id}`);
            const normalized = normalizeJoints(data.data);
            const loadedTool = data.toolConfig || null;
            if (loadedTool) {
              setToolType(loadedTool.toolType || 'gripper');
              setToolOffsetRpy(loadedTool.toolOffsetRpy || [0, 0, 0]);
              setToolGrip(loadedTool.toolGrip ?? 50);
            }
            commitJoints(normalized);
            syncTargetAndModeForJoints(normalized, null, loadedTool);
            setIsPlaying(false);
            setPlayTime(0);
            setWaypoints(
              Array.isArray(data.waypoints) && data.waypoints.length > 0
                ? syncWaypointsToJoints(data.waypoints, normalized)
                : defaultWaypointsForJoints(normalized)
            );
            setViewKey(k => k + 1);
            setMessage('Simulação carregada.');
          })}><Download size={16} /></Button><Button icon variant="danger" disabled={busy} title={`Apagar ${sim.name}`} aria-label={`Apagar ${sim.name}`} onClick={() => { if (window.confirm(`Apagar "${sim.name}"?`)) void persist(async () => { await request(`/${sim.id}`, { method: 'DELETE' }); await refresh(); }); }}><Trash2 size={16} /></Button></li>)}</ul>
        </section>
      </>}

      {tab === 'forward' && <>
        <section><h2>Coordenadas globais (u)</h2><MatrixView label="Origens dos referenciais" headers={['i', 'X', 'Y', 'Z']} values={fk.positions.map((p, i) => [i, ...p])} /></section>
        <section><h2>Transformações homogêneas</h2><div className="formula">Aᵢ = Rz(θᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)</div>
          <MatrixView label="T⁰ₙ = A₁ · … · Aₙ" values={matrixRows(fk.total)} />
          {fk.links.map((matrix, i) => <details key={joints[i].id}><summary>A{i + 1} · referencial {i} → {i + 1}</summary><MatrixView label={`A${i + 1}`} values={matrixRows(matrix)} /><MatrixView label={`T⁰${i + 1}`} values={matrixRows(fk.transforms[i + 1])} /></details>)}
        </section>
        <section><h2>Orientação</h2><MatrixView label="R⁰ₙ" values={fk.orientation.rotation} />
          <dl className="readings"><dt>RPY ZYX (°)</dt><dd>{vectorText(fk.orientation.rpy)}</dd><dt>Composição</dt><dd>Rz(yaw) Ry(pitch) Rx(roll)</dd><dt>Eixo de rotação</dt><dd>{fk.orientation.axis ? vectorText(fk.orientation.axis) : 'Indeterminado (rotação nula)'}</dd><dt>Ângulo (°)</dt><dd>{fmt(fk.orientation.angle)}</dd><dt>det(R)</dt><dd>{fmt(fk.orientation.determinant, 6)}</dd><dt>‖RᵀR − I‖</dt><dd>{fk.orientation.orthogonality.toExponential(2)}</dd></dl>
          {fk.orientation.gimbalLock && <Notice as="p">Singularidade RPY: roll e yaw não são únicos.</Notice>}
        </section>
      </>}

      {tab === 'differential' && <>
        <section><h2>Jacobiano geométrico</h2>{taskSettings}
          <MatrixView label="J · linhas [vx, vy, vz, ωx, ωy, ωz]" headers={joints.map((_, i) => `q̇${i + 1}`)} values={fk.jacobian} />
          <dl className="readings"><dt>Posto / máximo algébrico</dt><dd>{diagnostics.rank} / {diagnostics.maxRank}</dd><dt>Dimensões da tarefa</dt><dd>{diagnostics.dimensions}</dd><dt>Condição normalizada</dt><dd>{fmt(diagnostics.condition)}</dd><dt>Valores singulares normalizados</dt><dd>{vectorText(diagnostics.singularValues)}</dd><dt>det(J da tarefa)</dt><dd>{diagnostics.determinant === null ? 'Não se aplica (retangular)' : fmt(diagnostics.determinant, 6)}</dd></dl>
          <Notice as="p" variant={diagnostics.fullTask ? 'success' : 'warning'}>
            {diagnostics.fullTask ? 'Tarefa localmente controlável.' : diagnostics.deficient ? 'Posto reduzido: singularidade ou restrição estrutural da tarefa.' : 'Graus de liberdade insuficientes para controlar toda a tarefa.'}
            {diagnostics.condition > 1000 && Number.isFinite(diagnostics.condition) ? ' Próximo de perda de posto.' : ''}
          </Notice>
        </section>
        <section><h2>Velocidades articulares</h2><div className="field-grid">{joints.map((j, i) => <div key={j.id}>{numeric(`q̇${i + 1} (${j.type === 'R' ? 'rad/s' : 'u/s'})`, j.speed, speed => updateJoint(j.id, { speed }))}</div>)}</div>
          <MatrixView label="v = J q̇ · base global" values={velocity.map(v => [v])} />
          <div className="action-row">{numeric('Δt (s)', dt, setDt, { min: 0.001, max: 10 })}<Button onClick={step}><Activity size={16} />Aplicar passo</Button></div>
        </section>
        <section><h2>Inversa diferencial</h2><div className="field-grid">{TASK_ROWS[mode].map(i => <div key={i}>{numeric(velocityLabels[i], desiredVelocity[i], value => setDesiredVelocity(prev => prev.map((v, k) => k === i ? value : v)))}</div>)}</div>
          <MatrixView label="q̇ calculado · R: rad/s; P: u/s" values={differential.rates.map(v => [v])} />
          <MatrixView label="Velocidade obtida [u/s; rad/s]" values={differential.achieved.map(v => [v])} />
          <div className="metric">Resíduo normalizado: <output>{fmt(differential.error, 6)}</output></div>
          <Button onClick={() => { if (commitJoints(joints.map((j, i) => ({ ...j, speed: differential.rates[i] })))) setMessage('Velocidades articulares atualizadas.'); }}><Check size={16} />Usar velocidades</Button>
        </section>
      </>}

      {tab === 'inverse' && <>
        <section>
          <h2>Alvo cartesiano e orientação</h2>
          {taskSettings}
          <div className="tool-inline-block">
            <span className="tool-inline-label">Ferramenta na ponta ({TOOL_TYPES[toolType].shortLabel})</span>
            <SegmentedTabs
              ariaLabel="Ferramenta ativa na inversa"
              activeId={toolType}
              onChange={setToolType}
              items={TOOL_LIST.map(t => ({ id: t.id, label: t.shortLabel }))}
            />
          </div>
          <div className="gizmo-control-bar">
            <Checkbox checked={showGizmo} onChange={setShowGizmo}>
              Gizmo 3D interativo
            </Checkbox>
            {showGizmo && (
              <SegmentedTabs
                ariaLabel="Modo do gizmo 3D"
                activeId={gizmoMode}
                onChange={nextGizmoMode => {
                  setGizmoMode(nextGizmoMode);
                  if (nextGizmoMode === 'rotate' && mode !== 'pose') {
                    setMode('pose');
                    setSolution(null);
                  }
                }}
                items={[
                  { id: 'translate', label: 'Mover no 3D (XYZ)' },
                  { id: 'rotate', label: 'Girar no 3D (RPY)' },
                ]}
              />
            )}
          </div>
          <div className="ik-slider-list">
            {['X', 'Y', 'Z'].map((axis, i) => {
              if (mode === 'xy' && i >= 2) return null;
              const lo = cartesianBounds.min[i];
              const hi = cartesianBounds.max[i];
              const isFixed = cartesianBounds.span[i] < 1e-5;
              const roundedLo = Math.ceil(lo * 1000) / 1000;
              const roundedHi = Math.floor(hi * 1000) / 1000;
              const sliderMin = isFixed ? Number((lo - 1).toFixed(3)) : (roundedHi > roundedLo ? roundedLo : Number(lo.toFixed(3)));
              const sliderMax = isFixed ? Number((hi + 1).toFixed(3)) : (roundedHi > roundedLo ? roundedHi : Number(hi.toFixed(3)));
              const sliderVal = isFixed ? sliderMin : Math.max(sliderMin, Math.min(sliderMax, target.position[i]));
              return (
                <div className="ik-slider-card" key={axis}>
                  <div className="ik-slider-header">
                    <div className="ik-slider-meta">
                      <strong>Alvo {axis} (u)</strong>
                      <span className="muted">Alcance: [{fmt(lo, 2)}, {fmt(hi, 2)}] u</span>
                    </div>
                    {numeric(`Alvo ${axis} (u)`, target.position[i], v => changeTarget('position', i, v), { compact: true })}
                  </div>
                  <Slider
                    ariaLabel={`Deslizar Alvo ${axis} (u)`}
                    ariaValueText={`${fmt(sliderVal, 2)} u`}
                    min={sliderMin}
                    max={sliderMax}
                    disabled={isFixed}
                    value={sliderVal}
                    onChange={raw => {
                      const nextVal = Math.max(sliderMin, Math.min(sliderMax, Math.round(raw * 1000) / 1000));
                      slideTarget('position', i, nextVal);
                    }}
                  />
                </div>
              );
            })}
          </div>
          <div className="ik-orientation-header">
            <div className="ik-orientation-title">
              <strong>Orientação da ferramenta (RPY °)</strong>
              <span className="muted">
                {mode === 'pose' ? 'Ativa em conjunto com a posição XYZ' : 'Ajustar orientação ativa automaticamente o modo Pose XYZ + orientação'}
              </span>
            </div>
            <div className="ik-orientation-presets">
              <Button onClick={() => applyOrientationPreset([0, 0, 0])}>Frente 0°</Button>
              <Button onClick={() => applyOrientationPreset([0, -90, 0])}>Vertical -90°</Button>
              <Button onClick={() => applyOrientationPreset([0, 90, 0])}>Vertical +90°</Button>
              <Button onClick={() => applyOrientationPreset([0, 0, 90])}>Lateral 90°</Button>
              <Button onClick={() => applyOrientationPreset(toolFrame.orientation.rpy)}>Igualar atual</Button>
            </div>
          </div>
          <div className="ik-slider-list">
            {['Roll', 'Pitch', 'Yaw'].map((axis, i) => {
              const sliderVal = Math.max(-180, Math.min(180, target.rpy[i]));
              return (
                <div className="ik-slider-card" key={axis}>
                  <div className="ik-slider-header">
                    <div className="ik-slider-meta">
                      <strong>{axis} alvo (°)</strong>
                      <span className="muted">Alcance: [-180.00, 180.00] °</span>
                    </div>
                    {numeric(`${axis} alvo (°)`, target.rpy[i], v => changeTarget('rpy', i, v), { compact: true })}
                  </div>
                  <Slider
                    ariaLabel={`Deslizar ${axis} alvo (°)`}
                    ariaValueText={`${fmt(sliderVal, 1)} °`}
                    min={-180}
                    max={180}
                    value={sliderVal}
                    onChange={raw => {
                      const nextVal = Math.max(-180, Math.min(180, Math.round(raw * 10) / 10));
                      slideTarget('rpy', i, nextVal);
                    }}
                  />
                </div>
              );
            })}
          </div>
          <div className="ik-summary">
            <div className="ik-summary-row">
              <span>Efetuador atual</span>
              <output>X {fmt(currentEnd[0], 2)} · Y {fmt(currentEnd[1], 2)} · Z {fmt(currentEnd[2], 2)} u</output>
            </div>
            <div className="ik-summary-row">
              <span>Orientação da ferramenta</span>
              <output>{fmt(toolFrame.orientation.rpy[0], 1)}° · {fmt(toolFrame.orientation.rpy[1], 1)}° · {fmt(toolFrame.orientation.rpy[2], 1)}°</output>
            </div>
            <div className="ik-summary-row">
              <span>Distância até o alvo</span>
              <output>{fmt(targetDistance, 3)} u</output>
            </div>
            <div className="ik-summary-row">
              <span>Erro angular da ferramenta</span>
              <output>{fmt(toolAngularErrorDeg, 2)} °</output>
            </div>
          </div>
          <Checkbox checked={autoSolveIk} onChange={checked => {
            setAutoSolveIk(checked);
            if (checked) solveInverse(target, mode);
          }}>
            Resolver e aplicar automaticamente ao editar alvo
          </Checkbox>
          <div className="action-row">
            <Button onClick={() => { setTarget({ position: fk.positions.at(-1), rpy: toolFrame.orientation.rpy }); setSolution(null); }}>
              <Crosshair size={16} />Capturar pose atual
            </Button>
            <Button variant="primary" onClick={() => solveInverse()}>
              <Check size={16} />Resolver
            </Button>
          </div>
          <div className="ik-save-row">
            {numeric('Δt (s)', ikWaypointDuration, setIkWaypointDuration, { min: 0.1, max: 60 })}
            <Button onClick={() => addWaypointFromPose(joints.map(j => j.val), `Ponto ${waypoints.length + 1} (IK)`, ikWaypointDuration)}>
              <Plus size={16} />Salvar ponto na trajetória
            </Button>
          </div>
        </section>

        <section>
          <h2>Limites de controle das juntas (R / P)</h2>
          <p className="muted">Restrinja o intervalo permitido de cada junta prismática (u) ou rotativa (°); os limites dos sliders cartesianos e o solver IK obedecerão exatamente a essa faixa.</p>
          <div className="ik-joint-limits-list">
            {joints.map((joint, index) => {
              const unit = joint.type === 'R' ? '°' : 'u';
              const decimals = joint.type === 'R' ? 1 : 2;
              const atLimit = Math.abs(joint.val - joint.min) < 1e-4 || Math.abs(joint.val - joint.max) < 1e-4;
              return (
                <div className={`ik-joint-limit-card${atLimit ? ' at-limit' : ''}`} key={joint.id}>
                  <div className="ik-joint-limit-header">
                    <div className="joint-label-group">
                      <strong>Junta {index + 1}</strong>
                      <span className={`joint-type-badge ${joint.type === 'P' ? 'is-prismatic' : 'is-rotational'}`}>
                        {joint.type === 'P' ? 'Prismática (P)' : 'Rotativa (R)'}
                      </span>
                      {atLimit && <span className="joint-limit-flag">No limite</span>}
                    </div>
                    <output className="ik-joint-reading">q{index + 1} = {fmt(joint.val, decimals)} {unit}</output>
                  </div>
                  <Slider
                    ariaLabel={`Posição J${index + 1} (${unit})`}
                    ariaValueText={`${fmt(joint.val, decimals)} ${unit}`}
                    min={joint.min}
                    max={joint.max}
                    value={joint.val}
                    onChange={val => updateJoint(joint.id, { val }, { continuous: true })}
                  />
                  <div className="limit-fields ik-limit-fields">
                    {numeric(`Limite mín. J${index + 1} (${unit})`, joint.min, min => updateJoint(joint.id, { min }, { fromInverse: true }))}
                    {numeric(`Limite máx. J${index + 1} (${unit})`, joint.max, max => updateJoint(joint.id, { max }, { fromInverse: true }))}
                  </div>
                </div>
              );
            })}
          </div>
        </section>
        {environmentSection}

        {solution && <section>
          <h2>Resultado</h2>
          <Notice as="p" variant={solution.converged ? 'success' : 'warning'} role={message ? undefined : 'status'}>
            {solution.converged ? 'Solução convergiu dentro dos limites e foi aplicada ao manipulador.' : 'Não convergiu. Possíveis causas: alvo incompatível, limites ou mínimo local.'}
          </Notice>
          <dl className="readings">
            <dt>Iterações totais</dt><dd>{solution.iterations}</dd>
            <dt>Resíduo normalizado</dt><dd>{fmt(solution.error, 7)}</dd>
            <dt>Erro de posição XYZ (u)</dt><dd>{vectorText(solution.residual.slice(0, 3))}</dd>
            {mode === 'pose' && <><dt>Erro angular XYZ (rad)</dt><dd>{vectorText(solution.residual.slice(3))}</dd></>}
          </dl>
          <MatrixView label="Configuração encontrada · R: °; P: u" values={solution.joints.map(j => [j.val])} />
          <div className="action-row">
            <Button variant="primary" disabled={!solution.converged} onClick={() => addWaypointFromPose(solution.joints.map(j => j.val), `Ponto ${waypoints.length + 1} (IK)`, ikWaypointDuration)}>
              <Plus size={16} />Salvar este ponto na trajetória
            </Button>
          </div>
        </section>}

        <section>
          <div className="section-title">
            <h2>Waypoints salvos na trajetória ({waypoints.length})</h2>
            <div className="btn-row">
              <Button disabled={waypoints.length < 2} onClick={() => {
                if (isPlaying) {
                  setIsPlaying(false);
                } else {
                  if (playTime >= trajectoryData.totalDuration) setPlayTime(0);
                  setIsPlaying(true);
                }
              }}>
                {isPlaying ? <Pause size={14} /> : <Play size={14} />}
                {isPlaying ? 'Pausar' : 'Reproduzir'}
              </Button>
            </div>
          </div>
          <ul className="waypoint-list">
            {waypoints.map((wp, idx) => (
              <li key={wp.id} className="waypoint-item">
                <div className="waypoint-info">
                  <strong>{idx + 1}. {wp.name}</strong>
                  <span className="waypoint-coords">
                    XYZ: [{trajectoryData.waypointCoords[idx] ? trajectoryData.waypointCoords[idx].map(v => fmt(v, 2)).join(', ') : '-'}] · Δt: {fmt(wp.duration, 1)} s
                  </span>
                </div>
                <div className="item-actions">
                  <Button icon title="Ir para waypoint" aria-label={`Ir para ${wp.name}`} onClick={() => moveToWaypoint(wp)}>
                    <Focus size={15} />
                  </Button>
                  <Button icon variant="danger" title="Remover waypoint" aria-label={`Remover ${wp.name}`} disabled={waypoints.length <= 1} onClick={() => removeWaypoint(wp.id)}>
                    <Trash2 size={15} />
                  </Button>
                </div>
              </li>
            ))}
          </ul>
        </section>
      </>}

      {tab === 'trajectory' && <>
        <section>
          <h2>Sequenciador de Trajetórias</h2>
          <p className="muted">Interpolação contínua de movimentos articulares via splines quínticas e cúbicas sem descontinuidades nos atuadores.</p>

          <div className="timeline-panel">
            <div className="timeline-slider-row">
              <span>{fmt(playTime, 2)} s</span>
              <Slider
                className="timeline-slider"
                ariaLabel="Timeline da trajetória"
                min={0}
                max={Math.max(0.01, trajectoryData.totalDuration)}
                value={playTime}
                onChange={seekTrajectory}
              />
              <span>{fmt(trajectoryData.totalDuration, 2)} s</span>
            </div>

            <div className="timeline-controls">
              <div className="timeline-btns">
                <Button
                  variant="primary"
                  aria-label={isPlaying ? "Pausar reprodução" : "Iniciar reprodução"}
                  onClick={() => {
                    if (playTime >= trajectoryData.totalDuration) setPlayTime(0);
                    setIsPlaying(p => !p);
                  }}
                >
                  {isPlaying ? <Pause size={16} /> : <Play size={16} />}
                  {isPlaying ? 'Pausar' : 'Reproduzir'}
                </Button>
                <Button
                  aria-label="Parar e reiniciar timeline"
                  onClick={() => {
                    setIsPlaying(false);
                    seekTrajectory(0);
                  }}
                >
                  <Square size={16} /> Parar
                </Button>
              </div>

              <div className="timeline-opts">
                <label>
                  Velocidade:
                  <Select
                    compact
                    ariaLabel="Velocidade de reprodução"
                    value={playbackSpeed}
                    onChange={val => setPlaybackSpeed(Number(val))}
                    options={[
                      { value: 0.5, label: '0.5x' },
                      { value: 1.0, label: '1.0x' },
                      { value: 1.5, label: '1.5x' },
                      { value: 2.0, label: '2.0x' },
                    ]}
                  />
                </label>
                <Checkbox checked={isLooping} onChange={setIsLooping}>
                  <Repeat size={14} /> Loop
                </Checkbox>
              </div>
            </div>
          </div>

          <div className="field-grid">
            <label className="field-span-full">Modo de Trajetória
              <Select
                ariaLabel="Tipo de interpolação"
                value={interpType}
                onChange={setInterpType}
                options={[
                  { value: 'quintic', label: 'Quíntico PTP (para em cada ponto, sem solavanco)' },
                  { value: 'cubic', label: 'Cúbico PTP (para em cada ponto)' },
                  { value: 'continuous', label: 'Spline Contínua (passagem suave sem parar)' },
                  { value: 'cartesian', label: 'Linear Cartesiano — LIN (linha reta 3D)' },
                ]}
              />
            </label>
            <div className="field-action field-span-full">
              <Button block onClick={addCurrentWaypoint}>
                <Plus size={16} /> Adicionar pose atual
              </Button>
            </div>
          </div>
        </section>

        <section>
          <div className="section-title">
            <h2>Perfis Cinemáticos (q, q̇, q̈, |v_TCP|)</h2>
          </div>
          <p className="muted">Clique ou arraste sobre o gráfico para inspecionar e sincronizar qualquer instante t na cena 3D.</p>
          <TrajectoryCharts
            trajectoryData={trajectoryData}
            playTime={playTime}
            interpType={interpType}
            onSeek={(t) => {
              setIsPlaying(false);
              seekTrajectory(t);
            }}
          />
        </section>

        <section>
          <div className="section-title">
            <h2>Waypoints da sequência ({waypoints.length})</h2>
            <Checkbox checked={showTrajectory} onChange={setShowTrajectory}>
              Trilha 3D
            </Checkbox>
          </div>
          <ul className="waypoint-list">
            {waypoints.map((wp, idx) => (
              <li key={wp.id} className="waypoint-item">
                <div className="waypoint-info">
                  <strong>{idx + 1}. {wp.name}</strong>
                  <span className="waypoint-coords">
                    q: [{wp.q.map(v => fmt(v, 1)).join(', ')}]
                  </span>
                </div>
                {idx > 0 && (
                  <div className="waypoint-duration">
                    <span className="muted">Δt:</span>
                    <input
                      type="number"
                      className="ui-input ui-input-number compact"
                      aria-label={`Duração do waypoint ${idx + 1}`}
                      min={0.1}
                      max={60}
                      step={0.5}
                      defaultValue={wp.duration}
                      onBlur={e => updateWaypointDuration(wp.id, Number(e.target.value))}
                    />
                    <span className="muted">s</span>
                  </div>
                )}
                <div className="item-actions">
                  <Button icon title="Ir para waypoint" aria-label={`Ir para ${wp.name}`} onClick={() => moveToWaypoint(wp)}>
                    <Focus size={15} />
                  </Button>
                  <Button
                    icon
                    variant="danger"
                    title="Remover waypoint"
                    aria-label={`Remover ${wp.name}`}
                    disabled={waypoints.length <= 1}
                    onClick={() => removeWaypoint(wp.id)}
                  >
                    <Trash2 size={15} />
                  </Button>
                </div>
              </li>
            ))}
          </ul>
        </section>
      </>}

      {tab === 'workspace' && <>
        <section>
          <h2>Espaço de Trabalho 3D (Monte Carlo)</h2>
          <p className="muted">Geração do envelope de alcance com cálculo de destreza e índice de manipulabilidade de Yoshikawa.</p>
          <div className="field-grid">
            <label>Amostras
              <Select
                ariaLabel="Número de amostras"
                value={sampleCount}
                onChange={val => setSampleCount(Number(val))}
                options={[
                  { value: 1000, label: '1.000 pontos (Rápido)' },
                  { value: 2500, label: '2.500 pontos (Equilibrado)' },
                  { value: 5000, label: '5.000 pontos (Alta densidade)' },
                ]}
              />
            </label>
            <div className="field-action">
              <Button variant="primary" disabled={computingCloud} onClick={computeWorkspace}>
                {computingCloud ? 'Calculando...' : 'Calcular alcance'}
              </Button>
            </div>
          </div>
        </section>
        {workspaceCloud && <section>
          <h2>Estatísticas do Envelope</h2>
          <dl className="readings">
            <dt>Pontos amostrados</dt><dd>{workspaceCloud.count}</dd>
            <dt>Envelope X (u)</dt><dd>[{fmt(workspaceCloud.bounds.minX, 2)}, {fmt(workspaceCloud.bounds.maxX, 2)}] (Δ {fmt(workspaceCloud.bounds.dx, 2)})</dd>
            <dt>Envelope Y (u)</dt><dd>[{fmt(workspaceCloud.bounds.minY, 2)}, {fmt(workspaceCloud.bounds.maxY, 2)}] (Δ {fmt(workspaceCloud.bounds.dy, 2)})</dd>
            <dt>Envelope Z (u)</dt><dd>[{fmt(workspaceCloud.bounds.minZ, 2)}, {fmt(workspaceCloud.bounds.maxZ, 2)}] (Δ {fmt(workspaceCloud.bounds.dz, 2)})</dd>
            <dt>Volume envolvente (u³)</dt><dd>{fmt(workspaceCloud.boundingVolume, 2)}</dd>
            <dt>Máx. manipulabilidade w</dt><dd>{fmt(workspaceCloud.maxManipulability, 4)}</dd>
          </dl>
          <div className="action-row">
            <Checkbox checked={showWorkspace} onChange={setShowWorkspace}>
              Exibir pontos na cena 3D
            </Checkbox>
          </div>
          <Notice as="p">
            Gradiente térmico: <strong style={{ color: '#4da6ff' }}>Azul</strong> = proximidade de singularidade (baixa destreza); <strong style={{ color: '#ffea79' }}>Amarelo/Vermelho</strong> = alta destreza articular.
          </Notice>
        </section>}
      </>}

      {tab === 'export' && <>
        <section>
          <h2>Exportação de Código de Controle</h2>
          <p className="muted">Gera implementação autônoma e autocontida da cinemática para microcontroladores ou robótica em Python.</p>
          <SegmentedTabs
            ariaLabel="Linguagem de exportação"
            activeId={codeTarget}
            onChange={setCodeTarget}
            items={[
              { id: 'cpp', label: 'C++ / Arduino' },
              { id: 'python', label: 'Python / NumPy' },
              { id: 'ros2', label: 'ROS 2 (Node)' },
              { id: 'urdf', label: 'URDF (ROS)' },
            ]}
          />
          <div className="code-box-header">
            <span className="muted">{codeTarget === 'cpp' ? 'Arduino / ESP32 / C++11' : codeTarget === 'python' ? 'NumPy script' : codeTarget === 'ros2' ? 'rclpy node' : 'URDF XML (ROS)'}</span>
            <div className="btn-row">
              {codeTarget === 'urdf' && (
                <Button onClick={downloadUrdf}>
                  <Download size={16} /> Baixar .urdf
                </Button>
              )}
              <Button onClick={copyCode}>{copied ? <CheckCheck size={16} /> : <Copy size={16} />}{copied ? 'Copiado!' : 'Copiar código'}</Button>
            </div>
          </div>
          <pre className="code-block"><code>{generatedCode}</code></pre>
        </section>
      </>}

      {tab === 'report' && <>
        <section>
          <h2>Relatório Acadêmico (PDF / Impressão)</h2>
          <p className="muted">Geração de documentação formal para aulas e exercícios de robótica com parâmetros DH, matrizes de transformação e Jacobiano.</p>
          <div className="field-grid">
            <label>Nome do Aluno
              <input className="ui-input" aria-label="Nome do Aluno" value={reportMeta.studentName} onChange={e => setReportMeta({ ...reportMeta, studentName: e.target.value })} placeholder="Lucas Engenharia" />
            </label>
            <label>Disciplina / Curso
              <input className="ui-input" aria-label="Disciplina / Curso" value={reportMeta.courseName} onChange={e => setReportMeta({ ...reportMeta, courseName: e.target.value })} placeholder="Robótica e Cinemática" />
            </label>
            <label>Instituição
              <input className="ui-input" aria-label="Instituição" value={reportMeta.institution} onChange={e => setReportMeta({ ...reportMeta, institution: e.target.value })} placeholder="Universidade Federal / Faculdade" />
            </label>
            <label>Título / Modelo
              <input className="ui-input" aria-label="Título / Modelo" value={reportMeta.robotName} onChange={e => setReportMeta({ ...reportMeta, robotName: e.target.value })} placeholder={`Manipulador ${joints.map(j => j.type).join('')}`} />
            </label>
          </div>
          <div className="section-actions">
            <Button variant="primary" onClick={() => openAcademicReport(joints, { ...reportMeta, robotName: reportMeta.robotName || `Manipulador ${joints.map(j => j.type).join('')}`, toolType, toolOffsetRpy, toolGrip })}>
              <Printer size={16} /> Gerar e Imprimir Relatório (PDF)
            </Button>
          </div>
        </section>
        <section>
          <h2>Resumo dos Cálculos Gerados</h2>
          <dl className="readings">
            <dt>Manipulador</dt><dd>{joints.map(j => j.type).join('')} ({joints.length} DOF)</dd>
            <dt>Ferramenta (TCP)</dt><dd>{TOOL_TYPES[toolType].shortLabel} · {fmt(TOOL_TYPES[toolType].length, 2)} u</dd>
            <dt>Posição XYZ (u)</dt><dd>{vectorText(fk.positions.at(-1))}</dd>
            <dt>Posição do TCP (u)</dt><dd>{vectorText(toolFrame.tcpPosition)}</dd>
            <dt>Orientação RPY (°)</dt><dd>{vectorText(toolFrame.orientation.rpy)}</dd>
            <dt>Posto do Jacobiano</dt><dd>{diagnostics.rank} de {diagnostics.maxRank}</dd>
            <dt>Condicionamento</dt><dd>{fmt(diagnostics.condition)}</dd>
          </dl>
        </section>
      </>}
    </aside>
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
        ikTarget={tab === 'inverse' ? [target.position[0], target.position[1], mode === 'xy' ? currentEnd[2] : target.position[2]] : null}
        ikRpy={target.rpy}
        gizmoEnabled={tab === 'inverse' && showGizmo}
        gizmoMode={gizmoMode}
        taskMode={mode}
        onGizmoDrag={handleGizmoDrag}
        collisionPoint={activeCollision?.point || null}
        environment={environment}
      />
      <div className="scene-toolbar">
        <Checkbox checked={showFrames} onChange={setShowFrames}>Referenciais</Checkbox>
        <Checkbox checked={environment.obstacle.enabled} onChange={enabled => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, enabled } })}>Obstáculo 3D</Checkbox>
        {tab === 'inverse' && <Checkbox checked={showGizmo} onChange={setShowGizmo}>Gizmo 3D</Checkbox>}
        {trajectoryData.pathPoints.length > 0 && <Checkbox checked={showTrajectory} onChange={setShowTrajectory}>Trilha 3D</Checkbox>}
        {workspaceCloud && <Checkbox checked={showWorkspace} onChange={setShowWorkspace}>Nuvem 3D</Checkbox>}
        {customStlGeometry ? (
          <Button icon variant="danger" title={`Remover malha STL (${stlFileName})`} aria-label="Remover malha STL" onClick={removeStlMesh}>
            <Trash2 size={16} />
          </Button>
        ) : (
          <FileUploadButton icon accept=".stl" onChange={handleStlUpload} title="Carregar malha STL (Garra / Ferramenta)" ariaLabel="Carregar malha STL" inputAriaLabel="Upload de malha STL">
            <Upload size={17} />
          </FileUploadButton>
        )}
        <Button icon title="Enquadrar manipulador" aria-label="Enquadrar manipulador" onClick={() => setViewKey(k => k + 1)}><Focus size={19} /></Button>
      </div>
      <div className="scene-readout">
        {activeCollision && <div className="collision-readout-pill">Bloqueio de colisão: {activeCollision.message}</div>}
        <span>Efetuador ({TOOL_TYPES[toolType].shortLabel}) · RPY [{fmt(toolFrame.orientation.rpy[0], 1)}°, {fmt(toolFrame.orientation.rpy[1], 1)}°, {fmt(toolFrame.orientation.rpy[2], 1)}°]</span>
        <output data-testid="end-position">X {fmt(fk.positions.at(-1)[0], 2)} · Y {fmt(fk.positions.at(-1)[1], 2)} · Z {fmt(fk.positions.at(-1)[2], 2)} u</output>
      </div>
    </div>
  </main>;
}

export default App;
