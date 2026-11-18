import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import RobotScene from './RobotScene';
import SceneToolbar from './SceneToolbar';
import { NumberField } from './ui';
import { DEFAULT_ENVIRONMENT, DEFAULT_MODEL_STYLE, DEFAULT_TOOL_OFFSET_RPY, TOOL_TYPES, computeManipulabilityEllipsoid, computeToolFrame, detectSelfCollision, forwardKinematics, integrateJoints, inverseVelocity, jacobianDiagnostics, multiplyVector, normalizeJoints, normalizeModelStyle, staticJointEfforts, sweepJointsCollisionFree, validateJoints } from './kinematics';
import { PRESETS } from './presets';
import { computeCartesianBounds } from './workspace';
import { generateCpp, generatePython, generateRos2 } from './codeGenerator';
import { defaultWaypointsForJoints, syncWaypointsToJoints } from './trajectory';
import { exportToURDF, parseURDF } from './urdfParser';
import { parseStlBuffer } from './cadLoader';
import { fetchSharedSimulation, normalizeEnvironment, request } from './storage';
import { usePanels } from './hooks/usePanels';
import { useHistory } from './hooks/useHistory';
import { useTrajectoryPlayer } from './hooks/useTrajectoryPlayer';
import { useInverseControl } from './hooks/useInverseControl';
import { runSingularityAnalysisAsync, runWorkspaceCloudAsync, terminateWorkspaceWorker } from './workers/workspaceClient';
import { fmt } from './format';
import ModelTab from './tabs/ModelTab';
import ProjectsTab from './tabs/ProjectsTab';
import ForwardTab from './tabs/ForwardTab';
import DifferentialTab from './tabs/DifferentialTab';
import InverseTab from './tabs/InverseTab';
import SingularitiesTab from './tabs/SingularitiesTab';
import TrajectoryTab from './tabs/TrajectoryTab';
import WorkspaceTab from './tabs/WorkspaceTab';
import ExportTab from './tabs/ExportTab';
import ReportTab from './tabs/ReportTab';
import './App.css';

const INITIAL_PRESET = PRESETS.ur5;
const INITIAL_TOOL_OFFSET = Array.isArray(INITIAL_PRESET.toolOffsetRpy) ? [...INITIAL_PRESET.toolOffsetRpy] : [...DEFAULT_TOOL_OFFSET_RPY];
const INITIAL_FK = forwardKinematics(INITIAL_PRESET.joints);
const INITIAL_TOOL_FRAME = computeToolFrame(INITIAL_FK.total, INITIAL_TOOL_OFFSET, 'gripper');

function App() {
  const [joints, setJoints] = useState(() => INITIAL_PRESET.joints.map(j => ({ ...j })));
  const jointsRef = useRef(joints);
  useEffect(() => { jointsRef.current = joints; }, [joints]);
  const [simulations, setSimulations] = useState([]);
  const [simName, setSimName] = useState('');
  const [busy, setBusy] = useState(false);
  const [apiError, setApiError] = useState('');
  const [message, setMessage] = useState('');
  const clearMessage = useCallback(() => setMessage(''), []);
  const {
    tab,
    openPanels,
    pinnedPanels,
    panelFocusOrder,
    activePanel,
    showInverse,
    showDifferential,
    showSingularities,
    showReport,
    showExport,
    handleSelectTab,
    handleTogglePanel,
    handleClosePanel,
    handleTogglePinPanel,
    handleFocusPanel,
  } = usePanels(clearMessage);
  const [collisionAlert, setCollisionAlert] = useState(null);
  const [mode, setMode] = useState('position');
  const [length, setLength] = useState(5);
  const [damping, setDamping] = useState(0.02);
  const [dt, setDt] = useState(0.1);
  const [desiredVelocity, setDesiredVelocity] = useState([0, 0, 0, 0, 0, 0]);
  const [wrench, setWrench] = useState([0, 0, 0, 0, 0, 0]);
  const [payloadMass, setPayloadMass] = useState(2);
  const [gravity, setGravity] = useState(10);
  const [showFrames, setShowFrames] = useState(true);
  const [viewKey, setViewKey] = useState(0);
  const [viewPreset, setViewPreset] = useState('iso');
  const [showEllipsoid, setShowEllipsoid] = useState(false);
  const [showGhost, setShowGhost] = useState(false);
  const [workspaceCloud, setWorkspaceCloud] = useState(null);
  const [showWorkspace, setShowWorkspace] = useState(true);
  const [sampleCount, setSampleCount] = useState(2500);
  const [computingCloud, setComputingCloud] = useState(false);
  const [singularityReport, setSingularityReport] = useState(null);
  const [computingSingularities, setComputingSingularities] = useState(false);
  const [codeTarget, setCodeTarget] = useState('cpp');
  const [copied, setCopied] = useState(false);
  const [shareUrl, setShareUrl] = useState('');
  const [shareLoading, setShareLoading] = useState(false);
  const [shareCopied, setShareCopied] = useState(false);
  const [customStlGeometry, setCustomStlGeometry] = useState(null);
  const [stlFileName, setStlFileName] = useState('');
  const [customLinkStls, setCustomLinkStls] = useState({});
  const [urdfMeshHints, setUrdfMeshHints] = useState({});
  const [showSingularityHighlights, setShowSingularityHighlights] = useState(true);
  const [hoveredSingularity, setHoveredSingularity] = useState(null);
  const [toolType, setToolType] = useState('gripper');
  const toolTypeRef = useRef(toolType);
  useEffect(() => { toolTypeRef.current = toolType; }, [toolType]);
  const [defaultToolOffset, setDefaultToolOffset] = useState(() => [...INITIAL_TOOL_OFFSET]);
  const [toolOffsetRpy, setToolOffsetRpy] = useState(() => [...INITIAL_TOOL_OFFSET]);
  const [toolGrip, setToolGrip] = useState(50);
  const [toolScale, setToolScale] = useState(1);
  const [modelStyle, setModelStyle] = useState(() => ({ ...DEFAULT_MODEL_STYLE }));
  const [modelTarget, setModelTarget] = useState('global');
  const [environment, setEnvironment] = useState(() => ({
    ...DEFAULT_ENVIRONMENT,
    obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
  }));
  const environmentRef = useRef(environment);
  useEffect(() => { environmentRef.current = environment; }, [environment]);
  const [showObstacleGizmo, setShowObstacleGizmo] = useState(true);
  // Any continuous pointer interaction (range sliders in any panel, 3D canvas drags) defers
  // heavy derived computations (Cartesian slider bounds, singularity workers, undo snapshots) until release.
  const [isPointerInteracting, setIsPointerInteracting] = useState(false);
  useEffect(() => {
    const onDown = (e) => {
      const el = e.target;
      if (el instanceof Element && el.closest('input[type="range"], canvas')) setIsPointerInteracting(true);
    };
    const onUp = () => setIsPointerInteracting(false);
    window.addEventListener('pointerdown', onDown, true);
    window.addEventListener('pointerup', onUp, true);
    window.addEventListener('pointercancel', onUp, true);
    window.addEventListener('blur', onUp);
    return () => {
      window.removeEventListener('pointerdown', onDown, true);
      window.removeEventListener('pointerup', onUp, true);
      window.removeEventListener('pointercancel', onUp, true);
      window.removeEventListener('blur', onUp);
    };
  }, []);
  const shortcutsRef = useRef({ undo: () => {}, redo: () => {}, addWaypoint: () => {} });
  const handleFrameView = useCallback(() => setViewKey(k => k + 1), []);
  const handleSelectObstacle = useCallback(() => setShowObstacleGizmo(true), []);
  const [reportMeta, setReportMeta] = useState({
    studentName: 'Lucas Engenharia',
    courseName: 'Robótica Industrial e Cinemática',
    institution: 'Universidade / Faculdade de Engenharia',
    robotName: '',
  });
  const fk = useMemo(() => forwardKinematics(joints), [joints]);
  const toolFrame = useMemo(() => computeToolFrame(fk.total, toolOffsetRpy, toolType, toolScale), [fk.total, toolOffsetRpy, toolType, toolScale]);
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
  const syncTargetRef = useRef(null);
  const commitJointsRef = useRef(null);
  const onSyncTarget = useCallback((nextTarget) => syncTargetRef.current?.(nextTarget), []);
  const onCommitJoints = useCallback((nextJoints) => commitJointsRef.current?.(nextJoints), []);

  const {
    waypoints,
    setWaypoints,
    isPlaying,
    setIsPlaying,
    stopPlayback,
    playTime,
    setPlayTime,
    playbackSpeed,
    setPlaybackSpeed,
    isLooping,
    setIsLooping,
    interpType,
    setInterpType,
    angleUnit,
    setAngleUnit,
    showTrajectory,
    setShowTrajectory,
    trajectoryData,
    ghostJoints,
    seekTrajectory,
    addWaypointFromPose,
    addCurrentWaypoint,
    removeWaypoint,
    moveToWaypoint,
    updateWaypointDuration,
    toggleTrajectoryPlayback,
    handleSelectWaypoint,
  } = useTrajectoryPlayer({
    initialJoints: INITIAL_PRESET.joints,
    showGhost,
    joints,
    jointsRef,
    setJoints,
    toolType,
    toolTypeRef,
    toolOffsetRpy,
    environment,
    environmentRef,
    setCollisionAlert,
    setMessage,
    onSyncTarget,
    onCommitJoints,
  });

  const {
    target,
    setTarget,
    solution,
    setSolution,
    posture,
    setPosture,
    autoSolveIk,
    setAutoSolveIk,
    ikWaypointDuration,
    setIkWaypointDuration,
    showGizmo,
    setShowGizmo,
    gizmoMode,
    setGizmoMode,
    gizmoLockOrientation,
    setGizmoLockOrientation,
    gizmoLockPosition,
    setGizmoLockPosition,
    isContinuousMotion,
    cartesianBounds,
    postureInfo,
    targetDistance,
    toolAngularErrorDeg,
    ikTargetForScene,
    ikRpyForScene,
    solveInverse,
    selectPosture,
    applyPostureBranch,
    syncTargetAndModeForJoints,
    applyOrientationPreset,
    changeTarget,
    slideTarget,
    handleGizmoDragStart,
    handleGizmoDragEnd,
    handleGizmoDrag,
    handleGizmoModeChange,
    handleIkSliderDragStart,
    handleIkSliderDragEnd,
  } = useInverseControl({
    initialTarget: {
      position: INITIAL_FK.positions.at(-1),
      rpy: INITIAL_TOOL_FRAME.orientation.rpy,
    },
    joints,
    jointsRef,
    setJoints,
    currentEnd,
    toolFrame,
    mode,
    setMode,
    length,
    damping,
    toolType,
    toolOffsetRpy,
    toolScale,
    environment,
    tab,
    showInverse,
    globalCartesianBounds,
    isPointerInteracting,
    isPlaying,
    stopPlayback,
    setCollisionAlert,
    setMessage,
  });

  const workspaceReqRef = useRef(0);
  const invalidateWorkspaceCloud = useCallback(() => {
    workspaceReqRef.current += 1;
    setWorkspaceCloud(null);
    setComputingCloud(false);
  }, []);

  const singularityReqRef = useRef(0);
  const lastSingularityKeyRef = useRef('');
  const clearSingularityState = useCallback(() => {
    singularityReqRef.current += 1;
    lastSingularityKeyRef.current = '';
    setSingularityReport(null);
    setComputingSingularities(false);
  }, []);

  const singularityInputKey = useMemo(
    () => `${joints.map(j => `${j.id}:${j.type}:${j.a}:${j.alpha}:${j.d}:${j.theta}:${j.val}:${j.min}:${j.max}`).join('|')}@${mode}@${JSON.stringify(environment)}`,
    [joints, mode, environment]
  );

  useEffect(() => {
    if (!showSingularities || isContinuousMotion) return undefined;
    if (lastSingularityKeyRef.current === singularityInputKey && singularityReport) return undefined;
    let cancelled = false;
    const reqId = ++singularityReqRef.current;
    const snapshotJoints = joints.map(j => ({ ...j }));
    const snapshotEnd = [...currentEnd];
    const snapshotEnv = environment;
    const snapshotMode = mode;
    const snapshotKey = singularityInputKey;
    setComputingSingularities(true);
    runSingularityAnalysisAsync(snapshotJoints, {
      referencePos: snapshotEnd,
      mode: snapshotMode,
      environment: snapshotEnv,
    })
      .then((report) => {
        if (cancelled || singularityReqRef.current !== reqId) return;
        lastSingularityKeyRef.current = snapshotKey;
        setSingularityReport(report);
      })
      .catch(() => {})
      .finally(() => {
        if (!cancelled && singularityReqRef.current === reqId) {
          setComputingSingularities(false);
        }
      });
    return () => { cancelled = true; };
  }, [showSingularities, isContinuousMotion, singularityInputKey, singularityReport, joints, currentEnd, mode, environment]);

  useEffect(() => () => terminateWorkspaceWorker(), []);

  const diagnostics = useMemo(
    () => ((showDifferential || showReport || showSingularities) ? jacobianDiagnostics(joints, fk.jacobian, mode, length) : null),
    [showDifferential, showReport, showSingularities, joints, fk, mode, length]
  );
  const velocity = useMemo(
    () => (showDifferential ? multiplyVector(fk.jacobian, joints.map(j => j.speed)) : null),
    [showDifferential, fk, joints]
  );
  const differential = useMemo(
    () => (showDifferential ? inverseVelocity(joints, fk.jacobian, desiredVelocity, { mode, length, damping }) : null),
    [showDifferential, joints, fk, desiredVelocity, mode, length, damping]
  );
  const statics = useMemo(
    () => (showDifferential ? staticJointEfforts(joints, fk.jacobian, wrench) : null),
    [showDifferential, joints, fk, wrench]
  );
  const ellipsoid = useMemo(
    () => (showEllipsoid ? computeManipulabilityEllipsoid(fk.jacobian) : null),
    [showEllipsoid, fk.jacobian]
  );
  const selectedJointId = useMemo(
    () => (modelTarget !== 'global' && joints.some(j => String(j.id) === modelTarget) ? Number(modelTarget) : null),
    [modelTarget, joints]
  );

  useEffect(() => {
    const onKeyDown = (e) => {
      if (e.defaultPrevented) return;
      const targetEl = e.target;
      const tag = targetEl?.tagName?.toLowerCase();
      if (tag === 'input' || tag === 'textarea' || tag === 'select' || targetEl?.isContentEditable) return;
      const key = e.key.toLowerCase();
      if ((e.ctrlKey || e.metaKey) && !e.altKey) {
        if (key === 'z' && !e.shiftKey) {
          e.preventDefault();
          shortcutsRef.current.undo();
        } else if (key === 'y' || (key === 'z' && e.shiftKey)) {
          e.preventDefault();
          shortcutsRef.current.redo();
        }
        return;
      }
      if (!e.ctrlKey && !e.metaKey && !e.altKey && !e.shiftKey && key === 'w') {
        e.preventDefault();
        shortcutsRef.current.addWaypoint();
      } else if (!e.ctrlKey && !e.metaKey && !e.altKey && !e.shiftKey && e.code === 'Space' && tag !== 'button' && tag !== 'a') {
        e.preventDefault();
        shortcutsRef.current.togglePlayback();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, []);

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
      fetchSharedSimulation(slug, controller.signal)
        .then(resData => {
          if (resData.data && validateJoints(resData.data).length === 0) {
            const normalized = normalizeJoints(resData.data);
            const nextFk = forwardKinematics(normalized);
            const loadedTool = resData.toolConfig || null;
            const nextToolType = loadedTool?.toolType || 'gripper';
            const nextOffset = loadedTool?.toolOffsetRpy || [...DEFAULT_TOOL_OFFSET_RPY];
            const nextGrip = loadedTool?.toolGrip ?? 50;
            const nextScale = Number.isFinite(Number(loadedTool?.toolScale)) ? Math.max(0.4, Math.min(4.0, Number(loadedTool.toolScale))) : 1;
            const nextToolFrame = computeToolFrame(nextFk.total, nextOffset, nextToolType, nextScale);
            setJoints(normalized);
            setToolType(nextToolType);
            setToolOffsetRpy(nextOffset);
            setToolGrip(nextGrip);
            setToolScale(nextScale);
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
  }, [setIsPlaying, setPlayTime, setSolution, setTarget, setWaypoints]);

  function applyEnvironment(nextEnv) {
    setEnvironment(nextEnv);
    invalidateWorkspaceCloud();
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

  function commitJoints(next) {
    const errors = validateJoints(next);
    if (errors.length) { setMessage(errors.join(' ')); return false; }
    const normalized = normalizeJoints(next);
    setJoints(normalized);
    const nextFk = forwardKinematics(normalized);
    const nextTool = computeToolFrame(nextFk.total, toolOffsetRpy, toolType);
    setTarget({ position: nextFk.positions.at(-1), rpy: nextTool.orientation.rpy });
    setSolution(null); setMessage(''); setCollisionAlert(null);
    return true;
  }

  useEffect(() => {
    syncTargetRef.current = setTarget;
    commitJointsRef.current = commitJoints;
  });

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
      invalidateWorkspaceCloud();
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
    setPosture('auto');
    setViewKey(k => k + 1);
    invalidateWorkspaceCloud();
    clearSingularityState();
    Object.values(customLinkStls).forEach(entry => entry?.geometry?.dispose?.());
    setCustomLinkStls({});
    setUrdfMeshHints({});
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints(defaultWaypointsForJoints(p.joints));
    setMessage(`Robô ${p.name} carregado.`);
  }

  function applyJointPose(qVec, note = '') {
    if (!Array.isArray(qVec)) return;
    setIsPlaying(false);
    const candidate = joints.map((j, i) => ({
      ...j,
      val: qVec[i] !== undefined ? Math.max(j.min, Math.min(j.max, qVec[i])) : j.val,
      speed: 0,
    }));
    const normalized = normalizeJoints(candidate);
    const col = detectSelfCollision(normalized, null, toolType, environment);
    if (col.colliding) {
      setCollisionAlert(col);
      setMessage(`Pose bloqueada para evitar colisão: ${col.message}`);
      return;
    }
    if (commitJoints(normalized) && note) {
      setMessage(note);
    }
  }

  async function computeWorkspace() {
    const reqId = ++workspaceReqRef.current;
    setComputingCloud(true);
    try {
      const cloud = await runWorkspaceCloudAsync(jointsRef.current, sampleCount, environmentRef.current);
      if (workspaceReqRef.current !== reqId) return;
      setWorkspaceCloud(cloud);
      setShowWorkspace(true);
      setMessage(`Nuvem de trabalho calculada com ${cloud.count} amostras.`);
    } catch (err) {
      if (workspaceReqRef.current === reqId) {
        setMessage('Falha ao calcular espaço de trabalho: ' + err.message);
      }
    } finally {
      if (workspaceReqRef.current === reqId) {
        setComputingCloud(false);
      }
    }
  }

  const generatedCode = useMemo(() => {
    if (!showExport) return '';
    const robotName = `Manipulador_${joints.map(j => j.type).join('')}_${joints.length}DOF`;
    const linkMeshes = {};
    joints.forEach(j => {
      const entry = customLinkStls[String(j.id)];
      if (entry?.fileName) linkMeshes[String(j.id)] = { fileName: entry.fileName, scale: entry.scale ?? 1 };
    });
    const toolConfig = {
      toolType,
      toolOffsetRpy,
      toolGrip,
      modelStyle,
      ...(Object.keys(linkMeshes).length > 0 ? { linkMeshes } : {}),
      ...(stlFileName ? { toolMeshFileName: stlFileName } : {}),
    };
    if (codeTarget === 'cpp') return generateCpp(joints, robotName, toolConfig);
    if (codeTarget === 'python') return generatePython(joints, robotName, toolConfig);
    if (codeTarget === 'urdf') return exportToURDF(joints, robotName, toolConfig);
    return generateRos2(joints, robotName, toolConfig);
  }, [showExport, joints, codeTarget, toolType, toolOffsetRpy, toolGrip, modelStyle, customLinkStls, stlFileName]);

  function downloadUrdf() {
    const linkMeshes = {};
    joints.forEach(j => {
      const entry = customLinkStls[String(j.id)];
      if (entry?.fileName) linkMeshes[String(j.id)] = { fileName: entry.fileName, scale: entry.scale ?? 1 };
    });
    const urdfContent = exportToURDF(joints, `Manipulador_${joints.map(j => j.type).join('')}`, {
      toolType,
      toolOffsetRpy,
      toolGrip,
      modelStyle,
      ...(Object.keys(linkMeshes).length > 0 ? { linkMeshes } : {}),
      ...(stlFileName ? { toolMeshFileName: stlFileName } : {}),
    });
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
          const importedLinkMeshes = importedTool?.linkMeshes || {};
          Object.values(customLinkStls).forEach(entry => entry?.geometry?.dispose?.());
          setCustomLinkStls({});
          setUrdfMeshHints(importedLinkMeshes);
          clearSingularityState();
          invalidateWorkspaceCloud();
          syncTargetAndModeForJoints(normalized, 'position', importedTool);
          setIsPlaying(false);
          setPlayTime(0);
          setWaypoints(defaultWaypointsForJoints(normalized));
          setViewKey(k => k + 1);
          const meshCount = Object.keys(importedLinkMeshes).length + (importedTool?.toolMeshFileName ? 1 : 0);
          setMessage(meshCount > 0
            ? `Robô importado via URDF com ${parsed.length} juntas e ${meshCount} referência(s) <mesh>. Anexe os .stl em Modelagem 3D.`
            : `Robô importado via URDF com sucesso (${parsed.length} juntas).`);
        }
      } catch (err) {
        setMessage('Erro ao importar URDF: ' + err.message);
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  }

  function exportProjectJson() {
    const projectName = simName.trim() || `Manipulador_${joints.map(j => j.type).join('')}`;
    const payload = {
      format: 'rpr-simulator-project',
      version: 1,
      exportedAt: new Date().toISOString(),
      name: projectName,
      data: joints,
      waypoints,
      toolConfig: { toolType, toolOffsetRpy, toolGrip, toolScale, modelStyle },
      environment,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${projectName.replace(/[^a-zA-Z0-9_-]+/g, '_') || 'projeto_robo'}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    setMessage('Projeto JSON exportado com sucesso.');
  }

  function handleProjectJsonUpload(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const parsed = JSON.parse(String(ev.target.result || '{}'));
        const errors = validateJoints(parsed?.data);
        if (errors.length > 0) {
          setMessage('Arquivo JSON inválido: ' + errors.join(' '));
          return;
        }
        const normalized = normalizeJoints(parsed.data);
        const importedTool = parsed.toolConfig || null;
        const nextToolType = importedTool?.toolType && TOOL_TYPES[importedTool.toolType] ? importedTool.toolType : 'gripper';
        const nextOffset = Array.isArray(importedTool?.toolOffsetRpy) && importedTool.toolOffsetRpy.length === 3
          ? importedTool.toolOffsetRpy.map(Number)
          : [...DEFAULT_TOOL_OFFSET_RPY];
        const nextGrip = Number.isFinite(Number(importedTool?.toolGrip)) ? Number(importedTool.toolGrip) : 50;
        const nextScale = Number.isFinite(Number(importedTool?.toolScale)) ? Math.max(0.4, Math.min(4.0, Number(importedTool.toolScale))) : 1;
        const nextEnv = parsed.environment ? normalizeEnvironment(parsed.environment) : environment;
        setToolType(nextToolType);
        setToolOffsetRpy(nextOffset);
        setToolGrip(nextGrip);
        setToolScale(nextScale);
        setModelStyle(normalizeModelStyle(importedTool?.modelStyle));
        setEnvironment(nextEnv);
        invalidateWorkspaceCloud();
        clearSingularityState();
        commitJoints(normalized);
        syncTargetAndModeForJoints(normalized, null, { toolType: nextToolType, toolOffsetRpy: nextOffset });
        setIsPlaying(false);
        setPlayTime(0);
        setWaypoints(
          Array.isArray(parsed.waypoints) && parsed.waypoints.length > 0
            ? syncWaypointsToJoints(parsed.waypoints, normalized)
            : defaultWaypointsForJoints(normalized)
        );
        setViewKey(k => k + 1);
        setMessage(`Projeto "${parsed.name || file.name}" importado com sucesso.`);
      } catch (err) {
        setMessage('Erro ao importar projeto JSON: ' + err.message);
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

  function handleLinkStlUpload(jointId, e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const geom = parseStlBuffer(ev.target.result, { targetSize: 1.0 });
        const key = String(jointId);
        const jIdx = joints.findIndex(j => String(j.id) === key);
        setCustomLinkStls(prev => {
          if (prev[key]?.geometry) prev[key].geometry.dispose();
          return {
            ...prev,
            [key]: { geometry: geom, fileName: file.name, scale: prev[key]?.scale ?? 1 },
          };
        });
        setMessage(`Malha STL "${file.name}" aplicada ao elo ${jIdx >= 0 ? jIdx + 1 : key}.`);
      } catch (err) {
        setMessage('Erro ao processar arquivo STL do elo: ' + err.message);
      }
    };
    reader.readAsArrayBuffer(file);
    e.target.value = '';
  }

  function updateLinkStlScale(jointId, nextScale) {
    const key = String(jointId);
    const safeScale = Number.isFinite(Number(nextScale)) ? Math.max(0.3, Math.min(3.0, Number(nextScale))) : 1;
    setCustomLinkStls(prev => (prev[key] ? { ...prev, [key]: { ...prev[key], scale: safeScale } } : prev));
  }

  function removeLinkStl(jointId) {
    const key = String(jointId);
    const jIdx = joints.findIndex(j => String(j.id) === key);
    setCustomLinkStls(prev => {
      if (!prev[key]) return prev;
      if (prev[key].geometry) prev[key].geometry.dispose();
      const next = { ...prev };
      delete next[key];
      return next;
    });
    setMessage(`Malha STL removida do elo ${jIdx >= 0 ? jIdx + 1 : key}.`);
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
          toolConfig: { toolType, toolOffsetRpy, toolGrip, toolScale, modelStyle },
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
    clearSingularityState();
    invalidateWorkspaceCloud();
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints(defaultWaypointsForJoints(INITIAL_PRESET.joints));
    setViewKey(k => k + 1);
  }

  const handleSelectViewPreset = useCallback((preset) => {
    setViewPreset(preset);
    setViewKey(k => k + 1);
  }, []);

  const handleSelectJoint = useCallback((jointId) => {
    const idx = jointsRef.current.findIndex(j => j.id === jointId);
    if (idx < 0) return;
    setModelTarget(String(jointId));
    setMessage(`Eixo / Ligação ${idx + 1} selecionado na cena 3D.`);
  }, []);

  function applyHistorySnapshot(snap) {
    const normalized = normalizeJoints(snap.joints);
    setIsPlaying(false);
    setPlayTime(0);
    setJoints(normalized);
    setToolType(snap.toolType);
    setToolOffsetRpy([...snap.toolOffsetRpy]);
    setToolGrip(snap.toolGrip);
    setToolScale(snap.toolScale ?? 1);
    setModelStyle(normalizeModelStyle(snap.modelStyle));
    setEnvironment(normalizeEnvironment(snap.environment));
    setWaypoints(syncWaypointsToJoints(snap.waypoints, normalized));
    invalidateWorkspaceCloud();
    clearSingularityState();
    setCollisionAlert(null);
    syncTargetAndModeForJoints(normalized, null, { toolType: snap.toolType, toolOffsetRpy: snap.toolOffsetRpy });
  }

  const { canUndo, canRedo, handleUndo, handleRedo } = useHistory({
    joints,
    toolType,
    toolOffsetRpy,
    toolGrip,
    toolScale,
    modelStyle,
    environment,
    waypoints,
    isContinuousMotion,
    onApplySnapshot: applyHistorySnapshot,
    onMessage: setMessage,
  });

  useEffect(() => {
    shortcutsRef.current = {
      undo: handleUndo,
      redo: handleRedo,
      addWaypoint: addCurrentWaypoint,
      togglePlayback: toggleTrajectoryPlayback,
    };
  });

  const renderPanelContent = (panelId) => (panelId === 'projects' ? (
    <ProjectsTab
      joints={joints}
      applyPreset={applyPreset}
      handleUrdfUpload={handleUrdfUpload}
      toolType={toolType}
      toolOffsetRpy={toolOffsetRpy}
      toolGrip={toolGrip}
      toolScale={toolScale}
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
      setToolScaleLoaded={setToolScale}
      setModelStyleLoaded={setModelStyle}
      syncTargetAndModeForJoints={syncTargetAndModeForJoints}
      exportProjectJson={exportProjectJson}
      handleProjectJsonUpload={handleProjectJsonUpload}
    />
  ) : panelId === 'model' ? (
    <ModelTab
      joints={joints}
      modelStyle={modelStyle}
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
  ) : panelId === 'forward' ? (
    <ForwardTab joints={joints} fk={fk} />
  ) : panelId === 'differential' ? (
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
  ) : panelId === 'inverse' ? (
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
      gizmoLockOrientation={gizmoLockOrientation}
      setGizmoLockOrientation={setGizmoLockOrientation}
      gizmoLockPosition={gizmoLockPosition}
      setGizmoLockPosition={setGizmoLockPosition}
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
      posture={posture}
      postureInfo={postureInfo}
      onSelectPosture={selectPosture}
      onApplyBranch={applyPostureBranch}
      message={message}
      solveInverse={solveInverse}
      changeTarget={changeTarget}
      slideTarget={slideTarget}
      onSliderDragStart={handleIkSliderDragStart}
      onSliderDragEnd={handleIkSliderDragEnd}
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
  ) : panelId === 'singularities' ? (
    <SingularitiesTab
      joints={joints}
      toolFrame={toolFrame}
      mode={mode}
      setMode={setMode}
      length={length}
      setLength={setLength}
      damping={damping}
      setDamping={setDamping}
      onChangeSetting={changeSetting}
      diagnostics={diagnostics}
      singularityReport={singularityReport}
      applyJointPose={applyJointPose}
      onComputeWorkspace={computeWorkspace}
      computingCloud={computingCloud}
      hasWorkspace={Boolean(workspaceCloud)}
      showWorkspace={showWorkspace}
      setShowWorkspace={setShowWorkspace}
      showEllipsoid={showEllipsoid}
      setShowEllipsoid={setShowEllipsoid}
      showSingularityHighlights={showSingularityHighlights}
      setShowSingularityHighlights={setShowSingularityHighlights}
      activeSingularityHighlight={hoveredSingularity}
      onHoverSingularity={setHoveredSingularity}
      computingSingularities={computingSingularities}
      numeric={numeric}
    />
  ) : panelId === 'trajectory' ? (
    <TrajectoryTab
      joints={joints}
      modelStyle={modelStyle}
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
  ) : panelId === 'workspace' ? (
    <WorkspaceTab
      sampleCount={sampleCount}
      setSampleCount={setSampleCount}
      computingCloud={computingCloud}
      computeWorkspace={computeWorkspace}
      workspaceCloud={workspaceCloud}
      showWorkspace={showWorkspace}
      setShowWorkspace={setShowWorkspace}
    />
  ) : panelId === 'export' ? (
    <ExportTab
      codeTarget={codeTarget}
      setCodeTarget={setCodeTarget}
      generatedCode={generatedCode}
      downloadUrdf={downloadUrdf}
      copyCode={copyCode}
      copied={copied}
    />
  ) : panelId === 'report' ? (
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
  ) : null);

  return <main className="app-container app-fullscreen">
    <div className="canvas-container">
      <RobotScene
        joints={joints}
        endPosition={currentEnd}
        showFrames={showFrames}
        viewKey={viewKey}
        viewPreset={viewPreset}
        workspaceCloud={workspaceCloud}
        showWorkspace={showWorkspace}
        trajectoryPathPoints={trajectoryData.pathPoints}
        trajectoryWaypointCoords={trajectoryData.waypointCoords}
        showTrajectory={showTrajectory}
        onSelectWaypoint={handleSelectWaypoint}
        customStlGeometry={customStlGeometry}
        customLinkStls={customLinkStls}
        toolType={toolType}
        toolOffsetRpy={toolOffsetRpy}
        toolGrip={toolGrip}
        toolScale={toolScale}
        modelStyle={modelStyle}
        selectedJointId={selectedJointId}
        onSelectJoint={handleSelectJoint}
        ellipsoid={ellipsoid}
        showEllipsoid={showEllipsoid}
        singularityReport={showSingularities && showSingularityHighlights ? singularityReport : null}
        activeSingularityHighlight={showSingularities && showSingularityHighlights ? hoveredSingularity : null}
        ghostJoints={ghostJoints}
        showGhost={showGhost}
        ikTarget={ikTargetForScene}
        ikRpy={ikRpyForScene}
        gizmoEnabled={showGizmo}
        gizmoMode={gizmoMode}
        taskMode={mode}
        onGizmoDrag={handleGizmoDrag}
        onGizmoDragStart={handleGizmoDragStart}
        onGizmoDragEnd={handleGizmoDragEnd}
        collisionPoint={activeCollision?.point || null}
        environment={environment}
        obstacleGizmoEnabled={environment.obstacle.enabled && showObstacleGizmo}
        onDragObstacle={handleObstacleDrag}
        onSelectObstacle={handleSelectObstacle}
      />
      <SceneToolbar
        joints={joints}
        tab={tab}
        activePanel={activePanel}
        openPanels={openPanels}
        pinnedPanels={pinnedPanels}
        panelFocusOrder={panelFocusOrder}
        onSelectTab={handleSelectTab}
        onTogglePanel={handleTogglePanel}
        onClosePanel={handleClosePanel}
        onTogglePinPanel={handleTogglePinPanel}
        onFocusPanel={handleFocusPanel}
        renderPanelContent={renderPanelContent}
        message={message}
        activeCollision={activeCollision}
        showFrames={showFrames}
        setShowFrames={setShowFrames}
        environment={environment}
        applyEnvironment={applyEnvironment}
        showGizmo={showGizmo}
        setShowGizmo={setShowGizmo}
        gizmoMode={gizmoMode}
        onChangeGizmoMode={handleGizmoModeChange}
        gizmoLockOrientation={gizmoLockOrientation}
        setGizmoLockOrientation={setGizmoLockOrientation}
        gizmoLockPosition={gizmoLockPosition}
        setGizmoLockPosition={setGizmoLockPosition}
        hasTrajectory={trajectoryData.pathPoints.length > 0}
        showTrajectory={showTrajectory}
        setShowTrajectory={setShowTrajectory}
        hasWorkspace={Boolean(workspaceCloud)}
        showWorkspace={showWorkspace}
        setShowWorkspace={setShowWorkspace}
        showEllipsoid={showEllipsoid}
        setShowEllipsoid={setShowEllipsoid}
        showGhost={showGhost}
        setShowGhost={setShowGhost}
        customStlGeometry={customStlGeometry}
        stlFileName={stlFileName}
        handleStlUpload={handleStlUpload}
        removeStlMesh={removeStlMesh}
        customLinkStls={customLinkStls}
        onUploadLinkStl={handleLinkStlUpload}
        onScaleLinkStl={updateLinkStlScale}
        onRemoveLinkStl={removeLinkStl}
        urdfMeshHints={urdfMeshHints}
        onFrameView={handleFrameView}
        viewPreset={viewPreset}
        onSelectViewPreset={handleSelectViewPreset}
        onAddWaypoint={addCurrentWaypoint}
        isPlaying={isPlaying}
        canPlayTrajectory={waypoints.length >= 2 && trajectoryData.totalDuration > 0}
        onTogglePlayback={toggleTrajectoryPlayback}
        canUndo={canUndo}
        canRedo={canRedo}
        onUndo={handleUndo}
        onRedo={handleRedo}
        toolType={toolType}
        setToolType={setToolType}
        toolOffsetRpy={toolOffsetRpy}
        setToolOffsetRpy={setToolOffsetRpy}
        defaultToolOffset={defaultToolOffset}
        toolGrip={toolGrip}
        setToolGrip={setToolGrip}
        toolScale={toolScale}
        setToolScale={setToolScale}
        toolFrame={toolFrame}
        modelStyle={modelStyle}
        setModelStyle={setModelStyle}
        modelTarget={modelTarget}
        setModelTarget={setModelTarget}
        numeric={numeric}
      />
      <div className="scene-readout">
        {activeCollision && <div className="collision-readout-pill">Bloqueio de colisão: {activeCollision.message}</div>}
        <span>Efetuador ({TOOL_TYPES[toolType].shortLabel}) · RPY [{fmt(toolFrame.orientation.rpy[0], 1)}°, {fmt(toolFrame.orientation.rpy[1], 1)}°, {fmt(toolFrame.orientation.rpy[2], 1)}°]</span>
        <output data-testid="end-position">X {fmt(fk.positions.at(-1)[0], 2)} · Y {fmt(fk.positions.at(-1)[1], 2)} · Z {fmt(fk.positions.at(-1)[2], 2)} u</output>
      </div>
    </div>
  </main>;
}

export default App;
