import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  DEG,
  classifyPosture,
  computeToolFrame,
  detectAnalyticalArchitecture,
  detectSelfCollision,
  flangeRpyFromToolTarget,
  forwardKinematics,
  inverseKinematics,
  listPostureBranches,
  normalizeJoints,
  rotationFromRPY,
  solveBoundedTarget,
  validateJoints,
} from '../kinematics';
import { computeCartesianBounds } from '../workspace';

/**
 * Coordinates inverse kinematics targets, analytical posture selection, Cartesian slider drags,
 * adaptive slider bounds, and 3D XYZ/RPY TransformControls gizmo tracking.
 * @param {Object} params
 * @param {import('../types.js').CartesianTarget} params.initialTarget - Initial TCP target pose
 * @param {import('../types.js').Joint[]} params.joints - Active robot joints
 * @param {import('react').MutableRefObject<import('../types.js').Joint[]>} params.jointsRef - Ref mirroring latest joints during continuous drags
 * @param {Function} params.setJoints - Setter for robot joints
 * @param {import('../types.js').Vec3} params.currentEnd - Current flange end position
 * @param {{ orientation: { rpy: import('../types.js').RpyDeg, rotation: number[][] } }} params.toolFrame - Current computed tool frame
 * @param {import('../types.js').TaskMode} params.mode - Active IK task mode
 * @param {Function} params.setMode - Setter for task mode
 * @param {number} params.length - Reference length L for task scaling
 * @param {number} params.damping - DLS damping factor lambda
 * @param {import('../types.js').ToolType} params.toolType - Active tool type
 * @param {import('../types.js').RpyDeg} params.toolOffsetRpy - Tool mount offset angles (deg)
 * @param {number} params.toolScale - Tool 3D scale multiplier
 * @param {import('../types.js').EnvironmentConfig} params.environment - Floor and obstacle settings
 * @param {string} params.tab - Active analysis tab ID
 * @param {boolean} params.showInverse - Whether the Inversa panel or tab is active
 * @param {{ min: import('../types.js').Vec3, max: import('../types.js').Vec3, span: import('../types.js').Vec3 }} params.globalCartesianBounds - Structural workspace envelope
 * @param {boolean} params.isPointerInteracting - Whether any slider/canvas pointer interaction is ongoing
 * @param {boolean} params.isPlaying - Whether trajectory playback is active
 * @param {Function} params.stopPlayback - Callback pausing active trajectory playback
 * @param {Function} params.setCollisionAlert - Setter for collision alert banner
 * @param {Function} params.setMessage - Setter for UI status banner
 * @returns {Object} Inverse control states, derived bounds/posture info, and interaction handlers
 */
export function useInverseControl({
  initialTarget,
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
}) {
  const [target, setTarget] = useState(() => ({
    position: [...initialTarget.position],
    rpy: [...initialTarget.rpy],
  }));
  const [solution, setSolution] = useState(null);
  const [posture, setPosture] = useState('auto');
  const [autoSolveIk, setAutoSolveIk] = useState(false);
  const [ikWaypointDuration, setIkWaypointDuration] = useState(2.0);
  const [showGizmo, setShowGizmo] = useState(true);
  const [gizmoMode, setGizmoMode] = useState('translate');
  const [gizmoLockOrientation, setGizmoLockOrientation] = useState(false);
  const [gizmoLockPosition, setGizmoLockPosition] = useState(true);
  const [isGizmoDragging, setIsGizmoDragging] = useState(false);
  const [isIkSliderDragging, setIsIkSliderDragging] = useState(false);

  const gizmoAnchorPosRef = useRef(null);
  const gizmoAnchorRpyRef = useRef(null);
  const multiSeedFailRef = useRef(null);

  const handleIkSliderDragStart = useCallback(() => {
    multiSeedFailRef.current = null;
    setIsIkSliderDragging(true);
  }, []);
  const handleIkSliderDragEnd = useCallback(() => setIsIkSliderDragging(false), []);

  useEffect(() => {
    if (!isIkSliderDragging) return undefined;
    const stop = () => setIsIkSliderDragging(false);
    window.addEventListener('pointerup', stop);
    window.addEventListener('pointercancel', stop);
    return () => {
      window.removeEventListener('pointerup', stop);
      window.removeEventListener('pointercancel', stop);
    };
  }, [isIkSliderDragging]);

  const isContinuousMotion = isGizmoDragging || isIkSliderDragging || isPointerInteracting || isPlaying;

  const [frozenSnapshot, setFrozenSnapshot] = useState(() => ({
    joints,
    currentEnd,
    target,
    mode,
    environment,
    length,
    toolType,
    toolOffsetRpy,
  }));

  useEffect(() => {
    if (isContinuousMotion) return undefined;
    let cancelled = false;
    queueMicrotask(() => {
      if (cancelled) return;
      setFrozenSnapshot((prev) => (
        prev.joints === joints
        && prev.currentEnd === currentEnd
        && prev.target === target
        && prev.mode === mode
        && prev.environment === environment
        && prev.length === length
        && prev.toolType === toolType
        && prev.toolOffsetRpy === toolOffsetRpy
          ? prev
          : {
              joints,
              currentEnd,
              target,
              mode,
              environment,
              length,
              toolType,
              toolOffsetRpy,
            }
      ));
    });
    return () => { cancelled = true; };
  }, [isContinuousMotion, joints, currentEnd, target, mode, environment, length, toolType, toolOffsetRpy]);

  const frozenForMotion = isContinuousMotion && frozenSnapshot.joints.length === joints.length
    ? frozenSnapshot
    : null;

  const boundsJoints = frozenForMotion ? frozenForMotion.joints : joints;
  const boundsEnd = frozenForMotion ? frozenForMotion.currentEnd : currentEnd;
  const boundsMode = frozenForMotion ? frozenForMotion.mode : mode;
  const boundsEnv = frozenForMotion ? frozenForMotion.environment : environment;

  const cartesianBounds = useMemo(() => {
    if (!showInverse) return globalCartesianBounds;
    return computeCartesianBounds(boundsJoints, boundsEnd, boundsMode, boundsEnv);
  }, [showInverse, globalCartesianBounds, boundsJoints, boundsEnd, boundsMode, boundsEnv]);

  const postureJoints = frozenForMotion ? frozenForMotion.joints : joints;
  const postureTarget = frozenForMotion ? frozenForMotion.target : target;
  const postureMode = frozenForMotion ? frozenForMotion.mode : mode;
  const postureLength = frozenForMotion ? frozenForMotion.length : length;
  const postureToolType = frozenForMotion ? frozenForMotion.toolType : toolType;
  const postureToolOffsetRpy = frozenForMotion ? frozenForMotion.toolOffsetRpy : toolOffsetRpy;
  const postureEnv = frozenForMotion ? frozenForMotion.environment : environment;

  const basePostureInfo = useMemo(() => {
    if (!showInverse) return null;
    return listPostureBranches(postureJoints, postureTarget, {
      mode: postureMode,
      length: postureLength,
      toolType: postureToolType,
      toolOffsetRpy: postureToolOffsetRpy,
      environment: postureEnv,
    });
  }, [showInverse, postureJoints, postureTarget, postureMode, postureLength, postureToolType, postureToolOffsetRpy, postureEnv]);

  const postureInfo = useMemo(() => {
    if (!basePostureInfo) return null;
    if (!frozenForMotion) return basePostureInfo;
    const liveArch = detectAnalyticalArchitecture(joints);
    if (!liveArch || liveArch !== basePostureInfo.architecture) return basePostureInfo;
    return {
      ...basePostureInfo,
      currentPosture: classifyPosture(joints, liveArch),
    };
  }, [basePostureInfo, frozenForMotion, joints]);

  const targetDistance = mode === 'xy'
    ? Math.hypot(target.position[0] - currentEnd[0], target.position[1] - currentEnd[1])
    : Math.hypot(target.position[0] - currentEnd[0], target.position[1] - currentEnd[1], target.position[2] - currentEnd[2]);

  const toolAngularErrorDeg = useMemo(() => {
    if (!showInverse) return 0;
    const rCurr = toolFrame.orientation.rotation;
    const d = rotationFromRPY(target.rpy).elements;
    const trace =
      rCurr[0][0] * d[0] + rCurr[1][0] * d[1] + rCurr[2][0] * d[2]
      + rCurr[0][1] * d[4] + rCurr[1][1] * d[5] + rCurr[2][1] * d[6]
      + rCurr[0][2] * d[8] + rCurr[1][2] * d[9] + rCurr[2][2] * d[10];
    const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
    return Math.acos(cosTheta) / DEG;
  }, [showInverse, toolFrame, target.rpy]);

  function solveInverse(targetToSolve = target, modeToUse = mode, jointsToUse = joints, maxIterations = undefined, postureToUse = posture) {
    stopPlayback();
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
      posture: postureToUse,
    });
    if (res.converged && validateJoints(res.joints).length === 0) {
      const normalized = normalizeJoints(res.joints).map(j => ({ ...j, speed: 0 }));
      jointsRef.current = normalized;
      setJoints(normalized);
      setSolution({ ...res, joints: normalized, applied: true });
      setCollisionAlert(null);
      setMessage('');
      if (modeToUse !== 'pose' && (res.solver === 'dls' || res.orientationAdjusted)) {
        const nextFk = forwardKinematics(normalized);
        const nextTool = computeToolFrame(nextFk.total, toolOffsetRpy, toolType);
        setTarget(prev => ({
          ...prev,
          rpy: nextTool.orientation.rpy.map(v => Math.round(v * 10) / 10),
        }));
      }
    } else {
      setSolution({ ...res, applied: false });
      setMessage('');
    }
    return res;
  }

  function selectPosture(nextPosture) {
    setPosture(nextPosture);
    solveInverse(target, mode, joints, undefined, nextPosture);
  }

  function applyPostureBranch(branch) {
    if (!branch || !Array.isArray(branch.joints)) return;
    if (validateJoints(branch.joints).length !== 0) return;
    if (detectSelfCollision(branch.joints, null, toolType, environment).colliding) return;
    stopPlayback();
    const normalized = normalizeJoints(branch.joints).map(j => ({ ...j, speed: 0 }));
    jointsRef.current = normalized;
    setJoints(normalized);
    setPosture(branch.posture || 'auto');
    setCollisionAlert(null);
    setMessage('');
    setSolution({ ...branch, joints: normalized, applied: true });
    if (mode !== 'pose' && branch.orientationAdjusted) {
      const nextFk = forwardKinematics(normalized);
      const nextTool = computeToolFrame(nextFk.total, toolOffsetRpy, toolType);
      setTarget(prev => ({
        ...prev,
        rpy: nextTool.orientation.rpy.map(v => Math.round(v * 10) / 10),
      }));
    }
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

  function applyOrientationPreset(nextRpy) {
    const nextTarget = { ...target, rpy: nextRpy };
    if (mode !== 'pose') setMode('pose');
    setTarget(nextTarget);
    solveInverse(nextTarget, 'pose');
  }

  function changeTarget(key, index, value) {
    const nextTarget = { ...target, [key]: target[key].map((v, i) => (i === index ? value : v)) };
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
    stopPlayback();
    const activeJoints = jointsRef.current || joints;
    const activeEnd = forwardKinematics(activeJoints).positions.at(-1);
    const fixedTarget = { ...target, [key]: target[key].map((v, i) => (i === index ? value : v)) };
    const nextMode = key === 'rpy' && mode !== 'pose' ? 'pose' : mode;
    if (nextMode !== mode) setMode(nextMode);

    if (key === 'rpy') {
      const anchorPos = [...activeEnd];
      const bounded = solveBoundedTarget(
        activeJoints,
        { position: anchorPos, rpy: fixedTarget.rpy },
        { mode: 'pose', length, damping, maxIterations: 45, toolType, toolOffsetRpy, environment, lockPosition: gizmoLockPosition }
      );
      if (!bounded.blocked) {
        jointsRef.current = bounded.joints;
        setJoints(bounded.joints);
        setCollisionAlert(null);
      }
      setSolution({ ...bounded, applied: bounded.converged && !bounded.blocked });
      setMessage('');
      const outPos = (gizmoLockPosition ? anchorPos : bounded.position).map(v => Math.round(v * 1000) / 1000);
      setTarget({ position: outPos, rpy: bounded.rpy.map(v => Math.round(v * 10) / 10) });
      return;
    }

    const isValidSolved = attempt => (
      attempt
      && attempt.converged
      && validateJoints(attempt.joints).length === 0
      && !detectSelfCollision(attempt.joints, null, toolType, environment).colliding
    );

    const flangeTarget = {
      ...fixedTarget,
      rpy: flangeRpyFromToolTarget(fixedTarget.rpy, toolOffsetRpy),
    };
    const fallbackMode = nextMode === 'xy' ? 'xy' : (nextMode === 'pose' && gizmoLockOrientation ? 'pose' : 'position');

    // 1. Continuous same-branch tracking (single warm seed from jointsRef)
    let solvedMode = nextMode;
    let res = inverseKinematics(activeJoints, flangeTarget, {
      mode: nextMode,
      length,
      damping,
      maxIterations: 45,
      maxSeeds: 1,
      toolType,
      environment,
      posture,
      strictPosture: false,
      continuityToleranceDeg: 95,
    });
    if (!isValidSolved(res) && nextMode === 'pose' && !gizmoLockOrientation) {
      const posAttempt = inverseKinematics(activeJoints, flangeTarget, {
        mode: 'position',
        length,
        damping,
        maxIterations: 45,
        maxSeeds: 1,
        toolType,
        environment,
      });
      if (isValidSolved(posAttempt) || posAttempt.error < res.error) {
        res = posAttempt;
        solvedMode = 'position';
      }
    }

    // 2. Smooth 360-deg joint-wrap escape when a full-circle joint hits +-180 deg seam
    if (!isValidSolved(res)) {
      let hasWrapped = false;
      const wrapSeed = activeJoints.map(jt => {
        if (jt.type === 'R' && jt.max - jt.min >= 359) {
          if (jt.val >= jt.max - 5) {
            hasWrapped = true;
            return { ...jt, val: jt.val - 355 };
          }
          if (jt.val <= jt.min + 5) {
            hasWrapped = true;
            return { ...jt, val: jt.val + 355 };
          }
        }
        return jt;
      });
      if (hasWrapped) {
        const wrapAttempt = inverseKinematics(wrapSeed, flangeTarget, {
          mode: fallbackMode,
          length,
          damping,
          maxIterations: 45,
          maxSeeds: 1,
          toolType,
          environment,
        });
        if (isValidSolved(wrapAttempt)) {
          res = wrapAttempt;
          solvedMode = fallbackMode;
        }
      }
    }

    // 3. Fast 1D boundary bisection on the current branch (cheap, single warm seed)
    let applied = isValidSolved(res) ? res : null;
    let reachedCoord = applied ? value : activeEnd[index];
    if (!applied) {
      let lo = activeEnd[index];
      let hi = value;
      let bestRes = null;
      for (let stepIdx = 0; stepIdx < 6; stepIdx++) {
        const mid = (lo + hi) / 2;
        const candTarget = {
          ...flangeTarget,
          position: flangeTarget.position.map((v, i) => (i === index ? mid : v)),
        };
        const attempt = inverseKinematics(bestRes ? bestRes.joints : activeJoints, candTarget, {
          mode: fallbackMode,
          length,
          damping,
          maxIterations: 25,
          maxSeeds: 1,
          toolType,
          environment,
        });
        if (isValidSolved(attempt)) {
          bestRes = attempt;
          lo = mid;
        } else {
          hi = mid;
        }
      }
      if (bestRes) {
        applied = bestRes;
        reachedCoord = lo;
      }
    }

    // 4. Controlled branch transition across an interior singular / fold band
    const remainingGap = Math.abs(value - reachedCoord);
    const lastFail = multiSeedFailRef.current;
    const recentlyFailedHere = lastFail && lastFail.index === index && Math.abs(lastFail.value - value) < 0.15;
    if (remainingGap > 0.08 && !recentlyFailedHere) {
      const multiAttempt = inverseKinematics(activeJoints, flangeTarget, {
        mode: fallbackMode,
        length,
        damping,
        maxIterations: 40,
        maxSeeds: 4,
        toolType,
        environment,
      });
      if (isValidSolved(multiAttempt)) {
        applied = multiAttempt;
        solvedMode = fallbackMode;
        multiSeedFailRef.current = null;
      } else {
        multiSeedFailRef.current = { index, value };
      }
    }

    let outRpy = fixedTarget.rpy;
    if (applied) {
      const normalized = normalizeJoints(applied.joints).map(j => ({ ...j, speed: 0 }));
      jointsRef.current = normalized;
      setJoints(normalized);
      setCollisionAlert(null);
      setSolution({ ...applied, joints: normalized, applied: true });
      setMessage('');
      if (solvedMode !== 'pose' && mode !== 'pose') {
        const nextFk = forwardKinematics(normalized);
        outRpy = computeToolFrame(nextFk.total, toolOffsetRpy, toolType, toolScale)
          .orientation.rpy.map(v => Math.round(v * 10) / 10);
      }
    } else {
      setSolution({ ...res, applied: false });
      setMessage('');
    }

    setTarget({
      position: fixedTarget.position,
      rpy: outRpy,
    });
  }

  function handleGizmoDragStart() {
    stopPlayback();
    jointsRef.current = joints;
    const startFk = forwardKinematics(joints);
    const startEnd = startFk.positions.at(-1);
    const startRpy = computeToolFrame(startFk.total, toolOffsetRpy, toolType, toolScale).orientation.rpy;
    gizmoAnchorPosRef.current = [...startEnd];
    gizmoAnchorRpyRef.current = [...startRpy];
    if (tab !== 'inverse' || isPlaying) {
      setTarget({ position: [...startEnd], rpy: [...startRpy] });
    }
    setIsGizmoDragging(true);
  }

  function handleGizmoDragEnd() {
    setIsGizmoDragging(false);
    gizmoAnchorPosRef.current = null;
    gizmoAnchorRpyRef.current = null;
    const activeFk = forwardKinematics(jointsRef.current);
    const activeEnd = activeFk.positions.at(-1).map(v => Math.round(v * 1000) / 1000);
    const activeToolRpy = computeToolFrame(activeFk.total, toolOffsetRpy, toolType, toolScale).orientation.rpy.map(v => Math.round(v * 10) / 10);
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
    const canLockOrientation = activeJoints.filter(j => j.type === 'R').length >= 2;

    if (position) {
      const requestedPos = position.map((v, i) => (mode === 'xy' && i === 2 ? activeEnd[2] : v));
      const lockedRpy = gizmoAnchorRpyRef.current || toolFrame.orientation.rpy;
      const shouldLockOri = gizmoLockOrientation && canLockOrientation;
      const translateMode = shouldLockOri ? 'pose' : (mode === 'xy' ? 'xy' : 'position');
      const bounded = solveBoundedTarget(
        activeJoints,
        { position: requestedPos, rpy: lockedRpy },
        { mode: translateMode, length, damping, maxIterations: 45, toolType, toolOffsetRpy, environment, lockOrientation: shouldLockOri }
      );
      const nextPos = bounded.position.map((v, i) =>
        (mode === 'xy' && i === 2 ? activeEnd[2] : Math.round(v * 1000) / 1000)
      );
      if (!bounded.blocked) {
        jointsRef.current = bounded.joints;
        gizmoAnchorPosRef.current = [...bounded.position];
        if (!shouldLockOri) {
          gizmoAnchorRpyRef.current = [...bounded.rpy];
        }
        setJoints(bounded.joints);
        setCollisionAlert(null);
      }
      setSolution({ ...bounded, applied: bounded.converged && !bounded.blocked });
      setMessage('');
      setTarget(prev => ({
        ...prev,
        position: nextPos,
        rpy: (shouldLockOri ? lockedRpy : bounded.rpy).map(v => Math.round(v * 10) / 10),
      }));
      return { position: bounded.position };
    }

    if (rpy) {
      if (mode !== 'pose') setMode('pose');
      const lockedPos = gizmoAnchorPosRef.current || activeEnd;
      const bounded = solveBoundedTarget(
        activeJoints,
        { position: lockedPos, rpy },
        { mode: 'pose', length, damping, maxIterations: 45, toolType, toolOffsetRpy, environment, lockPosition: gizmoLockPosition }
      );
      const resultPos = gizmoLockPosition ? lockedPos : bounded.position;
      const nextPos = resultPos.map(v => Math.round(v * 1000) / 1000);
      const nextRpy = bounded.rpy.map(v => Math.round(v * 10) / 10);
      if (!bounded.blocked) {
        jointsRef.current = bounded.joints;
        gizmoAnchorRpyRef.current = [...bounded.rpy];
        if (!gizmoLockPosition) {
          gizmoAnchorPosRef.current = [...bounded.position];
        }
        setJoints(bounded.joints);
        setCollisionAlert(null);
      }
      setSolution({ ...bounded, applied: bounded.converged && !bounded.blocked });
      setMessage('');
      setTarget({ position: nextPos, rpy: nextRpy });
      return { position: resultPos, rpy: nextRpy };
    }
    return undefined;
  }

  const handleGizmoModeChange = useCallback((nextGizmoMode) => {
    setGizmoMode(nextGizmoMode);
    if (nextGizmoMode === 'rotate' && mode !== 'pose') {
      setMode('pose');
      setSolution(null);
    }
  }, [mode, setMode]);

  const ikTargetForScene = useMemo(() => {
    if (!showGizmo && tab !== 'inverse') return null;
    if (isPlaying || tab !== 'inverse') return currentEnd;
    return [target.position[0], target.position[1], mode === 'xy' ? currentEnd[2] : target.position[2]];
  }, [showGizmo, tab, isPlaying, currentEnd, target.position, mode]);

  const ikRpyForScene = (isPlaying || tab !== 'inverse') ? toolFrame.orientation.rpy : target.rpy;

  return {
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
    isGizmoDragging,
    isIkSliderDragging,
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
  };
}
