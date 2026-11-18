import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { DEG, computeToolFrame, forwardKinematics, sweepJointsCollisionFree } from '../kinematics';
import { defaultWaypointsForJoints, evaluateTrajectory, sampleTrajectoryPath } from '../trajectory';

/**
 * Manages trajectory waypoints, polynomial/LIN path sampling, ghost preview, and 60 FPS playback.
 * @param {Object} params
 * @param {import('../types.js').Joint[]} params.initialJoints - Baseline preset joints for default waypoints and ghost fallback
 * @param {boolean} [params.showGhost] - Whether the initial-waypoint ghost silhouette is active
 * @param {import('../types.js').Joint[]} params.joints - Active DH joint array
 * @param {import('react').MutableRefObject<import('../types.js').Joint[]>} params.jointsRef - Ref mirroring latest joints for rAF loop
 * @param {Function} params.setJoints - Setter for robot joint states
 * @param {import('../types.js').ToolType} params.toolType - Active end-effector tool type
 * @param {import('react').MutableRefObject<import('../types.js').ToolType>} params.toolTypeRef - Ref mirroring active toolType
 * @param {import('../types.js').RpyDeg} params.toolOffsetRpy - Tool flange orientation offset (deg)
 * @param {import('../types.js').EnvironmentConfig} params.environment - Workbench floor and obstacle state
 * @param {import('react').MutableRefObject<import('../types.js').EnvironmentConfig>} params.environmentRef - Ref mirroring environment for rAF loop
 * @param {Function} params.setCollisionAlert - Setter for collision alert banner
 * @param {Function} params.setMessage - Setter for UI status banner
 * @param {Function} params.onSyncTarget - Callback updating IK Cartesian target on seek/scrub
 * @param {Function} params.onCommitJoints - Callback committing joint state on waypoint selection
 * @returns {Object} Trajectory state, derived profiles, and playback controls
 */
export function useTrajectoryPlayer({
  initialJoints,
  showGhost = false,
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
}) {
  const [waypoints, setWaypoints] = useState(() => defaultWaypointsForJoints(initialJoints));
  const [isPlaying, setIsPlaying] = useState(false);
  const [playTime, setPlayTime] = useState(0);
  const playTimeRef = useRef(playTime);
  useEffect(() => { playTimeRef.current = playTime; }, [playTime]);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);
  const [isLooping, setIsLooping] = useState(false);
  const [interpType, setInterpType] = useState('quintic');
  const [angleUnit, setAngleUnit] = useState('deg');
  const [showTrajectory, setShowTrajectory] = useState(true);

  const syncTargetRef = useRef(onSyncTarget);
  useEffect(() => { syncTargetRef.current = onSyncTarget; }, [onSyncTarget]);
  const commitJointsRef = useRef(onCommitJoints);
  useEffect(() => { commitJointsRef.current = onCommitJoints; }, [onCommitJoints]);

  const stopPlayback = useCallback(() => setIsPlaying(false), []);

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

  const trajectoryData = useMemo(
    () => sampleTrajectoryPath(waypoints, structuralJoints, undefined, interpType, environment),
    [waypoints, structuralJoints, interpType, environment]
  );

  const ghostJoints = useMemo(() => {
    if (!showGhost) return null;
    const refQ = waypoints[0]?.q;
    if (!Array.isArray(refQ)) return initialJoints.map(j => ({ ...j }));
    return joints.map((j, i) => ({ ...j, val: refQ[i] !== undefined ? refQ[i] : j.val }));
  }, [showGhost, waypoints, joints, initialJoints]);

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
      const blockedJoints = swept.joints.map(j => ({ ...j, speed: 0 }));
      setJoints(blockedJoints);
      const blockedFk = forwardKinematics(blockedJoints);
      const blockedTool = computeToolFrame(blockedFk.total, toolOffsetRpy, toolType);
      syncTargetRef.current?.({ position: blockedFk.positions.at(-1), rpy: blockedTool.orientation.rpy });
      setCollisionAlert(swept.collision);
      setMessage(`Trajetória interrompida: ${swept.collision.message}`);
      return;
    }
    setPlayTime(time);
    setJoints(candidate);
    const candidateFk = forwardKinematics(candidate);
    const candidateTool = computeToolFrame(candidateFk.total, toolOffsetRpy, toolType);
    syncTargetRef.current?.({ position: candidateFk.positions.at(-1), rpy: candidateTool.orientation.rpy });
    setCollisionAlert(null);
    setMessage('');
  }

  useEffect(() => {
    if (!isPlaying) return undefined;
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
  }, [
    isPlaying,
    playbackSpeed,
    isLooping,
    trajectoryData.totalDuration,
    waypoints,
    interpType,
    jointsRef,
    environmentRef,
    toolTypeRef,
    setJoints,
    setCollisionAlert,
    setMessage,
  ]);

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
      commitJointsRef.current?.(swept.joints);
      setCollisionAlert(swept.collision);
      setMessage(`Movimento bloqueado antes da colisão: ${swept.collision.message}`);
      return;
    }
    commitJointsRef.current?.(candidate);
    setMessage(`Manipulador posicionado em "${wp.name}".`);
  }

  function updateWaypointDuration(id, duration) {
    setWaypoints(prev => prev.map(w => (w.id === id ? { ...w, duration: Math.max(0.1, duration) } : w)));
  }

  function toggleTrajectoryPlayback() {
    if (waypoints.length < 2 || trajectoryData.totalDuration <= 0) return;
    if (!isPlaying && playTimeRef.current >= trajectoryData.totalDuration - 1e-4) {
      playTimeRef.current = 0;
      setPlayTime(0);
    }
    setIsPlaying(prev => !prev);
  }

  const waypointsRef = useRef(waypoints);
  const moveToWaypointRef = useRef(moveToWaypoint);
  useEffect(() => {
    waypointsRef.current = waypoints;
    moveToWaypointRef.current = moveToWaypoint;
  });

  const handleSelectWaypoint = useCallback((idx) => {
    const wp = waypointsRef.current[idx];
    if (wp) moveToWaypointRef.current(wp);
  }, []);

  return {
    waypoints,
    setWaypoints,
    isPlaying,
    setIsPlaying,
    stopPlayback,
    playTime,
    setPlayTime,
    playTimeRef,
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
  };
}
