import { useCallback, useEffect, useRef, useState } from 'react';
import { cloneSnapshot } from '../storage';

export function useHistory({
  joints,
  toolType,
  toolOffsetRpy,
  toolGrip,
  toolScale,
  modelStyle,
  environment,
  waypoints,
  isContinuousMotion,
  onApplySnapshot,
  onMessage,
}) {
  const [historyState, setHistoryState] = useState({ canUndo: false, canRedo: false });
  const historyRef = useRef({ past: [], future: [], lastSnap: null, lastHash: '', skipRecord: false });

  const applySnapshotRef = useRef(onApplySnapshot);
  useEffect(() => { applySnapshotRef.current = onApplySnapshot; }, [onApplySnapshot]);

  const messageRef = useRef(onMessage);
  useEffect(() => { messageRef.current = onMessage; }, [onMessage]);

  useEffect(() => {
    if (isContinuousMotion) return;
    const snap = cloneSnapshot({
      joints,
      toolType,
      toolOffsetRpy,
      toolGrip,
      toolScale,
      modelStyle,
      environment,
      waypoints,
    });
    const hash = JSON.stringify(snap);
    const h = historyRef.current;
    if (!h.lastHash || h.skipRecord) {
      h.lastSnap = snap;
      h.lastHash = hash;
      h.skipRecord = false;
      return;
    }
    if (h.lastHash !== hash) {
      h.past = [...h.past.slice(-29), h.lastSnap];
      h.future = [];
      h.lastSnap = snap;
      h.lastHash = hash;
      setHistoryState({ canUndo: h.past.length > 0, canRedo: false });
    }
  }, [joints, toolType, toolOffsetRpy, toolGrip, toolScale, modelStyle, environment, waypoints, isContinuousMotion]);

  const handleUndo = useCallback(() => {
    const h = historyRef.current;
    if (h.past.length === 0) return;
    const prev = h.past[h.past.length - 1];
    h.past = h.past.slice(0, -1);
    if (h.lastSnap) h.future = [h.lastSnap, ...h.future.slice(0, 29)];
    setHistoryState({ canUndo: h.past.length > 0, canRedo: h.future.length > 0 });
    h.skipRecord = true;
    applySnapshotRef.current?.(prev);
    messageRef.current?.('Alteração desfeita.');
  }, []);

  const handleRedo = useCallback(() => {
    const h = historyRef.current;
    if (h.future.length === 0) return;
    const [next, ...rest] = h.future;
    h.future = rest;
    if (h.lastSnap) h.past = [...h.past.slice(-29), h.lastSnap];
    setHistoryState({ canUndo: h.past.length > 0, canRedo: h.future.length > 0 });
    h.skipRecord = true;
    applySnapshotRef.current?.(next);
    messageRef.current?.('Alteração refeita.');
  }, []);

  return {
    canUndo: historyState.canUndo,
    canRedo: historyState.canRedo,
    handleUndo,
    handleRedo,
  };
}
