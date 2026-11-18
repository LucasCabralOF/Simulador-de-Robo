import { useCallback, useEffect, useRef, useState } from 'react';

const ANALYSIS_IDS = new Set(['model', 'forward', 'differential', 'inverse', 'singularities', 'trajectory', 'workspace', 'export', 'report']);

export function usePanels(onClearMessage) {
  const [tab, setTab] = useState('model');
  const [openPanels, setOpenPanels] = useState([]);
  const [pinnedPanels, setPinnedPanels] = useState([]);
  const [panelFocusOrder, setPanelFocusOrder] = useState([]);

  const openPanelsRef = useRef(openPanels);
  useEffect(() => { openPanelsRef.current = openPanels; }, [openPanels]);

  const pinnedPanelsRef = useRef(pinnedPanels);
  useEffect(() => { pinnedPanelsRef.current = pinnedPanels; }, [pinnedPanels]);

  const activePanel = panelFocusOrder.filter(p => openPanels.includes(p)).at(-1) || openPanels.at(-1) || null;
  const activePanelRef = useRef(activePanel);
  useEffect(() => { activePanelRef.current = activePanel; }, [activePanel]);

  const clearMessageRef = useRef(onClearMessage);
  useEffect(() => { clearMessageRef.current = onClearMessage; }, [onClearMessage]);

  const toggleOrOpenPanel = useCallback((id, multi = false, isAnalysisTab = false) => {
    const currentlyOpen = openPanelsRef.current;
    if (currentlyOpen.includes(id)) {
      setOpenPanels(currentlyOpen.filter(p => p !== id));
      setPinnedPanels(prev => prev.filter(p => p !== id));
      setPanelFocusOrder(prev => prev.filter(p => p !== id));
      return;
    }
    clearMessageRef.current?.();
    if (isAnalysisTab) setTab(id);
    const currentPinned = pinnedPanelsRef.current;
    const kept = multi ? [...currentlyOpen] : currentlyOpen.filter(p => currentPinned.includes(p));
    const nextOpen = [...kept.slice(-2), id];
    setOpenPanels(nextOpen);
    setPanelFocusOrder(prev => [...prev.filter(p => nextOpen.includes(p) && p !== id), id]);
    setPinnedPanels(prev => {
      const base = multi && currentlyOpen.length > 0 ? Array.from(new Set([...prev, ...kept])) : prev;
      return base.filter(p => nextOpen.includes(p));
    });
  }, []);

  const handleSelectTab = useCallback((id, multi = false) => {
    toggleOrOpenPanel(id, multi, true);
  }, [toggleOrOpenPanel]);

  const handleTogglePanel = useCallback((id, multi = false) => {
    toggleOrOpenPanel(id, multi, false);
  }, [toggleOrOpenPanel]);

  const handleClosePanel = useCallback((id) => {
    const targetId = id || activePanelRef.current;
    if (!targetId) return;
    setOpenPanels(prev => prev.filter(p => p !== targetId));
    setPinnedPanels(prev => prev.filter(p => p !== targetId));
    setPanelFocusOrder(prev => prev.filter(p => p !== targetId));
  }, []);

  const handleTogglePinPanel = useCallback((id) => {
    if (!id) return;
    setPinnedPanels(prev => (prev.includes(id) ? prev.filter(p => p !== id) : [...prev, id]));
  }, []);

  const handleFocusPanel = useCallback((id) => {
    if (!id) return;
    setPanelFocusOrder(prev => (prev.at(-1) === id ? prev : [...prev.filter(p => p !== id), id]));
    if (ANALYSIS_IDS.has(id)) {
      setTab(id);
    }
  }, []);

  const showInverse = tab === 'inverse' || openPanels.includes('inverse');
  const showDifferential = tab === 'differential' || openPanels.includes('differential');
  const showSingularities = tab === 'singularities' || openPanels.includes('singularities');
  const showReport = tab === 'report' || openPanels.includes('report');
  const showExport = tab === 'export' || openPanels.includes('export');

  return {
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
  };
}
