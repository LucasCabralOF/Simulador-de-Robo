import { memo, useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import {
  Activity,
  AlertTriangle,
  Code2,
  Compass,
  Cpu,
  Crosshair,
  FileText,
  FolderOpen,
  Maximize2,
  Minimize2,
  Minus,
  Orbit,
  PanelLeft,
  Pin,
  Redo2,
  RotateCcw,
  Route,
  SlidersHorizontal,
  Undo2,
  Wrench,
  X,
} from 'lucide-react';
import { Button, Notice } from './ui';
import { DEFAULT_MODEL_STYLE, isDefaultModelStyle } from './kinematics';
import ModelingPanel from './tabs/ModelingPanel';
import ToolPanel from './tabs/ToolPanel';
import SceneViewportDock from './SceneViewportDock';

const ANALYSIS_TABS = [
  { id: 'model', label: 'Modelo', title: 'Modelo e Atuadores', size: 'wide', icon: Cpu },
  { id: 'forward', label: 'Direta', title: 'Cinemática Direta', size: 'wide', icon: Compass },
  { id: 'differential', label: 'Diferencial', title: 'Cinemática Diferencial e Estática', size: 'wide', icon: Activity },
  { id: 'inverse', label: 'Inversa', title: 'Cinemática Inversa (IK)', size: 'medium', icon: Crosshair },
  { id: 'singularities', label: 'Singularidades', title: 'Mapa e Análise de Singularidades', size: 'wide', icon: AlertTriangle },
  { id: 'trajectory', label: 'Trajetória', title: 'Planejamento de Trajetória', size: 'wide', icon: Route },
  { id: 'workspace', label: 'Alcance 3D', title: 'Espaço de Trabalho 3D', size: 'compact', icon: Orbit },
  { id: 'export', label: 'Código', title: 'Exportação de Código e URDF', size: 'wide', icon: Code2 },
  { id: 'report', label: 'Relatório', title: 'Relatório Acadêmico', size: 'wide', icon: FileText },
];

const POPOVER_META = {
  ...Object.fromEntries(ANALYSIS_TABS.map(item => [item.id, item])),
  projects: { id: 'projects', title: 'Projetos, Presets e Arquivos', size: 'medium' },
  modeling: { id: 'modeling', title: 'Modelagem 3D (hastes e juntas)', size: 'compact' },
  tool: { id: 'tool', title: 'Ferramenta do efetuador (TCP)', size: 'compact' },
};

const POPOVER_DEFAULT_WIDTHS = {
  compact: 404,
  medium: 460,
  wide: 560,
};

const DEFAULT_WINDOW_STATE = {
  layout: 'floating',
  pos: { x: 0, y: 0 },
  size: null,
  minimized: false,
};

const TOOL_BADGE_LABELS = {
  gripper: 'Garra',
  vacuum: 'Ventosa',
  welder: 'Tocha',
};

const SceneToolbar = memo(function SceneToolbar({
  joints,
  tab,
  activePanel,
  openPanels = [],
  pinnedPanels = [],
  panelFocusOrder = [],
  onSelectTab,
  onTogglePanel,
  onClosePanel,
  onTogglePinPanel,
  onFocusPanel,
  renderPanelContent,
  message,
  activeCollision,
  showFrames,
  setShowFrames,
  environment,
  applyEnvironment,
  showGizmo,
  setShowGizmo,
  gizmoMode,
  onChangeGizmoMode,
  gizmoLockOrientation,
  setGizmoLockOrientation,
  gizmoLockPosition,
  setGizmoLockPosition,
  hasTrajectory,
  showTrajectory,
  setShowTrajectory,
  hasWorkspace,
  showWorkspace,
  setShowWorkspace,
  showEllipsoid,
  setShowEllipsoid,
  showGhost,
  setShowGhost,
  customStlGeometry,
  stlFileName,
  handleStlUpload,
  removeStlMesh,
  onFrameView,
  viewPreset,
  onSelectViewPreset,
  onAddWaypoint,
  isPlaying,
  canPlayTrajectory,
  onTogglePlayback,
  canUndo,
  canRedo,
  onUndo,
  onRedo,
  toolType,
  setToolType,
  toolOffsetRpy,
  setToolOffsetRpy,
  defaultToolOffset,
  toolGrip,
  setToolGrip,
  toolScale = 1,
  setToolScale,
  toolFrame,
  modelStyle,
  setModelStyle,
  modelTarget,
  setModelTarget,
  numeric,
}) {
  const [windowStates, setWindowStates] = useState({});
  const [dragState, setDragState] = useState(null); // { panelId, type: 'move' | 'resize' } | null
  const workspaceRowRef = useRef(null);
  const popoverRefs = useRef({});
  const prevOpenPanelsRef = useRef([]);

  useLayoutEffect(() => {
    const prevOpen = prevOpenPanelsRef.current;
    prevOpenPanelsRef.current = openPanels;
    const wsWidth = workspaceRowRef.current?.getBoundingClientRect().width || 1000;
    setWindowStates(prev => {
      const next = {};
      for (const id of openPanels) {
        if (prev[id] && prevOpen.includes(id)) {
          next[id] = prev[id];
        }
      }
      for (const id of openPanels) {
        if (next[id]) continue;
        const others = Object.values(next);
        if (others.length === 0) {
          next[id] = { ...DEFAULT_WINDOW_STATE, pos: { x: 0, y: 0 } };
          continue;
        }
        const meta = POPOVER_META[id] || POPOVER_META.model;
        const defaultW = Math.min(POPOVER_DEFAULT_WIDTHS[meta.size] || 460, wsWidth);
        const leftOccupied = others.some(w => w.layout === 'dock-left' || w.pos.x < 160);
        const rightOccupied = others.some(w => w.pos.x > wsWidth / 2 - 80);
        let pos = { x: 0, y: 0 };
        if (leftOccupied && !rightOccupied) {
          pos = { x: Math.max(36, Math.round(wsWidth - defaultW)), y: 0 };
        } else if (leftOccupied && rightOccupied) {
          pos = { x: Math.max(24, Math.round((wsWidth - defaultW) / 2)), y: 28 };
        }
        next[id] = { ...DEFAULT_WINDOW_STATE, pos };
      }
      return next;
    });
  }, [openPanels]);

  useEffect(() => {
    if (!activePanel) return undefined;
    const handleKeyDown = (e) => {
      if (e.key !== 'Escape' || e.defaultPrevented || document.querySelector('.ui-select-dropdown')) return;
      onClosePanel(activePanel);
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [activePanel, onClosePanel]);

  useEffect(() => {
    if (openPanels.length === 0) return undefined;
    const clampToWorkspace = () => {
      const ws = workspaceRowRef.current;
      if (!ws) return;
      const wsRect = ws.getBoundingClientRect();
      setWindowStates(prev => {
        let changed = false;
        const next = { ...prev };
        for (const id of openPanels) {
          const cur = prev[id];
          const pop = popoverRefs.current[id];
          if (!cur || cur.layout !== 'floating' || !pop) continue;
          const popRect = pop.getBoundingClientRect();
          const maxX = Math.max(0, wsRect.width - popRect.width);
          const maxY = Math.max(0, wsRect.height - Math.min(popRect.height, 80));
          const nx = Math.max(0, Math.min(cur.pos.x, maxX));
          const ny = Math.max(0, Math.min(cur.pos.y, maxY));
          if (nx !== cur.pos.x || ny !== cur.pos.y) {
            next[id] = { ...cur, pos: { x: nx, y: ny } };
            changed = true;
          }
        }
        return changed ? next : prev;
      });
    };
    window.addEventListener('resize', clampToWorkspace);
    return () => window.removeEventListener('resize', clampToWorkspace);
  }, [openPanels]);

  const updateWindow = (panelId, updater) => {
    setWindowStates(prev => {
      const cur = prev[panelId] || DEFAULT_WINDOW_STATE;
      const patch = typeof updater === 'function' ? updater(cur) : updater;
      return { ...prev, [panelId]: { ...cur, ...patch } };
    });
  };

  const startDrag = (panelId, e) => {
    const win = windowStates[panelId] || DEFAULT_WINDOW_STATE;
    if (win.layout !== 'floating' || e.button !== 0) return;
    if (e.target.closest('button, input, select, a, label')) return;
    const ws = workspaceRowRef.current;
    const pop = popoverRefs.current[panelId];
    if (!ws || !pop) return;
    e.preventDefault();
    onFocusPanel?.(panelId);
    const wsRect = ws.getBoundingClientRect();
    const popRect = pop.getBoundingClientRect();
    const startX = e.clientX;
    const startY = e.clientY;
    const origX = win.pos.x;
    const origY = win.pos.y;
    const maxX = Math.max(0, wsRect.width - popRect.width);
    const maxY = Math.max(0, wsRect.height - Math.min(popRect.height, 80));
    setDragState({ panelId, type: 'move' });

    const onMove = (moveEvent) => {
      const nx = Math.max(0, Math.min(maxX, Math.round(origX + (moveEvent.clientX - startX))));
      const ny = Math.max(0, Math.min(maxY, Math.round(origY + (moveEvent.clientY - startY))));
      updateWindow(panelId, { pos: { x: nx, y: ny } });
    };
    const onEnd = () => {
      setDragState(null);
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onEnd);
      window.removeEventListener('pointercancel', onEnd);
    };
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onEnd);
    window.addEventListener('pointercancel', onEnd);
  };

  const startResize = (panelId, dir, e) => {
    const win = windowStates[panelId] || DEFAULT_WINDOW_STATE;
    if (e.button !== 0 || win.layout === 'fullscreen') return;
    const ws = workspaceRowRef.current;
    const pop = popoverRefs.current[panelId];
    if (!ws || !pop) return;
    e.preventDefault();
    e.stopPropagation();
    onFocusPanel?.(panelId);
    const wsRect = ws.getBoundingClientRect();
    const popRect = pop.getBoundingClientRect();
    const startX = e.clientX;
    const startY = e.clientY;
    const startW = popRect.width;
    const startH = popRect.height;
    const offsetX = win.layout === 'floating' ? win.pos.x : 0;
    const offsetY = win.layout === 'floating' ? win.pos.y : 0;
    const minW = Math.min(320, Math.max(220, wsRect.width - offsetX));
    const maxW = Math.max(minW, wsRect.width - offsetX);
    const minH = Math.min(220, Math.max(160, wsRect.height - offsetY));
    const maxH = Math.max(minH, wsRect.height - offsetY);
    setDragState({ panelId, type: 'resize' });

    const onMove = (moveEvent) => {
      const dx = moveEvent.clientX - startX;
      const dy = moveEvent.clientY - startY;
      updateWindow(panelId, cur => {
        const nextSize = { ...(cur.size || {}) };
        if (dir === 'e' || dir === 'se') {
          nextSize.width = Math.round(Math.max(minW, Math.min(maxW, startW + dx)));
        }
        if (dir === 's' || dir === 'se') {
          nextSize.height = Math.round(Math.max(minH, Math.min(maxH, startH + dy)));
        }
        return { size: nextSize };
      });
    };
    const onEnd = () => {
      setDragState(null);
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onEnd);
      window.removeEventListener('pointercancel', onEnd);
    };
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onEnd);
    window.addEventListener('pointercancel', onEnd);
  };

  const activeTarget = modelTarget !== 'global' && joints.some(j => String(j.id) === modelTarget)
    ? modelTarget
    : 'global';
  const targetIdx = joints.findIndex(j => String(j.id) === activeTarget);
  const targetJoint = targetIdx >= 0 ? joints[targetIdx] : null;
  const overrideCount = modelStyle.perJoint ? Object.keys(modelStyle.perJoint).length : 0;
  const modelingBadge = targetJoint ? `J${targetIdx + 1}` : (overrideCount > 0 ? `${overrideCount}×` : null);
  const isDefaultTool = toolType === 'gripper'
    && toolOffsetRpy.every((v, idx) => Math.abs(v - defaultToolOffset[idx]) < 1e-6)
    && toolGrip === 50
    && Math.abs(toolScale - 1) < 1e-6;
  const chainSummary = `${joints.map(j => j.type).join('')} · ${joints.length} juntas`;

  const onToggleModeling = useCallback(() => onTogglePanel('modeling'), [onTogglePanel]);
  const onClearJointSelection = useCallback(() => setModelTarget('global'), [setModelTarget]);

  return (
    <>
      <div className="scene-toolbar-shell">
        <div className="scene-toolbar" role="toolbar" aria-label="Controles da cena 3D">
          <div className="scene-toolbar-group scene-toolbar-left" role="group" aria-label="Projeto e Cadeia">
            <header className="scene-brand" title={`Manipulador ${chainSummary}`}>
              <h1 className="visually-hidden">Cinemática DH</h1>
              <span className="scene-brand-meta">
                <span className="visually-hidden">Manipulador </span>
                {chainSummary}
              </span>
            </header>
            <Button
              type="button"
              className={`scene-toolbar-trigger${openPanels.includes('projects') ? ' is-active' : ''}`}
              aria-expanded={openPanels.includes('projects')}
              aria-controls="scene-popover-projects"
              title="Projetos, Presets e Arquivos · Ctrl+Clique para manter outras abertas"
              onClick={e => onTogglePanel('projects', Boolean(e.ctrlKey || e.metaKey || e.shiftKey))}
            >
              <FolderOpen size={14} aria-hidden="true" />
              <span>Projetos</span>
            </Button>
            <Button
              icon
              type="button"
              className="scene-toolbar-icon-btn"
              disabled={!canUndo}
              title="Desfazer alteração (Ctrl+Z)"
              aria-label="Desfazer"
              onClick={onUndo}
            >
              <Undo2 size={14} aria-hidden="true" />
            </Button>
            <Button
              icon
              type="button"
              className="scene-toolbar-icon-btn"
              disabled={!canRedo}
              title="Refazer alteração (Ctrl+Y)"
              aria-label="Refazer"
              onClick={onRedo}
            >
              <Redo2 size={14} aria-hidden="true" />
            </Button>
          </div>

          <span className="scene-toolbar-sep" role="separator" aria-orientation="vertical" />

          <nav className="scene-toolbar-nav" aria-label="Análises">
            {ANALYSIS_TABS.map(item => {
              const Icon = item.icon;
              const isOpen = openPanels.includes(item.id);
              return (
                <button
                  key={item.id}
                  type="button"
                  className={`scene-toolbar-tab${tab === item.id ? ' is-context' : ''}${isOpen ? ' is-open' : ''}`}
                  aria-current={tab === item.id ? 'page' : undefined}
                  aria-expanded={isOpen}
                  aria-controls={`scene-popover-${item.id}`}
                  title={`${item.title} · Ctrl+Clique para abrir em paralelo`}
                  onClick={e => onSelectTab(item.id, Boolean(e.ctrlKey || e.metaKey || e.shiftKey))}
                >
                  <Icon size={13} aria-hidden="true" />
                  <span>{item.label}</span>
                </button>
              );
            })}
          </nav>

          <span className="scene-toolbar-sep" role="separator" aria-orientation="vertical" />

          <div className="scene-toolbar-group scene-toolbar-actions" role="group" aria-label="Geometria e Efetuador">
            <Button
              type="button"
              className={`scene-toolbar-trigger${openPanels.includes('modeling') ? ' is-active' : ''}`}
              aria-expanded={openPanels.includes('modeling')}
              aria-controls="scene-popover-modeling"
              title="Modelagem 3D · Ctrl+Clique para abrir em paralelo"
              onClick={e => onTogglePanel('modeling', Boolean(e.ctrlKey || e.metaKey || e.shiftKey))}
            >
              <SlidersHorizontal size={14} aria-hidden="true" />
              <span>Modelagem 3D</span>
              {modelingBadge && <span className="scene-toolbar-badge">{modelingBadge}</span>}
            </Button>
            <Button
              type="button"
              className={`scene-toolbar-trigger${openPanels.includes('tool') ? ' is-active' : ''}`}
              aria-expanded={openPanels.includes('tool')}
              aria-controls="scene-popover-tool"
              title="Ferramenta (TCP) · Ctrl+Clique para abrir em paralelo"
              onClick={e => onTogglePanel('tool', Boolean(e.ctrlKey || e.metaKey || e.shiftKey))}
            >
              <Wrench size={14} aria-hidden="true" />
              <span>Ferramenta</span>
              <span className="scene-toolbar-badge">{TOOL_BADGE_LABELS[toolType] || 'TCP'}</span>
            </Button>
          </div>
        </div>

        <div className="scene-workspace-row" ref={workspaceRowRef}>
          {openPanels.map((panelId) => {
            const panelMeta = POPOVER_META[panelId] || POPOVER_META.model;
            const win = windowStates[panelId] || DEFAULT_WINDOW_STATE;
            const isPinned = pinnedPanels.includes(panelId);
            const isFocused = activePanel === panelId;
            const focusIdx = panelFocusOrder.indexOf(panelId);
            const zIndex = 10 + (focusIdx >= 0 ? focusIdx : openPanels.indexOf(panelId));
            const isDraggingThis = dragState?.panelId === panelId;
            const popoverStyle = win.layout === 'fullscreen'
              ? undefined
              : win.layout === 'dock-left'
                ? {
                    width: win.size?.width ? `${win.size.width}px` : undefined,
                    zIndex,
                  }
                : {
                    transform: `translate3d(${win.pos.x}px, ${win.pos.y}px, 0)`,
                    width: win.size?.width ? `${win.size.width}px` : undefined,
                    height: (!win.minimized && win.size?.height) ? `${win.size.height}px` : undefined,
                    maxWidth: win.pos.x > 0 ? `calc(100% - ${win.pos.x}px)` : undefined,
                    maxHeight: win.pos.y > 0 ? `calc(100% - ${win.pos.y}px)` : '100%',
                    zIndex,
                  };
            return (
              <section
                key={panelId}
                ref={el => {
                  if (el) popoverRefs.current[panelId] = el;
                  else delete popoverRefs.current[panelId];
                }}
                id={`scene-popover-${panelId}`}
                data-panel-id={panelId}
                className={`scene-popover scene-popover--${panelMeta.size} scene-popover--${win.layout}${win.minimized ? ' is-minimized' : ''}${isFocused ? ' is-focused' : ''}${isDraggingThis ? ` is-${dragState.type}` : ''}`}
                style={popoverStyle}
                role="dialog"
                aria-label={panelMeta.title}
                onPointerDown={() => onFocusPanel?.(panelId)}
              >
                <div
                  className="scene-popover-header"
                  onPointerDown={e => startDrag(panelId, e)}
                  onDoubleClick={(e) => {
                    if (e.target.closest('button')) return;
                    updateWindow(panelId, cur => ({ layout: cur.layout === 'fullscreen' ? 'floating' : 'fullscreen', minimized: false }));
                  }}
                >
                  <h2>{panelMeta.title}</h2>
                  <div className="scene-popover-header-actions">
                    {panelId === 'modeling' && (
                      <Button
                        icon
                        title="Restaurar modelagem padrão"
                        aria-label="Restaurar modelagem padrão"
                        disabled={isDefaultModelStyle(modelStyle)}
                        onClick={() => { setModelStyle({ ...DEFAULT_MODEL_STYLE }); setModelTarget('global'); }}
                      >
                        <RotateCcw size={15} />
                      </Button>
                    )}
                    {panelId === 'tool' && (
                      <Button
                        icon
                        title="Restaurar ferramenta padrão"
                        aria-label="Restaurar ferramenta padrão"
                        disabled={isDefaultTool}
                        onClick={() => { setToolType('gripper'); setToolOffsetRpy([...defaultToolOffset]); setToolGrip(50); setToolScale?.(1); }}
                      >
                        <RotateCcw size={15} />
                      </Button>
                    )}
                    <Button
                      icon
                      className={isPinned ? 'is-active' : ''}
                      title={isPinned ? 'Desafixar janela (fecha ao abrir outra aba)' : 'Manter janela aberta ao abrir outras abas'}
                      aria-label="Manter janela aberta"
                      aria-pressed={isPinned}
                      onClick={() => onTogglePinPanel?.(panelId)}
                    >
                      <Pin size={14} />
                    </Button>
                    <Button
                      icon
                      className={win.minimized ? 'is-active' : ''}
                      title={win.minimized ? 'Expandir painel' : 'Minimizar painel'}
                      aria-label="Minimizar painel"
                      aria-pressed={win.minimized}
                      onClick={() => updateWindow(panelId, cur => ({ minimized: !cur.minimized, ...(cur.minimized ? {} : { layout: cur.layout === 'fullscreen' ? 'floating' : cur.layout }) }))}
                    >
                      <Minus size={14} />
                    </Button>
                    <Button
                      icon
                      className={win.layout === 'dock-left' ? 'is-active' : ''}
                      title={win.layout === 'dock-left' ? 'Soltar painel da lateral' : 'Fixar painel na lateral'}
                      aria-label="Fixar painel na lateral"
                      aria-pressed={win.layout === 'dock-left'}
                      onClick={() => updateWindow(panelId, cur => ({ layout: cur.layout === 'dock-left' ? 'floating' : 'dock-left', minimized: false }))}
                    >
                      <PanelLeft size={14} />
                    </Button>
                    <Button
                      icon
                      className={win.layout === 'fullscreen' ? 'is-active' : ''}
                      title={win.layout === 'fullscreen' ? 'Restaurar janela flutuante' : 'Tela cheia do painel'}
                      aria-label="Tela cheia do painel"
                      aria-pressed={win.layout === 'fullscreen'}
                      onClick={() => updateWindow(panelId, cur => ({ layout: cur.layout === 'fullscreen' ? 'floating' : 'fullscreen', minimized: false }))}
                    >
                      {win.layout === 'fullscreen' ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
                    </Button>
                    <Button icon title="Fechar painel" aria-label="Fechar painel" onClick={() => onClosePanel(panelId)}>
                      <X size={15} />
                    </Button>
                  </div>
                </div>

                {!win.minimized && (
                  <div className="scene-popover-body">
                    {panelId === 'modeling' && (
                      <ModelingPanel
                        joints={joints}
                        modelStyle={modelStyle}
                        setModelStyle={setModelStyle}
                        modelTarget={modelTarget}
                        setModelTarget={setModelTarget}
                        numeric={numeric}
                      />
                    )}

                    {panelId === 'tool' && (
                      <ToolPanel
                        toolType={toolType}
                        setToolType={setToolType}
                        toolGrip={toolGrip}
                        setToolGrip={setToolGrip}
                        toolScale={toolScale}
                        setToolScale={setToolScale}
                        toolOffsetRpy={toolOffsetRpy}
                        setToolOffsetRpy={setToolOffsetRpy}
                        toolFrame={toolFrame}
                        numeric={numeric}
                      />
                    )}

                    {panelId !== 'modeling' && panelId !== 'tool' && renderPanelContent?.(panelId)}
                  </div>
                )}

                {!win.minimized && win.layout !== 'fullscreen' && (
                  <div
                    className="scene-popover-resize-e"
                    onPointerDown={e => startResize(panelId, 'e', e)}
                    aria-hidden="true"
                  />
                )}
                {!win.minimized && win.layout === 'floating' && (
                  <>
                    <div
                      className="scene-popover-resize-s"
                      onPointerDown={e => startResize(panelId, 's', e)}
                      aria-hidden="true"
                    />
                    <div
                      className="scene-popover-resize-se"
                      data-testid="popover-resize-se"
                      onPointerDown={e => startResize(panelId, 'se', e)}
                      aria-hidden="true"
                    />
                  </>
                )}
              </section>
            );
          })}

          <div className="scene-toast-slot">
            {message && <Notice role="status" variant={activeCollision ? 'error' : undefined}>{message}</Notice>}
          </div>
        </div>
      </div>

      <SceneViewportDock
        targetJoint={targetJoint}
        targetIdx={targetIdx}
        isModelingOpen={openPanels.includes('modeling')}
        onToggleModeling={onToggleModeling}
        onClearJointSelection={onClearJointSelection}
        showFrames={showFrames}
        setShowFrames={setShowFrames}
        environment={environment}
        applyEnvironment={applyEnvironment}
        hasTrajectory={hasTrajectory}
        showTrajectory={showTrajectory}
        setShowTrajectory={setShowTrajectory}
        hasWorkspace={hasWorkspace}
        showWorkspace={showWorkspace}
        setShowWorkspace={setShowWorkspace}
        showEllipsoid={showEllipsoid}
        setShowEllipsoid={setShowEllipsoid}
        showGhost={showGhost}
        setShowGhost={setShowGhost}
        viewPreset={viewPreset}
        onSelectViewPreset={onSelectViewPreset}
        customStlGeometry={customStlGeometry}
        stlFileName={stlFileName}
        handleStlUpload={handleStlUpload}
        removeStlMesh={removeStlMesh}
        showGizmo={showGizmo}
        setShowGizmo={setShowGizmo}
        gizmoMode={gizmoMode}
        onChangeGizmoMode={onChangeGizmoMode}
        gizmoLockOrientation={gizmoLockOrientation}
        setGizmoLockOrientation={setGizmoLockOrientation}
        gizmoLockPosition={gizmoLockPosition}
        setGizmoLockPosition={setGizmoLockPosition}
        onFrameView={onFrameView}
        onAddWaypoint={onAddWaypoint}
        isPlaying={isPlaying}
        canPlayTrajectory={canPlayTrajectory}
        onTogglePlayback={onTogglePlayback}
      />
    </>
  );
});

export default SceneToolbar;
