import { memo, useEffect, useRef, useState } from 'react';
import {
  Focus,
  FolderOpen,
  Maximize2,
  Minimize2,
  PanelLeft,
  RotateCcw,
  SlidersHorizontal,
  Trash2,
  Upload,
  Wrench,
  X,
} from 'lucide-react';
import { Button, Checkbox, FileUploadButton, Notice, SegmentedTabs, Select, Slider } from './ui';
import {
  DEFAULT_MODEL_STYLE,
  LINK_SHAPE_LIST,
  MODEL_STYLE_BOUNDS,
  TOOL_LIST,
  TOOL_TYPES,
  isDefaultModelStyle,
  normalizeModelStyle,
  resolveJointModelStyle,
} from './kinematics';
import { fmt } from './format';

const ANALYSIS_TABS = [
  { id: 'model', label: 'Modelo', title: 'Modelo e Atuadores', size: 'wide' },
  { id: 'forward', label: 'Direta', title: 'Cinemática Direta', size: 'wide' },
  { id: 'differential', label: 'Diferencial', title: 'Cinemática Diferencial e Estática', size: 'wide' },
  { id: 'inverse', label: 'Inversa', title: 'Cinemática Inversa (IK)', size: 'medium' },
  { id: 'trajectory', label: 'Trajetória', title: 'Planejamento de Trajetória', size: 'wide' },
  { id: 'workspace', label: 'Alcance 3D', title: 'Espaço de Trabalho 3D', size: 'compact' },
  { id: 'export', label: 'Código', title: 'Exportação de Código e URDF', size: 'wide' },
  { id: 'report', label: 'Relatório', title: 'Relatório Acadêmico', size: 'wide' },
];

const POPOVER_META = {
  ...Object.fromEntries(ANALYSIS_TABS.map(item => [item.id, item])),
  projects: { id: 'projects', title: 'Projetos, Presets e Arquivos', size: 'medium' },
  modeling: { id: 'modeling', title: 'Modelagem 3D (hastes e juntas)', size: 'compact' },
  tool: { id: 'tool', title: 'Ferramenta do efetuador (TCP)', size: 'compact' },
};

const MODEL_STYLE_SLIDERS = [
  { key: 'linkRadius', label: 'Raio das hastes (u)', hint: 'Espessura radial/transversal dos elos', step: 0.005, digits: 3 },
  { key: 'revRadius', label: 'Raio da junta rotativa R (u)', hint: 'Diâmetro visual dos motores R', step: 0.01, digits: 2 },
  { key: 'revLength', label: 'Altura da junta rotativa R (u)', hint: 'Comprimento axial dos cilindros R', step: 0.02, digits: 2 },
  { key: 'prismWidth', label: 'Largura da junta prismática P (u)', hint: 'Seção do bloco/guia P', step: 0.01, digits: 2 },
  { key: 'prismLength', label: 'Comprimento da junta prismática P (u)', hint: 'Extensão da manga deslizante P', step: 0.02, digits: 2 },
];

const TOOL_BADGE_LABELS = {
  gripper: 'Garra',
  vacuum: 'Ventosa',
  torch: 'Tocha',
};

const SceneToolbar = memo(function SceneToolbar({
  joints,
  tab,
  activePanel,
  onSelectTab,
  onTogglePanel,
  onClosePanel,
  message,
  activeCollision,
  showFrames,
  setShowFrames,
  environment,
  applyEnvironment,
  showGizmo,
  setShowGizmo,
  hasTrajectory,
  showTrajectory,
  setShowTrajectory,
  hasWorkspace,
  showWorkspace,
  setShowWorkspace,
  customStlGeometry,
  stlFileName,
  handleStlUpload,
  removeStlMesh,
  onFrameView,
  toolType,
  setToolType,
  toolOffsetRpy,
  setToolOffsetRpy,
  defaultToolOffset,
  toolGrip,
  setToolGrip,
  toolFrame,
  modelStyle,
  setModelStyle,
  modelTarget,
  setModelTarget,
  numeric,
  children,
}) {
  const [panelLayout, setPanelLayout] = useState('floating'); // 'floating' | 'dock-left' | 'fullscreen'
  const [panelPos, setPanelPos] = useState({ x: 0, y: 0 });
  const [customSize, setCustomSize] = useState(null);
  const [dragState, setDragState] = useState(null); // 'move' | 'resize' | null
  const workspaceRowRef = useRef(null);
  const popoverRef = useRef(null);

  useEffect(() => {
    if (!activePanel) return undefined;
    const handleKeyDown = (e) => {
      if (e.key === 'Escape' && !e.defaultPrevented && !document.querySelector('.ui-select-dropdown')) {
        onClosePanel();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [activePanel, onClosePanel]);

  useEffect(() => {
    if (!activePanel || panelLayout !== 'floating') return undefined;
    const clampToWorkspace = () => {
      const ws = workspaceRowRef.current;
      const pop = popoverRef.current;
      if (!ws || !pop) return;
      const wsRect = ws.getBoundingClientRect();
      const popRect = pop.getBoundingClientRect();
      setPanelPos(prev => {
        const maxX = Math.max(0, wsRect.width - popRect.width);
        const maxY = Math.max(0, wsRect.height - Math.min(popRect.height, 80));
        const nx = Math.max(0, Math.min(prev.x, maxX));
        const ny = Math.max(0, Math.min(prev.y, maxY));
        return nx === prev.x && ny === prev.y ? prev : { x: nx, y: ny };
      });
    };
    window.addEventListener('resize', clampToWorkspace);
    return () => window.removeEventListener('resize', clampToWorkspace);
  }, [activePanel, panelLayout]);

  const startDrag = (e) => {
    if (panelLayout !== 'floating' || e.button !== 0) return;
    if (e.target.closest('button, input, select, a, label')) return;
    const ws = workspaceRowRef.current;
    const pop = popoverRef.current;
    if (!ws || !pop) return;
    e.preventDefault();
    const wsRect = ws.getBoundingClientRect();
    const popRect = pop.getBoundingClientRect();
    const startX = e.clientX;
    const startY = e.clientY;
    const origX = panelPos.x;
    const origY = panelPos.y;
    const maxX = Math.max(0, wsRect.width - popRect.width);
    const maxY = Math.max(0, wsRect.height - Math.min(popRect.height, 80));
    setDragState('move');

    const onMove = (moveEvent) => {
      const nx = Math.max(0, Math.min(maxX, Math.round(origX + (moveEvent.clientX - startX))));
      const ny = Math.max(0, Math.min(maxY, Math.round(origY + (moveEvent.clientY - startY))));
      setPanelPos({ x: nx, y: ny });
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

  const startResize = (dir, e) => {
    if (e.button !== 0 || panelLayout === 'fullscreen') return;
    const ws = workspaceRowRef.current;
    const pop = popoverRef.current;
    if (!ws || !pop) return;
    e.preventDefault();
    e.stopPropagation();
    const wsRect = ws.getBoundingClientRect();
    const popRect = pop.getBoundingClientRect();
    const startX = e.clientX;
    const startY = e.clientY;
    const startW = popRect.width;
    const startH = popRect.height;
    const offsetX = panelLayout === 'floating' ? panelPos.x : 0;
    const offsetY = panelLayout === 'floating' ? panelPos.y : 0;
    const minW = Math.min(320, Math.max(220, wsRect.width - offsetX));
    const maxW = Math.max(minW, wsRect.width - offsetX);
    const minH = Math.min(220, Math.max(160, wsRect.height - offsetY));
    const maxH = Math.max(minH, wsRect.height - offsetY);
    setDragState('resize');

    const onMove = (moveEvent) => {
      const dx = moveEvent.clientX - startX;
      const dy = moveEvent.clientY - startY;
      setCustomSize(prev => {
        const next = { ...(prev || {}) };
        if (dir === 'e' || dir === 'se') {
          next.width = Math.round(Math.max(minW, Math.min(maxW, startW + dx)));
        }
        if (dir === 's' || dir === 'se') {
          next.height = Math.round(Math.max(minH, Math.min(maxH, startH + dy)));
        }
        return next;
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

  const panelMeta = activePanel ? (POPOVER_META[activePanel] || POPOVER_META.model) : null;
  const activeTarget = modelTarget !== 'global' && joints.some(j => String(j.id) === modelTarget)
    ? modelTarget
    : 'global';
  const targetIdx = joints.findIndex(j => String(j.id) === activeTarget);
  const targetJoint = targetIdx >= 0 ? joints[targetIdx] : null;
  const activeStyle = targetJoint ? resolveJointModelStyle(modelStyle, targetJoint.id) : modelStyle;
  const visibleSliders = targetJoint
    ? MODEL_STYLE_SLIDERS.filter(s => s.key === 'linkRadius' || (targetJoint.type === 'R' ? s.key.startsWith('rev') : s.key.startsWith('prism')))
    : MODEL_STYLE_SLIDERS;

  const overrideCount = modelStyle.perJoint ? Object.keys(modelStyle.perJoint).length : 0;
  const hasPerJoint = overrideCount > 0;
  const modelingBadge = targetJoint ? `J${targetIdx + 1}` : (hasPerJoint ? `${overrideCount}×` : null);
  const isDefaultTool = toolType === 'gripper'
    && toolOffsetRpy.every((v, idx) => Math.abs(v - defaultToolOffset[idx]) < 1e-6)
    && toolGrip === 50;

  const applyStyleEdit = (patch) => {
    if (!targetJoint) {
      setModelStyle(prev => normalizeModelStyle({ ...prev, ...patch }));
      return;
    }
    const jKey = String(targetJoint.id);
    setModelStyle(prev => normalizeModelStyle({
      ...prev,
      perJoint: {
        ...(prev.perJoint || {}),
        [jKey]: { ...(prev.perJoint?.[jKey] || {}), ...patch },
      },
    }));
  };

  const clearJointOverride = () => {
    if (!targetJoint) return;
    const nextPer = { ...(modelStyle.perJoint || {}) };
    delete nextPer[String(targetJoint.id)];
    setModelStyle(prev => normalizeModelStyle({ ...prev, perJoint: nextPer }));
  };

  const chainSummary = `${joints.map(j => j.type).join('')} · ${joints.length} juntas`;
  const popoverStyle = panelLayout === 'fullscreen'
    ? undefined
    : panelLayout === 'dock-left'
      ? {
          width: customSize?.width ? `${customSize.width}px` : undefined,
        }
      : {
          transform: `translate3d(${panelPos.x}px, ${panelPos.y}px, 0)`,
          width: customSize?.width ? `${customSize.width}px` : undefined,
          height: customSize?.height ? `${customSize.height}px` : undefined,
          maxWidth: panelPos.x > 0 ? `calc(100% - ${panelPos.x}px)` : undefined,
          maxHeight: panelPos.y > 0 ? `calc(100% - ${panelPos.y}px)` : '100%',
        };

  return (
    <div className="scene-toolbar-shell">
      <div className="scene-toolbar" role="toolbar" aria-label="Controles da cena 3D">
        <div className="scene-toolbar-group scene-toolbar-left">
          <header className="scene-brand" title={`Manipulador ${chainSummary}`}>
            <h1 className="visually-hidden">Cinemática DH</h1>
            <span className="scene-brand-meta">
              <span className="visually-hidden">Manipulador </span>
              {chainSummary}
            </span>
          </header>
          <Button
            type="button"
            className={`scene-toolbar-trigger${activePanel === 'projects' ? ' is-active' : ''}`}
            aria-expanded={activePanel === 'projects'}
            aria-controls="scene-popover-panel"
            onClick={() => onTogglePanel('projects')}
          >
            <FolderOpen size={14} />
            <span>Projetos</span>
          </Button>
        </div>

        <nav className="scene-toolbar-nav" aria-label="Análises">
          {ANALYSIS_TABS.map(item => (
            <button
              key={item.id}
              type="button"
              className={`scene-toolbar-tab${tab === item.id ? ' is-context' : ''}${activePanel === item.id ? ' is-open' : ''}`}
              aria-current={tab === item.id ? 'page' : undefined}
              aria-expanded={activePanel === item.id}
              aria-controls="scene-popover-panel"
              onClick={() => onSelectTab(item.id)}
            >
              {item.label}
            </button>
          ))}
        </nav>

        <div className="scene-toolbar-group scene-toolbar-actions">
          <Button
            type="button"
            className={`scene-toolbar-trigger${activePanel === 'modeling' ? ' is-active' : ''}`}
            aria-expanded={activePanel === 'modeling'}
            aria-controls="scene-popover-panel"
            onClick={() => onTogglePanel('modeling')}
          >
            <SlidersHorizontal size={14} />
            <span>Modelagem 3D</span>
            {modelingBadge && <span className="scene-toolbar-badge">{modelingBadge}</span>}
          </Button>
          <Button
            type="button"
            className={`scene-toolbar-trigger${activePanel === 'tool' ? ' is-active' : ''}`}
            aria-expanded={activePanel === 'tool'}
            aria-controls="scene-popover-panel"
            onClick={() => onTogglePanel('tool')}
          >
            <Wrench size={14} />
            <span>Ferramenta</span>
            <span className="scene-toolbar-badge">{TOOL_BADGE_LABELS[toolType] || 'TCP'}</span>
          </Button>
        </div>

        <div className="scene-toolbar-group scene-toolbar-toggles">
          <Checkbox checked={showFrames} onChange={setShowFrames}>Referenciais</Checkbox>
          <Checkbox
            checked={environment.obstacle.enabled}
            onChange={enabled => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, enabled } })}
          >
            Obstáculo 3D
          </Checkbox>
          {tab === 'inverse' && <Checkbox checked={showGizmo} onChange={setShowGizmo}>Gizmo 3D</Checkbox>}
          {hasTrajectory && <Checkbox checked={showTrajectory} onChange={setShowTrajectory}>Trilha 3D</Checkbox>}
          {hasWorkspace && <Checkbox checked={showWorkspace} onChange={setShowWorkspace}>Nuvem 3D</Checkbox>}
          {customStlGeometry ? (
            <Button icon variant="danger" title={`Remover malha STL (${stlFileName})`} aria-label="Remover malha STL" onClick={removeStlMesh}>
              <Trash2 size={16} />
            </Button>
          ) : (
            <FileUploadButton icon accept=".stl" onChange={handleStlUpload} title="Carregar malha STL (Garra / Ferramenta)" ariaLabel="Carregar malha STL" inputAriaLabel="Upload de malha STL">
              <Upload size={17} />
            </FileUploadButton>
          )}
          <Button icon title="Enquadrar manipulador" aria-label="Enquadrar manipulador" onClick={onFrameView}>
            <Focus size={18} />
          </Button>
        </div>
      </div>

      <div className="scene-workspace-row" ref={workspaceRowRef}>
        {panelMeta && (
          <section
            ref={popoverRef}
            id="scene-popover-panel"
            className={`scene-popover scene-popover--${panelMeta.size} scene-popover--${panelLayout}${dragState ? ` is-${dragState}` : ''}`}
            style={popoverStyle}
            role="dialog"
            aria-label={panelMeta.title}
          >
            <div
              className="scene-popover-header"
              onPointerDown={startDrag}
              onDoubleClick={(e) => {
                if (e.target.closest('button')) return;
                setPanelLayout(prev => (prev === 'fullscreen' ? 'floating' : 'fullscreen'));
              }}
            >
              <h2>{panelMeta.title}</h2>
              <div className="scene-popover-header-actions">
                {activePanel === 'modeling' && (
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
                {activePanel === 'tool' && (
                  <Button
                    icon
                    title="Restaurar ferramenta padrão"
                    aria-label="Restaurar ferramenta padrão"
                    disabled={isDefaultTool}
                    onClick={() => { setToolType('gripper'); setToolOffsetRpy([...defaultToolOffset]); setToolGrip(50); }}
                  >
                    <RotateCcw size={15} />
                  </Button>
                )}
                <Button
                  icon
                  className={panelLayout === 'dock-left' ? 'is-active' : ''}
                  title={panelLayout === 'dock-left' ? 'Soltar painel da lateral' : 'Fixar painel na lateral'}
                  aria-label="Fixar painel na lateral"
                  aria-pressed={panelLayout === 'dock-left'}
                  onClick={() => setPanelLayout(prev => (prev === 'dock-left' ? 'floating' : 'dock-left'))}
                >
                  <PanelLeft size={14} />
                </Button>
                <Button
                  icon
                  className={panelLayout === 'fullscreen' ? 'is-active' : ''}
                  title={panelLayout === 'fullscreen' ? 'Restaurar janela flutuante' : 'Tela cheia do painel'}
                  aria-label="Tela cheia do painel"
                  aria-pressed={panelLayout === 'fullscreen'}
                  onClick={() => setPanelLayout(prev => (prev === 'fullscreen' ? 'floating' : 'fullscreen'))}
                >
                  {panelLayout === 'fullscreen' ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
                </Button>
                <Button icon title="Fechar painel" aria-label="Fechar painel" onClick={onClosePanel}>
                  <X size={15} />
                </Button>
              </div>
            </div>

            <div className="scene-popover-body">
              {activePanel === 'modeling' && (
                <>
                  <div className="preset-row">
                    <label className="tool-inline-label">Alvo da personalização
                      <Select
                        ariaLabel="Alvo da personalização 3D"
                        value={activeTarget}
                        onChange={setModelTarget}
                        options={[
                          { value: 'global', label: 'Padrão global (todos os elos e eixos)' },
                          ...joints.map((j, idx) => ({
                            value: String(j.id),
                            label: `Eixo / Ligação ${idx + 1} (${j.type})${modelStyle.perJoint?.[String(j.id)] ? ' · Personalizado' : ''}`,
                          })),
                        ]}
                      />
                    </label>
                  </div>
                  <span className="tool-inline-label">{targetJoint ? `Formato da haste · Ligação ${targetIdx + 1}` : 'Formato das hastes'}</span>
                  <SegmentedTabs
                    ariaLabel="Formato das hastes"
                    activeId={activeStyle.linkShape}
                    onChange={linkShape => applyStyleEdit({ linkShape })}
                    items={LINK_SHAPE_LIST.map(s => ({ id: s.id, label: s.label }))}
                  />
                  <div className="ik-joint-limits-list">
                    {visibleSliders.map(({ key, label, hint, step, digits }) => {
                      const [min, max] = MODEL_STYLE_BOUNDS[key];
                      const value = activeStyle[key];
                      return (
                        <div className="ik-slider-card" key={key}>
                          <div className="ik-slider-header">
                            <div className="ik-slider-meta">
                              <strong>{label}</strong>
                              <span className="muted">{hint}</span>
                            </div>
                            {numeric(label, value, next => applyStyleEdit({ [key]: next }), { min, max, compact: true })}
                          </div>
                          <Slider
                            ariaLabel={`Controle ${label}`}
                            ariaValueText={`${fmt(value, digits)} u`}
                            min={min}
                            max={max}
                            step={step}
                            value={value}
                            onChange={next => applyStyleEdit({ [key]: next })}
                          />
                        </div>
                      );
                    })}
                  </div>
                  {targetJoint && activeStyle.overridden && (
                    <div className="action-row">
                      <Button onClick={clearJointOverride}>Usar padrão global neste eixo</Button>
                    </div>
                  )}
                  {!targetJoint && hasPerJoint && (
                    <div className="action-row">
                      <Button onClick={() => setModelStyle(prev => normalizeModelStyle({ ...prev, perJoint: undefined }))}>Aplicar padrão global a todos</Button>
                    </div>
                  )}
                </>
              )}

              {activePanel === 'tool' && (
                <>
                  <SegmentedTabs
                    ariaLabel="Tipo de ferramenta"
                    activeId={toolType}
                    onChange={setToolType}
                    items={TOOL_LIST.map(t => ({ id: t.id, label: t.shortLabel }))}
                  />
                  <p className="muted scene-popover-desc">{TOOL_TYPES[toolType].description}</p>
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
                </>
              )}

              {activePanel !== 'modeling' && activePanel !== 'tool' && children}
            </div>

            {panelLayout !== 'fullscreen' && (
              <div
                className="scene-popover-resize-e"
                onPointerDown={e => startResize('e', e)}
                aria-hidden="true"
              />
            )}
            {panelLayout === 'floating' && (
              <>
                <div
                  className="scene-popover-resize-s"
                  onPointerDown={e => startResize('s', e)}
                  aria-hidden="true"
                />
                <div
                  className="scene-popover-resize-se"
                  data-testid="popover-resize-se"
                  onPointerDown={e => startResize('se', e)}
                  aria-hidden="true"
                />
              </>
            )}
          </section>
        )}

        <div className="scene-toast-slot">
          {message && <Notice role="status" variant={activeCollision ? 'error' : undefined}>{message}</Notice>}
        </div>
      </div>
    </div>
  );
});

export default SceneToolbar;
