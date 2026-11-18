import { memo, useEffect, useRef, useState } from 'react';
import {
  ChevronUp,
  Focus,
  Layers,
  Lock,
  Pause,
  Play,
  Plus,
  Trash2,
  Unlock,
  Upload,
  X,
} from 'lucide-react';
import { Button, Checkbox, FileUploadButton } from './ui';
import { fmt } from './format';

const CAMERA_PRESETS = [
  { id: 'iso', label: '3D', aria: 'Vista 3D isométrica' },
  { id: 'top', label: 'XY', aria: 'Vista superior XY' },
  { id: 'front', label: 'XZ', aria: 'Vista frontal XZ' },
  { id: 'side', label: 'YZ', aria: 'Vista lateral YZ' },
];

const SceneViewportDock = memo(function SceneViewportDock({
  targetJoint,
  targetIdx,
  isModelingOpen,
  onToggleModeling,
  onClearJointSelection,
  showFrames,
  setShowFrames,
  environment,
  applyEnvironment,
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
  viewPreset,
  onSelectViewPreset,
  customStlGeometry,
  stlFileName,
  handleStlUpload,
  removeStlMesh,
  showGizmo,
  setShowGizmo,
  gizmoMode,
  onChangeGizmoMode,
  gizmoLockOrientation,
  setGizmoLockOrientation,
  gizmoLockPosition,
  setGizmoLockPosition,
  onFrameView,
  onAddWaypoint,
  isPlaying,
  canPlayTrajectory,
  onTogglePlayback,
}) {
  const [layersOpen, setLayersOpen] = useState(false);
  const dockRef = useRef(null);

  useEffect(() => {
    if (!layersOpen) return undefined;
    const handleKeyDown = (e) => {
      if (e.key !== 'Escape' || e.defaultPrevented || document.querySelector('.ui-select-dropdown')) return;
      e.preventDefault();
      setLayersOpen(false);
    };
    const handlePointerDown = (e) => {
      if (dockRef.current?.contains(e.target)) return;
      setLayersOpen(false);
    };
    window.addEventListener('keydown', handleKeyDown, true);
    document.addEventListener('pointerdown', handlePointerDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown, true);
      document.removeEventListener('pointerdown', handlePointerDown);
    };
  }, [layersOpen]);

  const activeLayerCount = [
    showFrames,
    environment.obstacle.enabled,
    hasTrajectory && showTrajectory,
    hasWorkspace && showWorkspace,
    showEllipsoid,
    showGhost,
    Boolean(customStlGeometry),
  ].filter(Boolean).length;

  return (
    <div ref={dockRef} className="scene-viewport-dock" role="region" aria-label="Atalhos da cena 3D">
      {layersOpen && (
        <div id="scene-layers-menu" className="scene-layers-dropup" role="group" aria-label="Camadas e malha 3D">
          <div className="scene-layers-dropup-title">Camadas da cena</div>
          <Checkbox checked={showFrames} onChange={setShowFrames}>Referenciais</Checkbox>
          <Checkbox
            checked={environment.obstacle.enabled}
            onChange={enabled => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, enabled } })}
          >
            Obstáculo 3D
          </Checkbox>
          {hasTrajectory && <Checkbox checked={showTrajectory} onChange={setShowTrajectory}>Trilha 3D</Checkbox>}
          {hasWorkspace && <Checkbox checked={showWorkspace} onChange={setShowWorkspace}>Nuvem 3D</Checkbox>}
          <Checkbox checked={showEllipsoid} onChange={setShowEllipsoid}>Elipsoide 3D</Checkbox>
          <Checkbox checked={showGhost} onChange={setShowGhost}>Pose Fantasma</Checkbox>
          <div className="scene-layers-divider" role="separator" />
          <div className="scene-layers-dropup-title">Vistas de câmera CAD</div>
          <div className="scene-camera-presets" role="group" aria-label="Vistas de câmera CAD">
            {CAMERA_PRESETS.map(v => (
              <Button
                key={v.id}
                type="button"
                className={`scene-toolbar-trigger scene-camera-preset-btn${viewPreset === v.id ? ' is-active' : ''}`}
                aria-label={v.aria}
                aria-pressed={viewPreset === v.id}
                onClick={() => onSelectViewPreset(v.id)}
              >
                <span>{v.label}</span>
              </Button>
            ))}
          </div>
          <div className="scene-layers-divider" role="separator" />
          {customStlGeometry ? (
            <Button variant="danger" className="scene-layers-stl-btn" title={`Remover malha STL (${stlFileName})`} aria-label="Remover malha STL" onClick={removeStlMesh}>
              <Trash2 size={14} aria-hidden="true" /> Remover STL ({stlFileName})
            </Button>
          ) : (
            <FileUploadButton accept=".stl" className="scene-layers-stl-btn" onChange={handleStlUpload} title="Carregar malha STL (Garra / Ferramenta)" ariaLabel="Carregar malha STL" inputAriaLabel="Upload de malha STL">
              <Upload size={14} aria-hidden="true" /> Carregar malha STL
            </FileUploadButton>
          )}
        </div>
      )}

      {targetJoint && !isModelingOpen && (
        <div className="scene-selected-joint-pill" data-testid="selected-joint-pill">
          <span>
            Eixo {targetIdx + 1} ({targetJoint.type}) · {fmt(targetJoint.val, targetJoint.type === 'R' ? 1 : 2)}
            {targetJoint.type === 'R' ? '°' : ' u'}
          </span>
          <Button
            type="button"
            className="scene-toolbar-trigger"
            onClick={onToggleModeling}
          >
            <span>Editar</span>
          </Button>
          <Button
            icon
            type="button"
            title="Limpar seleção de junta"
            aria-label="Limpar seleção de junta"
            onClick={onClearJointSelection}
          >
            <X size={13} />
          </Button>
        </div>
      )}

      <div className="scene-viewport-bar" role="toolbar" aria-label="Controles de viewport 3D">
        <Button
          type="button"
          className={`scene-toolbar-trigger scene-play-trajectory-btn${isPlaying ? ' is-active' : ''}`}
          disabled={!canPlayTrajectory}
          title={isPlaying ? 'Pausar trajetória (Atalho: Espaço)' : 'Rodar trajetória (Atalho: Espaço)'}
          aria-label={isPlaying ? 'Pausar animação 3D' : 'Executar animação 3D'}
          onClick={onTogglePlayback}
        >
          {isPlaying ? <Pause size={13} aria-hidden="true" /> : <Play size={13} aria-hidden="true" />}
          <span>{isPlaying ? 'Pausar' : 'Rodar'}</span>
        </Button>
        <Button
          type="button"
          className="scene-toolbar-trigger scene-add-waypoint-btn"
          title="Gravar ponto atual na sequência (Atalho: W)"
          aria-label="Gravar ponto atual"
          onClick={onAddWaypoint}
        >
          <Plus size={13} aria-hidden="true" />
          <span>Ponto</span>
        </Button>
        <Checkbox checked={showGizmo} onChange={setShowGizmo}>Gizmo 3D</Checkbox>
        {showGizmo && (
          <>
            <Button
              type="button"
              className="scene-toolbar-trigger scene-gizmo-mode-btn"
              title={gizmoMode === 'translate' ? 'Modo do Gizmo: Mover (XYZ) — clique para Girar (RPY)' : 'Modo do Gizmo: Girar (RPY) — clique para Mover (XYZ)'}
              aria-label="Alternar modo do Gizmo 3D"
              onClick={() => onChangeGizmoMode(gizmoMode === 'translate' ? 'rotate' : 'translate')}
            >
              <span>{gizmoMode === 'translate' ? 'XYZ' : 'RPY'}</span>
            </Button>
            {gizmoMode === 'translate' ? (
              <Button
                type="button"
                className={`scene-toolbar-trigger scene-lock-btn${gizmoLockOrientation ? ' is-active' : ''}`}
                title={gizmoLockOrientation ? 'Orientação (RPY) travada ao mover XYZ — clique para deixar livre' : 'Orientação livre ao mover XYZ — clique para travar RPY'}
                aria-label="Travar orientação RPY ao mover XYZ"
                aria-pressed={gizmoLockOrientation}
                onClick={() => setGizmoLockOrientation(prev => !prev)}
              >
                {gizmoLockOrientation ? <Lock size={12} aria-hidden="true" /> : <Unlock size={12} aria-hidden="true" />}
                <span>RPY</span>
              </Button>
            ) : (
              <Button
                type="button"
                className={`scene-toolbar-trigger scene-lock-btn${gizmoLockPosition ? ' is-active' : ''}`}
                title={gizmoLockPosition ? 'Posição (XYZ) travada ao girar RPY — clique para deixar livre' : 'Posição livre ao girar RPY — clique para travar XYZ'}
                aria-label="Travar posição XYZ ao girar RPY"
                aria-pressed={gizmoLockPosition}
                onClick={() => setGizmoLockPosition(prev => !prev)}
              >
                {gizmoLockPosition ? <Lock size={12} aria-hidden="true" /> : <Unlock size={12} aria-hidden="true" />}
                <span>XYZ</span>
              </Button>
            )}
          </>
        )}
        <Button
          type="button"
          className={`scene-toolbar-trigger scene-layers-trigger${layersOpen ? ' is-active' : ''}`}
          aria-label="Camadas da cena"
          aria-expanded={layersOpen}
          aria-controls="scene-layers-menu"
          title="Camadas de visualização, vistas CAD e malha STL"
          onClick={() => setLayersOpen(prev => !prev)}
        >
          <Layers size={14} aria-hidden="true" />
          <span>Camadas</span>
          <span className="scene-toolbar-badge">{activeLayerCount}</span>
          <ChevronUp size={13} className={`scene-layers-chevron${layersOpen ? ' is-open' : ''}`} aria-hidden="true" />
        </Button>
        <Button icon title="Enquadrar manipulador" aria-label="Enquadrar manipulador" onClick={onFrameView}>
          <Focus size={18} />
        </Button>
      </div>
    </div>
  );
});

export default SceneViewportDock;
