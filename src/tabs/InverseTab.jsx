import { memo } from 'react';
import { Check, Crosshair, Focus, Pause, Play, Plus, Trash2 } from 'lucide-react';
import { Button, Checkbox, MatrixView, Notice, SegmentedTabs, Slider } from '../ui';
import { TOOL_LIST, TOOL_TYPES } from '../kinematics';
import { fmt, vectorText } from '../format';
import EnvironmentSection from './EnvironmentSection';
import TaskSettingsFields from './TaskSettingsFields';

const PITCH_SINGULARITY_MARKERS = [
  { value: -90, start: -93, end: -87, kind: 'interior', label: 'Singularidade RPY (Gimbal Lock): -90°' },
  { value: 90, start: 87, end: 93, kind: 'interior', label: 'Singularidade RPY (Gimbal Lock): +90°' },
];

const InverseTab = memo(function InverseTab({
  joints,
  fk,
  currentEnd,
  mode,
  setMode,
  length,
  setLength,
  damping,
  setDamping,
  onChangeSetting,
  toolType,
  setToolType,
  toolFrame,
  showGizmo,
  setShowGizmo,
  gizmoMode,
  setGizmoMode,
  gizmoLockOrientation,
  setGizmoLockOrientation,
  gizmoLockPosition,
  setGizmoLockPosition,
  cartesianBounds,
  target,
  setTarget,
  targetDistance,
  toolAngularErrorDeg,
  autoSolveIk,
  setAutoSolveIk,
  ikWaypointDuration,
  setIkWaypointDuration,
  solution,
  setSolution,
  message,
  solveInverse,
  changeTarget,
  slideTarget,
  onSliderDragStart,
  onSliderDragEnd,
  applyOrientationPreset,
  updateJoint,
  addWaypointFromPose,
  waypoints,
  trajectoryData,
  isPlaying,
  setIsPlaying,
  playTime,
  setPlayTime,
  moveToWaypoint,
  removeWaypoint,
  environment,
  applyEnvironment,
  placeObstacleInFront,
  showObstacleGizmo,
  setShowObstacleGizmo,
  numeric,
}) {
  return (
    <>
      <section>
        <h2>Alvo cartesiano e orientação</h2>
        <TaskSettingsFields
          mode={mode}
          setMode={setMode}
          length={length}
          setLength={setLength}
          damping={damping}
          setDamping={setDamping}
          onChangeSetting={onChangeSetting}
          numeric={numeric}
        />
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
            <>
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
              <Checkbox checked={gizmoLockOrientation} onChange={setGizmoLockOrientation}>
                Travar orientação (RPY) ao mover posição
              </Checkbox>
              <Checkbox checked={gizmoLockPosition} onChange={setGizmoLockPosition}>
                Travar posição (XYZ) ao girar orientação
              </Checkbox>
            </>
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
            const axisSingularities = !isFixed
              ? (cartesianBounds.singularities?.[i] || [])
                  .filter(m => {
                    const s = Number.isFinite(m.start) ? m.start : m.value;
                    const e = Number.isFinite(m.end) ? m.end : m.value;
                    return e >= lo - 0.05 && s <= hi + 0.05;
                  })
                  .map(m => {
                    const s = Number.isFinite(m.start) ? Math.max(sliderMin, Math.min(sliderMax, m.start)) : Math.max(sliderMin, Math.min(sliderMax, m.value));
                    const e = Number.isFinite(m.end) ? Math.max(sliderMin, Math.min(sliderMax, m.end)) : Math.max(sliderMin, Math.min(sliderMax, m.value));
                    return {
                      ...m,
                      start: Math.min(s, e),
                      end: Math.max(s, e),
                      value: Math.max(sliderMin, Math.min(sliderMax, m.value)),
                    };
                  })
              : [];
            return (
              <div className="ik-slider-card" key={axis}>
                <div className="ik-slider-header">
                  <div className="ik-slider-meta">
                    <strong>Alvo {axis} (u)</strong>
                    <span className="muted">
                      Alcance: [{fmt(lo, 2)}, {fmt(hi, 2)}] u
                      {axisSingularities.length > 0 && (
                        <>
                          {' · '}
                          <span
                            className="ik-singularity-badge"
                            title={axisSingularities.map(m => m.label).join(' · ')}
                          >
                            <span className="ik-singularity-legend-dot" aria-hidden="true" />
                            Sing.: {axisSingularities.map(m => (
                              Math.abs(m.end - m.start) >= 0.05
                                ? `[${fmt(m.start, 1)}, ${fmt(m.end, 1)}]`
                                : fmt(m.value, 2)
                            )).join(', ')}
                          </span>
                        </>
                      )}
                    </span>
                  </div>
                  {numeric(`Alvo ${axis} (u)`, target.position[i], v => changeTarget('position', i, v), { compact: true })}
                </div>
                <Slider
                  ariaLabel={`Deslizar Alvo ${axis} (u)`}
                  ariaValueText={`${fmt(sliderVal, 2)} u`}
                  min={sliderMin}
                  max={sliderMax}
                  disabled={isFixed}
                  markers={axisSingularities}
                  value={sliderVal}
                  onPointerDown={onSliderDragStart}
                  onPointerUp={onSliderDragEnd}
                  onPointerCancel={onSliderDragEnd}
                  onBlur={onSliderDragEnd}
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
            const rpyMarkers = i === 1 ? PITCH_SINGULARITY_MARKERS : undefined;
            return (
              <div className="ik-slider-card" key={axis}>
                <div className="ik-slider-header">
                  <div className="ik-slider-meta">
                    <strong>{axis} alvo (°)</strong>
                    <span className="muted">
                      Alcance: [-180.00, 180.00] °
                      {rpyMarkers && (
                        <>
                          {' · '}
                          <span className="ik-singularity-badge" title="Singularidade RPY (Gimbal Lock) em ±90°">
                            <span className="ik-singularity-legend-dot" aria-hidden="true" />
                            Sing. (Gimbal Lock): ±90°
                          </span>
                        </>
                      )}
                    </span>
                  </div>
                  {numeric(`${axis} alvo (°)`, target.rpy[i], v => changeTarget('rpy', i, v), { compact: true })}
                </div>
                <Slider
                  ariaLabel={`Deslizar ${axis} alvo (°)`}
                  ariaValueText={`${fmt(sliderVal, 1)} °`}
                  min={-180}
                  max={180}
                  markers={rpyMarkers}
                  value={sliderVal}
                  onPointerDown={onSliderDragStart}
                  onPointerUp={onSliderDragEnd}
                  onPointerCancel={onSliderDragEnd}
                  onBlur={onSliderDragEnd}
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
                  onPointerDown={onSliderDragStart}
                  onPointerUp={onSliderDragEnd}
                  onPointerCancel={onSliderDragEnd}
                  onBlur={onSliderDragEnd}
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

      <EnvironmentSection
        environment={environment}
        applyEnvironment={applyEnvironment}
        placeObstacleInFront={placeObstacleInFront}
        showObstacleGizmo={showObstacleGizmo}
        setShowObstacleGizmo={setShowObstacleGizmo}
        numeric={numeric}
      />

      {solution && (
        <section>
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
        </section>
      )}

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
    </>
  );
});

export default InverseTab;
