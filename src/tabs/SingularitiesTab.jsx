import { memo } from 'react';
import { AlertTriangle, Crosshair, Eye, Orbit } from 'lucide-react';
import { Button, Checkbox, Notice } from '../ui';
import { fmt } from '../format';
import TaskSettingsFields from './TaskSettingsFields';

function AxisManipulabilityChart({ axisData }) {
  const { min, span, currentCoord, intervals, profile, label } = axisData;
  const width = 320;
  const height = 64;
  const padX = 6;
  const padY = 6;
  const plotW = width - padX * 2;
  const plotH = height - padY * 2;

  const toX = (coord) => {
    if (span <= 1e-6) return padX + plotW / 2;
    const t = Math.max(0, Math.min(1, (coord - min) / span));
    return padX + t * plotW;
  };

  // Split curve into contiguous non-empty polyline segments across dead-zones
  const segments = [];
  let currentSeg = [];
  for (const bin of profile) {
    if (bin.wNorm === null || bin.deadZone) {
      if (currentSeg.length > 0) {
        segments.push(currentSeg);
        currentSeg = [];
      }
      continue;
    }
    const x = toX(bin.coord);
    const y = padY + (1 - Math.max(0, Math.min(1, bin.wNorm))) * plotH;
    currentSeg.push(`${x.toFixed(1)},${y.toFixed(1)}`);
  }
  if (currentSeg.length > 0) segments.push(currentSeg);

  const curX = toX(currentCoord);

  return (
    <svg
      className="singularity-axis-svg"
      viewBox={`0 0 ${width} ${height}`}
      role="img"
      aria-label={`Perfil de manipulabilidade ao longo do eixo ${label}`}
    >
      <rect x={padX} y={padY} width={plotW} height={plotH} rx="3" className="singularity-svg-bg" />
      <line x1={padX} y1={padY + plotH * 0.78} x2={padX + plotW} y2={padY + plotH * 0.78} className="singularity-threshold-line" />
      {intervals.map((iv, idx) => {
        const x0 = toX(iv.start);
        const x1 = toX(iv.end);
        const w = Math.max(4, x1 - x0);
        const drawX = Math.min(padX + plotW - w, Math.max(padX, x0 - (x1 - x0 < 4 ? (4 - (x1 - x0)) / 2 : 0)));
        return (
          <rect
            key={`${iv.kind}-${idx}`}
            x={drawX}
            y={padY}
            width={w}
            height={plotH}
            className={`singularity-svg-band${iv.kind === 'deadzone' ? ' is-deadzone' : ''}`}
          />
        );
      })}
      {segments.map((pts, i) => (
        pts.length > 1 ? (
          <polyline key={i} points={pts.join(' ')} className="singularity-svg-curve" />
        ) : (
          <circle key={i} cx={pts[0].split(',')[0]} cy={pts[0].split(',')[1]} r="2.2" className="singularity-svg-point" />
        )
      ))}
      <line x1={curX} y1={padY} x2={curX} y2={padY + plotH} className="singularity-cursor-line" />
      <circle cx={curX} cy={padY + 4} r="2.8" className="singularity-cursor-dot" />
    </svg>
  );
}

const SingularitiesTab = memo(function SingularitiesTab({
  joints,
  toolFrame,
  mode,
  setMode,
  length,
  setLength,
  damping,
  setDamping,
  onChangeSetting,
  diagnostics,
  singularityReport,
  applyJointPose,
  onComputeWorkspace,
  computingCloud,
  hasWorkspace,
  showWorkspace,
  setShowWorkspace,
  showEllipsoid,
  setShowEllipsoid,
  showSingularityHighlights = true,
  setShowSingularityHighlights,
  activeSingularityHighlight = null,
  onHoverSingularity,
  computingSingularities = false,
  numeric,
}) {
  if (!singularityReport) {
    return (
      <section>
        <h2>Estado atual e Gimbal Lock</h2>
        <Notice as="p">Calculando mapa de singularidades em segundo plano...</Notice>
      </section>
    );
  }

  const pitchDeg = toolFrame.orientation.rpy[1];
  const gimbalGap = Math.min(Math.abs(pitchDeg - 90), Math.abs(pitchDeg + 90));
  const nearGimbalLock = gimbalGap <= 10;
  const nearConditionLimit = diagnostics.deficient || (Number.isFinite(diagnostics.condition) && diagnostics.condition > 1000);
  const lowDexterity = singularityReport.currentDexterityPct < 20;

  let statusVariant = 'success';
  let statusText = 'Pose atual bem condicionada (longe de singularidades cinemáticas).';
  if (diagnostics.deficient) {
    statusVariant = 'warning';
    statusText = 'Perda de posto no Jacobiano: o robô está em singularidade cinemática ou a tarefa possui menos graus que dimensões.';
  } else if (nearConditionLimit || lowDexterity) {
    statusVariant = 'warning';
    statusText = `Proximidade de singularidade: destreza em ${singularityReport.currentDexterityPct}% do máximo (κ = ${fmt(diagnostics.condition, 1)}).`;
  }

  return (
    <>
      <section>
        <div className="section-title">
          <h2>Estado atual e Gimbal Lock</h2>
          {computingSingularities && <span className="muted">Atualizando...</span>}
        </div>
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
        <dl className="readings">
          <dt>Destreza relativa (Yoshikawa)</dt>
          <dd>{singularityReport.currentDexterityPct}% ({fmt(singularityReport.currentW, 2)} / {fmt(singularityReport.globalMaxW, 2)})</dd>
          <dt>Posto do Jacobiano</dt>
          <dd>{diagnostics.rank} / {diagnostics.maxRank}</dd>
          <dt>Condição normalizada (κ)</dt>
          <dd>{fmt(diagnostics.condition, 2)}</dd>
          <dt>Distância ao Gimbal Lock (Pitch ±90°)</dt>
          <dd>{fmt(gimbalGap, 1)}° (Pitch {fmt(pitchDeg, 1)}°)</dd>
        </dl>
        <Notice as="p" variant={statusVariant}>
          {statusText}
          {nearGimbalLock ? ' Atenção: orientação RPY próxima ao Gimbal Lock (±90°).' : ''}
        </Notice>
        <div className="action-row singularity-visual-actions">
          <Button
            disabled={computingCloud}
            onClick={() => {
              if (!hasWorkspace) onComputeWorkspace();
              else setShowWorkspace(true);
            }}
          >
            <Orbit size={15} />
            {computingCloud ? 'Calculando nuvem...' : (hasWorkspace && showWorkspace ? 'Nuvem 3D ativa (azul = singularidade)' : 'Ver singularidades na nuvem 3D')}
          </Button>
          <Checkbox checked={showEllipsoid} onChange={setShowEllipsoid}>
            Elipsoide 3D
          </Checkbox>
          {setShowSingularityHighlights && (
            <Checkbox checked={showSingularityHighlights} onChange={setShowSingularityHighlights}>
              Destaques 3D na cena
            </Checkbox>
          )}
        </div>
      </section>

      <section>
        <h2>Mapa por eixo cartesiano (X, Y, Z)</h2>
        <p className="muted">
          Perfis de manipulabilidade mantendo as outras coordenadas em ({fmt(singularityReport.referencePos[0], 2)}, {fmt(singularityReport.referencePos[1], 2)}, {fmt(singularityReport.referencePos[2], 2)}) u. Faixas vermelhas indicam perda de destreza ou zonas mortas internas.
        </p>
        <div className="singularity-axis-list">
          {singularityReport.axes.map(axisData => (
            <div className="singularity-card" key={axisData.label}>
              <div className="singularity-card-header">
                <strong>Eixo {axisData.label} (atual: {fmt(axisData.currentCoord, 2)} u)</strong>
                <span className="muted">[{fmt(axisData.min, 2)}, {fmt(axisData.max, 2)}] u</span>
              </div>
              <AxisManipulabilityChart axisData={axisData} />
              {axisData.intervals.length > 0 ? (
                <ul className="singularity-interval-list">
                  {axisData.intervals.map((iv, idx) => {
                    const ivId = `iv-${axisData.axis}-${idx}`;
                    const isHighlighted = activeSingularityHighlight?.id === ivId;
                    const payload = {
                      kind: 'interval',
                      id: ivId,
                      axis: axisData.axis,
                      start: iv.start,
                      end: iv.end,
                      label: iv.label,
                    };
                    return (
                      <li
                        key={`${axisData.label}-${idx}`}
                        tabIndex={0}
                        className={`singularity-interval-item is-${iv.kind} is-interactive${isHighlighted ? ' is-highlighted' : ''}`}
                        onMouseEnter={() => onHoverSingularity?.(payload)}
                        onMouseLeave={() => onHoverSingularity?.(prev => (prev?.id === ivId ? null : prev))}
                        onFocus={() => onHoverSingularity?.(payload)}
                        onBlur={() => onHoverSingularity?.(prev => (prev?.id === ivId ? null : prev))}
                      >
                        <AlertTriangle size={12} aria-hidden="true" />
                        <span>{iv.label}</span>
                      </li>
                    );
                  })}
                </ul>
              ) : (
                <span className="muted">Nenhuma faixa crítica detectada ao longo desta reta.</span>
              )}
            </div>
          ))}
        </div>
      </section>

      <section>
        <h2>Ângulos e cursos críticos por junta</h2>
        <p className="muted">
          Varredura individual de cada junta mantendo as restantes fixas; valores abaixo de 25% do pico local indicam colinearidade, dobramento ou limite de curso.
        </p>
        {singularityReport.jointSweeps.length === 0 ? (
          <p className="muted">Nenhuma queda acentuada isolada foi detectada nas juntas nesta configuração.</p>
        ) : (
          <div className="singularity-joint-list">
            {singularityReport.jointSweeps.map(sweep => (
              <div className="singularity-card" key={sweep.id}>
                <div className="singularity-card-header">
                  <strong>Junta {sweep.jointIndex + 1} ({sweep.type})</strong>
                  <span className="muted">Atual: {fmt(sweep.currentVal, sweep.type === 'R' ? 1 : 2)} {sweep.unit}</span>
                </div>
                <div className="singularity-chip-row">
                  {sweep.criticalValues.map((cv, idx) => {
                    const swId = `sw-${sweep.jointIndex}-${idx}`;
                    const swPayload = {
                      kind: 'sweep',
                      id: swId,
                      pos: cv.pos,
                      wPct: cv.wPct,
                      label: `J${sweep.jointIndex + 1} = ${cv.val} ${sweep.unit}`,
                    };
                    return (
                      <Button
                        key={idx}
                        aria-label={`Testar J${sweep.jointIndex + 1} em ${cv.val} ${sweep.unit}`}
                        onMouseEnter={() => onHoverSingularity?.(swPayload)}
                        onMouseLeave={() => onHoverSingularity?.(prev => (prev?.id === swId ? null : prev))}
                        onFocus={() => onHoverSingularity?.(swPayload)}
                        onBlur={() => onHoverSingularity?.(prev => (prev?.id === swId ? null : prev))}
                        onClick={() => {
                          onHoverSingularity?.(swPayload);
                          applyJointPose(cv.qVec, `Junta ${sweep.jointIndex + 1} em ${cv.val} ${sweep.unit} (destreza ${cv.wPct}%)`);
                        }}
                      >
                        <Eye size={13} />
                        {fmt(cv.val, sweep.type === 'R' ? 0 : 2)} {sweep.unit} ({cv.wPct}%)
                      </Button>
                    );
                  })}
                </div>
              </div>
            ))}
          </div>
        )}
      </section>

      <section>
        <h2>Poses críticas encontradas no espaço 3D</h2>
        <p className="muted">
          Clique para posicionar o manipulador em cada região de baixa manipulabilidade descoberta no espaço de trabalho.
        </p>
        {singularityReport.criticalPoses.length === 0 ? (
          <p className="muted">Nenhuma pose singular crítica encontrada na amostragem atual.</p>
        ) : (
          <ul className="singularity-pose-list">
            {singularityReport.criticalPoses.map((pose, idx) => {
              const isHighlighted = activeSingularityHighlight?.id === pose.id;
              const posePayload = {
                kind: 'pose',
                id: pose.id,
                pos: pose.pos,
                wPct: pose.wPct,
                label: pose.label,
              };
              return (
                <li
                  className={`singularity-card singularity-pose-item${isHighlighted ? ' is-highlighted' : ''}`}
                  key={pose.id}
                  onMouseEnter={() => onHoverSingularity?.(posePayload)}
                  onMouseLeave={() => onHoverSingularity?.(prev => (prev?.id === pose.id ? null : prev))}
                >
                  <div className="singularity-pose-info">
                    <div className="singularity-card-header">
                      <strong>{idx + 1}. {pose.label}</strong>
                      <span className="singularity-pct-badge">w = {pose.wPct}%</span>
                    </div>
                    <span className="singularity-coords">
                      XYZ ({fmt(pose.pos[0], 2)}, {fmt(pose.pos[1], 2)}, {fmt(pose.pos[2], 2)}) u · q [{pose.q.map((v, k) => `${fmt(v, joints[k]?.type === 'R' ? 0 : 2)}${joints[k]?.type === 'R' ? '°' : 'u'}`).join(', ')}]
                    </span>
                  </div>
                  <Button
                    aria-label={`Aplicar pose singular ${idx + 1}`}
                    onFocus={() => onHoverSingularity?.(posePayload)}
                    onBlur={() => onHoverSingularity?.(prev => (prev?.id === pose.id ? null : prev))}
                    onClick={() => {
                      onHoverSingularity?.(posePayload);
                      applyJointPose(pose.q, `Pose singular ${idx + 1} aplicada (${pose.wPct}% de destreza).`);
                    }}
                  >
                    <Crosshair size={14} />Aplicar
                  </Button>
                </li>
              );
            })}
          </ul>
        )}
      </section>
    </>
  );
});

export default SingularitiesTab;
