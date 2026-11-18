import { memo } from 'react';
import { Focus, Pause, Play, Plus, Repeat, Square, Trash2 } from 'lucide-react';
import { Button, Checkbox, SegmentedTabs, Select, Slider } from '../ui';
import { DEG } from '../kinematics';
import { fmt } from '../format';
import TrajectoryCharts from '../TrajectoryCharts';

const TrajectoryTab = memo(function TrajectoryTab({
  joints,
  modelStyle,
  waypoints,
  trajectoryData,
  playTime,
  setPlayTime,
  isPlaying,
  setIsPlaying,
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
  seekTrajectory,
  addCurrentWaypoint,
  moveToWaypoint,
  removeWaypoint,
  updateWaypointDuration,
}) {
  return (
    <>
      <section>
        <h2>Sequenciador de Trajetórias</h2>
        <p className="muted">Interpolação contínua de movimentos articulares via splines quínticas e cúbicas sem descontinuidades nos atuadores.</p>

        <div className="timeline-panel">
          <div className="timeline-slider-row">
            <span>{fmt(playTime, 2)} s</span>
            <Slider
              className="timeline-slider"
              ariaLabel="Timeline da trajetória"
              min={0}
              max={Math.max(0.01, trajectoryData.totalDuration)}
              value={playTime}
              onChange={seekTrajectory}
            />
            <span>{fmt(trajectoryData.totalDuration, 2)} s</span>
          </div>

          <div className="timeline-controls">
            <div className="timeline-btns">
              <Button
                variant="primary"
                aria-label={isPlaying ? 'Pausar reprodução' : 'Iniciar reprodução'}
                onClick={() => {
                  if (playTime >= trajectoryData.totalDuration) setPlayTime(0);
                  setIsPlaying(p => !p);
                }}
              >
                {isPlaying ? <Pause size={16} /> : <Play size={16} />}
                {isPlaying ? 'Pausar' : 'Reproduzir'}
              </Button>
              <Button
                aria-label="Parar e reiniciar timeline"
                onClick={() => {
                  setIsPlaying(false);
                  seekTrajectory(0);
                }}
              >
                <Square size={16} /> Parar
              </Button>
            </div>

            <div className="timeline-opts">
              <label>
                Velocidade:
                <Select
                  compact
                  ariaLabel="Velocidade de reprodução"
                  value={playbackSpeed}
                  onChange={val => setPlaybackSpeed(Number(val))}
                  options={[
                    { value: 0.5, label: '0.5x' },
                    { value: 1.0, label: '1.0x' },
                    { value: 1.5, label: '1.5x' },
                    { value: 2.0, label: '2.0x' },
                  ]}
                />
              </label>
              <Checkbox checked={isLooping} onChange={setIsLooping}>
                <Repeat size={14} /> Loop
              </Checkbox>
            </div>
          </div>
        </div>

        <div className="field-grid">
          <label className="field-span-full">Modo de Trajetória
            <Select
              ariaLabel="Tipo de interpolação"
              value={interpType}
              onChange={setInterpType}
              options={[
                { value: 'quintic', label: 'Quíntico PTP (para em cada ponto, sem solavanco)' },
                { value: 'cubic', label: 'Cúbico PTP (para em cada ponto)' },
                { value: 'trapezoidal', label: 'Trapezoidal — LSPB (aceleração e cruzeiro constantes)' },
                { value: 'continuous', label: 'Spline Contínua (passagem suave sem parar)' },
                { value: 'cartesian', label: 'Linear Cartesiano — LIN (linha reta 3D)' },
              ]}
            />
          </label>
          <div className="field-action field-span-full">
            <Button block onClick={addCurrentWaypoint}>
              <Plus size={16} /> Adicionar pose atual
            </Button>
          </div>
        </div>
      </section>

      <section>
        <div className="section-title">
          <h2>Perfis Cinemáticos (q, q̇, q̈, |v_TCP|)</h2>
        </div>
        <p className="muted">Clique ou arraste sobre o gráfico para inspecionar e sincronizar qualquer instante t na cena 3D.</p>
        <SegmentedTabs
          ariaLabel="Unidade angular da trajetória"
          activeId={angleUnit}
          onChange={setAngleUnit}
          items={[
            { id: 'deg', label: 'Graus (°)' },
            { id: 'rad', label: 'Radianos (rad)' },
          ]}
        />
        <TrajectoryCharts
          trajectoryData={trajectoryData}
          joints={joints}
          modelStyle={modelStyle}
          playTime={playTime}
          interpType={interpType}
          angleUnit={angleUnit}
          onSeek={(t) => {
            setIsPlaying(false);
            seekTrajectory(t);
          }}
        />
      </section>

      <section>
        <div className="section-title">
          <h2>Waypoints da sequência ({waypoints.length})</h2>
          <Checkbox checked={showTrajectory} onChange={setShowTrajectory}>
            Trilha 3D
          </Checkbox>
        </div>
        <ul className="waypoint-list">
          {waypoints.map((wp, idx) => (
            <li key={wp.id} className="waypoint-item">
              <div className="waypoint-info">
                <strong>{idx + 1}. {wp.name}</strong>
                <span className="waypoint-coords">
                  q: [{wp.q.map((v, jIdx) => {
                    const isRev = joints[jIdx]?.type === 'R';
                    if (isRev) {
                      return angleUnit === 'rad' ? `${fmt(v * DEG, 3)} rad` : `${fmt(v, 1)}°`;
                    }
                    return `${fmt(v, 2)} u`;
                  }).join(', ')}]
                </span>
              </div>
              {idx > 0 && (
                <div className="waypoint-duration">
                  <span className="muted">Δt:</span>
                  <input
                    type="number"
                    className="ui-input ui-input-number compact"
                    aria-label={`Duração do waypoint ${idx + 1}`}
                    min={0.1}
                    max={60}
                    step={0.5}
                    defaultValue={wp.duration}
                    onBlur={e => updateWaypointDuration(wp.id, Number(e.target.value))}
                  />
                  <span className="muted">s</span>
                </div>
              )}
              <div className="item-actions">
                <Button icon title="Ir para waypoint" aria-label={`Ir para ${wp.name}`} onClick={() => moveToWaypoint(wp)}>
                  <Focus size={15} />
                </Button>
                <Button
                  icon
                  variant="danger"
                  title="Remover waypoint"
                  aria-label={`Remover ${wp.name}`}
                  disabled={waypoints.length <= 1}
                  onClick={() => removeWaypoint(wp.id)}
                >
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

export default TrajectoryTab;
