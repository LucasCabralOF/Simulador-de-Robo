import { useMemo, useRef, useState } from 'react';
import { DEG } from './kinematics';
import { SegmentedTabs } from './ui';

const VIEWBOX_WIDTH = 420;
const VIEWBOX_HEIGHT = 188;
const PAD_LEFT = 46;
const PAD_RIGHT = 12;
const PAD_TOP = 18;
const PAD_BOTTOM = 26;
const PLOT_WIDTH = VIEWBOX_WIDTH - PAD_LEFT - PAD_RIGHT;
const PLOT_HEIGHT = VIEWBOX_HEIGHT - PAD_TOP - PAD_BOTTOM;

const JOINT_COLORS = [
  '#62d5b5',
  '#4da6ff',
  '#f3b74f',
  '#f17d8b',
  '#b59cff',
  '#ff9868',
  '#7cd96e',
  '#e58fd8',
];

const METRIC_TABS = [
  { id: 'q', label: 'Posição q(t)' },
  { id: 'v', label: 'Velocidade q̇(t)' },
  { id: 'a', label: 'Aceleração q̈(t)' },
  { id: 'tcp', label: 'Vel. TCP |v|' },
];

const INTERP_NOTES = {
  quintic: 'Quíntico PTP: velocidade e aceleração nulas no início e fim de cada segmento.',
  cubic: 'Cúbico PTP: velocidade nula nos waypoints, com descontinuidade em q̈(t).',
  trapezoidal: 'Trapezoidal (LSPB): aceleração constante (±a), velocidade de cruzeiro constante e desaceleração simétrica.',
  continuous: 'Spline Contínua (C²): passagem sem paradas intermediárias nos waypoints.',
  cartesian: 'Linear Cartesiano (LIN): velocidade articular ajustada para manter linha reta 3D.',
};

function formatCompact(n) {
  if (!Number.isFinite(n)) return '0';
  const abs = Math.abs(n);
  if (abs < 1e-6) return '0';
  if (abs >= 100) return n.toFixed(0);
  if (abs >= 10) return n.toFixed(1);
  return n.toFixed(2);
}

function formatUnitValue(val, type, metric, angleUnit = 'deg') {
  if (!Number.isFinite(val)) return '-';
  const rounded = Math.abs(val) < 1e-4 ? 0 : val;
  if (metric === 'tcp') return `${rounded.toFixed(2)} u/s`;
  const unit = type === 'R' ? (angleUnit === 'rad' ? 'rad' : '°') : 'u';
  const suffix = metric === 'q' ? unit : metric === 'v' ? `${unit}/s` : `${unit}/s²`;
  const digits = type === 'R' ? (angleUnit === 'rad' ? 3 : 1) : 2;
  return `${rounded.toFixed(digits)} ${suffix}`;
}

function interpolateSeriesAtTime(times, values, targetTime) {
  if (!times || times.length === 0 || !values || values.length === 0) return 0;
  if (targetTime <= times[0]) return values[0];
  const lastIdx = times.length - 1;
  if (targetTime >= times[lastIdx]) return values[lastIdx];

  let lo = 0;
  let hi = lastIdx;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (times[mid] <= targetTime) lo = mid;
    else hi = mid;
  }
  const span = times[hi] - times[lo];
  if (span <= 1e-9) return values[lo];
  const frac = (targetTime - times[lo]) / span;
  return values[lo] + frac * (values[hi] - values[lo]);
}

function buildPolylinePath(times, values, xOf, yOf) {
  let d = '';
  for (let i = 0; i < times.length; i++) {
    const x = xOf(times[i]);
    const y = yOf(values[i]);
    d += i === 0 ? `M ${x.toFixed(2)} ${y.toFixed(2)}` : ` L ${x.toFixed(2)} ${y.toFixed(2)}`;
  }
  return d;
}

export default function TrajectoryCharts({
  trajectoryData,
  playTime = 0,
  interpType = 'quintic',
  angleUnit = 'deg',
  onSeek,
}) {
  const [metric, setMetric] = useState('q');
  const [hiddenJoints, setHiddenJoints] = useState([]);
  const svgRef = useRef(null);
  const draggingRef = useRef(false);

  const times = trajectoryData?.times;
  const waypointTimes = trajectoryData?.waypointTimes;
  const jointProfiles = trajectoryData?.jointProfiles;
  const tcpSpeeds = trajectoryData?.tcpSpeeds;
  const totalDuration = trajectoryData?.totalDuration || 0;

  const hasData = Array.isArray(times) && times.length >= 2 && totalDuration > 0;
  const clampedTime = Math.max(0, Math.min(totalDuration, playTime));
  const rScale = angleUnit === 'rad' ? DEG : 1;

  const scaledProfiles = useMemo(() => {
    if (!Array.isArray(jointProfiles)) return [];
    if (rScale === 1) return jointProfiles;
    return jointProfiles.map(jp => (jp.type === 'R'
      ? {
          ...jp,
          q: jp.q.map(v => v * rScale),
          v: jp.v.map(v => v * rScale),
          a: jp.a.map(v => v * rScale),
        }
      : jp));
  }, [jointProfiles, rScale]);

  const peaks = useMemo(() => {
    if (!hasData || !Array.isArray(scaledProfiles)) {
      return { maxVelR: 0, maxVelP: 0, maxAccR: 0, maxAccP: 0, maxTcp: 0, hasR: false, hasP: false };
    }
    let maxVelR = 0;
    let maxVelP = 0;
    let maxAccR = 0;
    let maxAccP = 0;
    let hasR = false;
    let hasP = false;

    for (const jp of scaledProfiles) {
      if (jp.type === 'R') {
        hasR = true;
        for (const v of jp.v) maxVelR = Math.max(maxVelR, Math.abs(v));
        for (const a of jp.a) maxAccR = Math.max(maxAccR, Math.abs(a));
      } else {
        hasP = true;
        for (const v of jp.v) maxVelP = Math.max(maxVelP, Math.abs(v));
        for (const a of jp.a) maxAccP = Math.max(maxAccP, Math.abs(a));
      }
    }

    let maxTcp = 0;
    if (Array.isArray(tcpSpeeds)) {
      for (const s of tcpSpeeds) maxTcp = Math.max(maxTcp, Math.abs(s));
    }

    return { maxVelR, maxVelP, maxAccR, maxAccP, maxTcp, hasR, hasP };
  }, [hasData, scaledProfiles, tcpSpeeds]);

  const activeSeries = useMemo(() => {
    if (!hasData) return [];
    if (metric === 'tcp') {
      return [{
        key: 'tcp',
        label: 'TCP |v|',
        type: 'P',
        color: '#62d5b5',
        values: tcpSpeeds || [],
      }];
    }
    return scaledProfiles
      .filter(jp => !hiddenJoints.includes(jp.index))
      .map(jp => ({
        key: `j-${jp.index}`,
        label: jp.label,
        type: jp.type,
        color: JOINT_COLORS[jp.index % JOINT_COLORS.length],
        values: jp[metric] || [],
      }));
  }, [hasData, metric, tcpSpeeds, scaledProfiles, hiddenJoints]);

  const { yMin, yMax, yTicks } = useMemo(() => {
    let lo = Infinity;
    let hi = -Infinity;
    for (const series of activeSeries) {
      for (const val of series.values) {
        if (Number.isFinite(val)) {
          if (val < lo) lo = val;
          if (val > hi) hi = val;
        }
      }
    }
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
      return { yMin: -1, yMax: 1, yTicks: [-1, -0.5, 0, 0.5, 1] };
    }
    if (metric === 'tcp') {
      lo = 0;
      hi = Math.max(0.1, hi * 1.1);
    } else if (hi - lo < 1e-4) {
      const center = (lo + hi) / 2;
      lo = center - 1;
      hi = center + 1;
    } else {
      const margin = (hi - lo) * 0.08;
      lo -= margin;
      hi += margin;
    }
    const ticks = Array.from({ length: 5 }, (_, i) => lo + ((hi - lo) * (4 - i)) / 4);
    return { yMin: lo, yMax: hi, yTicks: ticks };
  }, [activeSeries, metric]);

  function toggleJoint(index) {
    setHiddenJoints(prev => (
      prev.includes(index) ? prev.filter(i => i !== index) : [...prev, index]
    ));
  }

  function scrubFromClientX(clientX) {
    if (!onSeek || !svgRef.current || totalDuration <= 0) return;
    const rect = svgRef.current.getBoundingClientRect();
    if (rect.width <= 0) return;
    const svgX = ((clientX - rect.left) / rect.width) * VIEWBOX_WIDTH;
    const ratio = Math.max(0, Math.min(1, (svgX - PAD_LEFT) / PLOT_WIDTH));
    onSeek(ratio * totalDuration);
  }

  function handlePointerDown(e) {
    if (!hasData) return;
    draggingRef.current = true;
    e.currentTarget.setPointerCapture?.(e.pointerId);
    scrubFromClientX(e.clientX);
  }

  function handlePointerMove(e) {
    if (!draggingRef.current) return;
    scrubFromClientX(e.clientX);
  }

  function handlePointerEnd(e) {
    if (!draggingRef.current) return;
    draggingRef.current = false;
    e.currentTarget.releasePointerCapture?.(e.pointerId);
  }

  if (!hasData) {
    return (
      <div className="trajectory-charts-empty muted">
        Adicione pelo menos 2 waypoints com duração positiva para visualizar os perfis cinemáticos.
      </div>
    );
  }

  const xOf = (t) => PAD_LEFT + (t / totalDuration) * PLOT_WIDTH;
  const yOf = (val) => PAD_TOP + ((yMax - val) / Math.max(1e-9, yMax - yMin)) * PLOT_HEIGHT;
  const playheadX = xOf(clampedTime);
  const currentTcpSpeed = interpolateSeriesAtTime(times, tcpSpeeds, clampedTime);
  const showZeroLine = yMin < 0 && yMax > 0;

  return (
    <div className="trajectory-charts">
      <SegmentedTabs
        ariaLabel="Grandeza do gráfico de trajetória"
        items={METRIC_TABS}
        activeId={metric}
        onChange={setMetric}
      />

      <p className="muted chart-interp-note">{INTERP_NOTES[interpType] || INTERP_NOTES.quintic}</p>

      <div className="chart-frame">
        <svg
          ref={svgRef}
          className="kinematic-svg"
          viewBox={`0 0 ${VIEWBOX_WIDTH} ${VIEWBOX_HEIGHT}`}
          role="img"
          aria-label="Gráfico cinemático da trajetória"
          data-testid="trajectory-kinematic-chart"
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerEnd}
          onPointerCancel={handlePointerEnd}
        >
          {/* Horizontal grid lines and Y-axis ticks */}
          {yTicks.map((tickVal, idx) => {
            const y = yOf(tickVal);
            return (
              <g key={idx}>
                <line
                  x1={PAD_LEFT}
                  x2={PAD_LEFT + PLOT_WIDTH}
                  y1={y}
                  y2={y}
                  className="chart-grid-line"
                />
                <text x={PAD_LEFT - 6} y={y + 3} className="chart-axis-text" textAnchor="end">
                  {formatCompact(tickVal)}
                </text>
              </g>
            );
          })}

          {showZeroLine && (
            <line
              x1={PAD_LEFT}
              x2={PAD_LEFT + PLOT_WIDTH}
              y1={yOf(0)}
              y2={yOf(0)}
              className="chart-zero-line"
            />
          )}

          {/* Waypoint knot vertical markers */}
          {(waypointTimes || []).map((wt, idx) => {
            const x = xOf(wt);
            return (
              <g key={idx}>
                <line
                  x1={x}
                  x2={x}
                  y1={PAD_TOP}
                  y2={PAD_TOP + PLOT_HEIGHT}
                  className="chart-waypoint-line"
                />
                <text x={x} y={PAD_TOP - 5} className="chart-waypoint-label" textAnchor="middle">
                  P{idx + 1}
                </text>
              </g>
            );
          })}

          {/* Area under curve for TCP speed */}
          {metric === 'tcp' && activeSeries[0]?.values.length === times.length && (
            <path
              d={`${buildPolylinePath(times, activeSeries[0].values, xOf, yOf)} L ${xOf(totalDuration).toFixed(2)} ${yOf(0).toFixed(2)} L ${xOf(0).toFixed(2)} ${yOf(0).toFixed(2)} Z`}
              fill="rgba(98, 213, 181, 0.14)"
            />
          )}

          {/* Data polylines */}
          {activeSeries.map(series => (
            <path
              key={series.key}
              d={buildPolylinePath(times, series.values, xOf, yOf)}
              fill="none"
              stroke={series.color}
              strokeWidth={metric === 'tcp' ? 2.2 : 1.8}
              strokeLinejoin="round"
              strokeLinecap="round"
              data-series-key={series.key}
            />
          ))}

          {/* Playhead cursor + instantaneous markers */}
          <line
            x1={playheadX}
            x2={playheadX}
            y1={PAD_TOP}
            y2={PAD_TOP + PLOT_HEIGHT}
            className="chart-playhead-line"
            data-testid="trajectory-playhead"
          />
          {activeSeries.map(series => {
            const val = interpolateSeriesAtTime(times, series.values, clampedTime);
            return (
              <circle
                key={`${series.key}-dot`}
                cx={playheadX}
                cy={yOf(val)}
                r={3.2}
                fill={series.color}
                stroke="#121518"
                strokeWidth={1.2}
              />
            );
          })}

          {/* X-axis baseline & timestamps */}
          <line
            x1={PAD_LEFT}
            x2={PAD_LEFT + PLOT_WIDTH}
            y1={PAD_TOP + PLOT_HEIGHT}
            y2={PAD_TOP + PLOT_HEIGHT}
            className="chart-axis-line"
          />
          <text x={PAD_LEFT} y={VIEWBOX_HEIGHT - 7} className="chart-axis-text" textAnchor="start">
            0.0s
          </text>
          <text x={PAD_LEFT + PLOT_WIDTH / 2} y={VIEWBOX_HEIGHT - 7} className="chart-axis-text" textAnchor="middle">
            {`${(totalDuration / 2).toFixed(2)}s`}
          </text>
          <text x={PAD_LEFT + PLOT_WIDTH} y={VIEWBOX_HEIGHT - 7} className="chart-axis-text" textAnchor="end">
            {`${totalDuration.toFixed(2)}s`}
          </text>
        </svg>
      </div>

      {metric === 'tcp' ? (
        <div className="chart-legend" aria-label="Leitura instantânea do TCP">
          <span className="chart-legend-chip is-static">
            <span className="chart-swatch" style={{ backgroundColor: '#62d5b5' }} />
            <strong>|v_TCP|(t)</strong>
            <code>{formatUnitValue(currentTcpSpeed, 'P', 'tcp')}</code>
          </span>
        </div>
      ) : (
        <div className="chart-legend" aria-label="Legenda e filtro das juntas">
          {scaledProfiles.map(jp => {
            const color = JOINT_COLORS[jp.index % JOINT_COLORS.length];
            const hidden = hiddenJoints.includes(jp.index);
            const currentVal = interpolateSeriesAtTime(times, jp[metric], clampedTime);
            return (
              <button
                key={jp.index}
                type="button"
                className={`chart-legend-chip ${hidden ? 'is-hidden' : ''}`}
                aria-pressed={!hidden}
                title={`Clique para ${hidden ? 'exibir' : 'ocultar'} ${jp.label}`}
                onClick={() => toggleJoint(jp.index)}
              >
                <span className="chart-swatch" style={{ backgroundColor: color }} />
                <strong>{jp.label} ({jp.type})</strong>
                <code>{formatUnitValue(currentVal, jp.type, metric, angleUnit)}</code>
              </button>
            );
          })}
        </div>
      )}

      <dl className="chart-stats">
        {peaks.hasR && (
          <div>
            <dt>Pico |q̇| (R)</dt>
            <dd>{angleUnit === 'rad' ? `${peaks.maxVelR.toFixed(3)} rad/s` : `${peaks.maxVelR.toFixed(1)} °/s`}</dd>
          </div>
        )}
        {peaks.hasP && (
          <div>
            <dt>Pico |q̇| (P)</dt>
            <dd>{peaks.maxVelP.toFixed(2)} u/s</dd>
          </div>
        )}
        {peaks.hasR && (
          <div>
            <dt>Pico |q̈| (R)</dt>
            <dd>{angleUnit === 'rad' ? `${peaks.maxAccR.toFixed(3)} rad/s²` : `${peaks.maxAccR.toFixed(1)} °/s²`}</dd>
          </div>
        )}
        {peaks.hasP && (
          <div>
            <dt>Pico |q̈| (P)</dt>
            <dd>{peaks.maxAccP.toFixed(2)} u/s²</dd>
          </div>
        )}
        <div>
          <dt>Pico |v_TCP|</dt>
          <dd>{peaks.maxTcp.toFixed(2)} u/s</dd>
        </div>
      </dl>
    </div>
  );
}
