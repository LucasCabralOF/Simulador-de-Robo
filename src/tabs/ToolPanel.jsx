import { memo } from 'react';
import { SegmentedTabs, Slider } from '../ui';
import { TOOL_LIST, TOOL_TYPES } from '../kinematics';
import { fmt } from '../format';

const OFFSET_AXES = [
  { axis: 'Roll', hint: 'Giro em X do TCP' },
  { axis: 'Pitch', hint: 'Inclinação em Y do TCP' },
  { axis: 'Yaw', hint: 'Azimute em Z do TCP' },
];

const ToolPanel = memo(function ToolPanel({
  toolType,
  setToolType,
  toolGrip,
  setToolGrip,
  toolScale = 1,
  setToolScale,
  toolOffsetRpy,
  setToolOffsetRpy,
  toolFrame,
  numeric,
}) {
  return (
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
      <div className="ik-slider-card tool-config-card">
        <div className="ik-slider-header">
          <div className="ik-slider-meta">
            <strong>Volume da ferramenta (escala 3D)</strong>
            <span className="muted">Tamanho geral e alcance visual da ponta ({fmt(toolScale, 2)}×)</span>
          </div>
          {numeric('Volume da ferramenta (escala)', toolScale, v => setToolScale?.(v), { min: 0.4, max: 4.0, step: 0.05, compact: true })}
        </div>
        <Slider
          ariaLabel="Controle Volume da ferramenta (escala)"
          ariaValueText={`${fmt(toolScale, 2)}x`}
          min={0.4}
          max={4.0}
          step={0.05}
          value={toolScale}
          onChange={val => setToolScale?.(Math.round(val * 100) / 100)}
        />
      </div>
      <div className="ik-joint-limits-list">
        {OFFSET_AXES.map(({ axis, hint }, idx) => (
          <div className="ik-slider-card" key={axis}>
            <div className="ik-slider-header">
              <div className="ik-slider-meta">
                <strong>Offset {axis} (°)</strong>
                <span className="muted">{hint}</span>
              </div>
              {numeric(`Offset ${axis} (°)`, toolOffsetRpy[idx], v => setToolOffsetRpy(prev => prev.map((cur, i) => (i === idx ? v : cur))), { min: -180, max: 180, compact: true })}
            </div>
            <Slider
              ariaLabel={`Deslizar Offset ${axis} (°)`}
              ariaValueText={`${fmt(toolOffsetRpy[idx], 1)}°`}
              min={-180}
              max={180}
              step={1}
              value={toolOffsetRpy[idx]}
              onChange={val => setToolOffsetRpy(prev => prev.map((cur, i) => (i === idx ? val : cur)))}
            />
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
  );
});

export default ToolPanel;
