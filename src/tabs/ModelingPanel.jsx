import { memo, useState } from 'react';
import { Palette } from 'lucide-react';
import { Button, SegmentedTabs, Select, Slider } from '../ui';
import {
  DEFAULT_JOINT_COLORS,
  JOINT_CHART_PALETTE,
  LINK_SHAPE_LIST,
  MODEL_STYLE_BOUNDS,
  hexToHsl,
  hslToHex,
  normalizeHexColor,
  normalizeModelStyle,
  resolveJointModelStyle,
} from '../kinematics';
import { fmt } from '../format';

const MODEL_STYLE_SLIDERS = [
  { key: 'linkRadius', label: 'Raio das hastes (u)', hint: 'Espessura radial/transversal dos elos', step: 0.005, digits: 3 },
  { key: 'revRadius', label: 'Raio da junta rotativa R (u)', hint: 'Diâmetro visual dos motores R', step: 0.01, digits: 2 },
  { key: 'revLength', label: 'Altura da junta rotativa R (u)', hint: 'Comprimento axial dos cilindros R', step: 0.02, digits: 2 },
  { key: 'prismWidth', label: 'Largura da junta prismática P (u)', hint: 'Seção do bloco/guia P', step: 0.01, digits: 2 },
  { key: 'prismLength', label: 'Comprimento da junta prismática P (u)', hint: 'Extensão da manga deslizante P', step: 0.02, digits: 2 },
];

function ModelColorChip({ label, ariaLabel, value, active = false, onSelect, onChange }) {
  return (
    <label className={`model-color-chip${active ? ' is-active' : ''}`} onClick={onSelect}>
      <input
        type="color"
        className="model-color-picker"
        aria-label={ariaLabel}
        value={value}
        onChange={e => {
          onSelect?.();
          onChange(e.target.value);
        }}
      />
      <span>{label}</span>
    </label>
  );
}

const ModelingPanel = memo(function ModelingPanel({
  joints,
  modelStyle,
  setModelStyle,
  modelTarget,
  setModelTarget,
  numeric,
}) {
  const [colorChannel, setColorChannel] = useState('joint');
  const [advancedColorsOpen, setAdvancedColorsOpen] = useState(false);
  const [hexEditing, setHexEditing] = useState(null);

  const activeTarget = modelTarget !== 'global' && joints.some(j => String(j.id) === modelTarget)
    ? modelTarget
    : 'global';
  const targetIdx = joints.findIndex(j => String(j.id) === activeTarget);
  const targetJoint = targetIdx >= 0 ? joints[targetIdx] : null;
  const activeStyle = targetJoint
    ? resolveJointModelStyle(modelStyle, targetJoint.id, targetJoint.type, targetIdx)
    : resolveJointModelStyle(modelStyle, null, 'R');
  const visibleSliders = targetJoint
    ? MODEL_STYLE_SLIDERS.filter(s => s.key === 'linkRadius' || (targetJoint.type === 'R' ? s.key.startsWith('rev') : s.key.startsWith('prism')))
    : MODEL_STYLE_SLIDERS;
  const hasPerJoint = Boolean(modelStyle.perJoint && Object.keys(modelStyle.perJoint).length > 0);

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

  const applyChartColorsToJoints = () => {
    setModelStyle(prev => {
      const nextPer = { ...(prev.perJoint || {}) };
      joints.forEach((j, idx) => {
        const jKey = String(j.id);
        nextPer[jKey] = {
          ...(nextPer[jKey] || {}),
          jointColor: JOINT_CHART_PALETTE[idx % JOINT_CHART_PALETTE.length],
        };
      });
      return normalizeModelStyle({ ...prev, perJoint: nextPer });
    });
  };

  const applyRainbowColorsToJoints = () => {
    const n = Math.max(1, joints.length);
    setModelStyle(prev => {
      const nextPer = { ...(prev.perJoint || {}) };
      joints.forEach((j, idx) => {
        const jKey = String(j.id);
        nextPer[jKey] = {
          ...(nextPer[jKey] || {}),
          jointColor: hslToHex(Math.round((idx * 360) / n), 82, 62),
        };
      });
      return normalizeModelStyle({ ...prev, perJoint: nextPer });
    });
  };

  const effectiveColorField = targetJoint
    ? (colorChannel === 'link' ? 'linkColor' : 'jointColor')
    : (colorChannel === 'link' ? 'linkColor' : colorChannel === 'prism' ? 'prismColor' : 'revColor');
  const activeChannelValue = activeStyle[effectiveColorField];
  const activeChannelDefault = effectiveColorField === 'linkColor'
    ? DEFAULT_JOINT_COLORS.linkColor
    : (effectiveColorField === 'prismColor' || targetJoint?.type === 'P')
      ? DEFAULT_JOINT_COLORS.prismColor
      : DEFAULT_JOINT_COLORS.revColor;
  const activeHsl = hexToHsl(activeChannelValue);
  const visibleSwatches = Array.from(new Set([
    activeChannelDefault,
    ...(advancedColorsOpen ? JOINT_CHART_PALETTE : JOINT_CHART_PALETTE.slice(0, 8)),
  ]));

  return (
    <div className="modeling-popover-content">
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
      <div className="ik-slider-card model-color-section">
        <div className="model-color-section-header">
          <strong>Cores (3D e Gráficos)</strong>
          <div className="model-color-header-actions">
            <Button
              type="button"
              className="scene-toolbar-trigger"
              aria-label="Colorir juntas pela paleta do gráfico"
              title="Aplica uma cor distinta da paleta dos gráficos em cada junta J1–Jn"
              onClick={applyChartColorsToJoints}
            >
              <Palette size={13} aria-hidden="true" />
              <span>Paleta J1–J{joints.length}</span>
            </Button>
            <Button
              type="button"
              className={`scene-toolbar-trigger${advancedColorsOpen ? ' is-active' : ''}`}
              aria-label="Expandir seletor avançado de cores"
              aria-expanded={advancedColorsOpen}
              title="Abrir 24 cores, saturação, brilho e arco-íris contínuo"
              onClick={() => setAdvancedColorsOpen(prev => !prev)}
            >
              <span>{advancedColorsOpen ? 'Menos cores' : 'Mais cores'}</span>
            </Button>
          </div>
        </div>
        <div className="model-color-grid">
          {targetJoint ? (
            <>
              <ModelColorChip
                label={`Junta J${targetIdx + 1}`}
                ariaLabel="Cor da junta"
                value={activeStyle.jointColor}
                active={effectiveColorField === 'jointColor'}
                onSelect={() => setColorChannel('joint')}
                onChange={jointColor => applyStyleEdit({ jointColor })}
              />
              <ModelColorChip
                label={`Haste ${targetIdx + 1}`}
                ariaLabel="Cor da haste"
                value={activeStyle.linkColor}
                active={effectiveColorField === 'linkColor'}
                onSelect={() => setColorChannel('link')}
                onChange={linkColor => applyStyleEdit({ linkColor })}
              />
            </>
          ) : (
            <>
              <ModelColorChip
                label="Juntas R"
                ariaLabel="Cor das juntas R"
                value={activeStyle.revColor}
                active={effectiveColorField === 'revColor'}
                onSelect={() => setColorChannel('rev')}
                onChange={revColor => applyStyleEdit({ revColor })}
              />
              <ModelColorChip
                label="Juntas P"
                ariaLabel="Cor das juntas P"
                value={activeStyle.prismColor}
                active={effectiveColorField === 'prismColor'}
                onSelect={() => setColorChannel('prism')}
                onChange={prismColor => applyStyleEdit({ prismColor })}
              />
              <ModelColorChip
                label="Hastes"
                ariaLabel="Cor da haste"
                value={activeStyle.linkColor}
                active={effectiveColorField === 'linkColor'}
                onSelect={() => setColorChannel('link')}
                onChange={linkColor => applyStyleEdit({ linkColor })}
              />
            </>
          )}
        </div>
        <div className="model-color-spectrum-row">
          <input
            type="range"
            className="model-color-hue-slider"
            aria-label="Matiz da cor (0 a 360 graus)"
            min={0}
            max={359}
            step={1}
            value={activeHsl.h}
            onChange={e => {
              const hue = Number(e.target.value);
              const sat = activeHsl.s < 12 ? 80 : activeHsl.s;
              const light = activeHsl.l < 12 || activeHsl.l > 90 ? 58 : activeHsl.l;
              applyStyleEdit({ [effectiveColorField]: hslToHex(hue, sat, light) });
            }}
          />
          <input
            type="text"
            className="ui-input compact model-color-hex-input"
            aria-label="Código hexadecimal da cor"
            maxLength={7}
            spellCheck={false}
            value={hexEditing !== null ? hexEditing : activeChannelValue}
            onFocus={() => setHexEditing(activeChannelValue)}
            onChange={e => {
              const raw = e.target.value;
              setHexEditing(raw);
              const normalized = normalizeHexColor(raw.startsWith('#') ? raw : `#${raw}`);
              if (normalized) applyStyleEdit({ [effectiveColorField]: normalized });
            }}
            onBlur={() => setHexEditing(null)}
          />
        </div>
        <div className="model-color-swatches" role="group" aria-label="Paleta rápida de cores">
          {visibleSwatches.map(hex => (
            <button
              key={hex}
              type="button"
              className={`model-color-swatch${activeChannelValue === hex ? ' is-active' : ''}`}
              style={{ backgroundColor: hex }}
              title={hex === activeChannelDefault ? `Cor padrão (${hex})` : hex}
              aria-label={`${effectiveColorField} ${hex}`}
              aria-pressed={activeChannelValue === hex}
              onClick={() => applyStyleEdit({ [effectiveColorField]: hex })}
            />
          ))}
        </div>
        {advancedColorsOpen && (
          <div className="model-color-advanced">
            <div className="model-color-hsl-sliders">
              <label>
                <span>Saturação ({activeHsl.s}%)</span>
                <input
                  type="range"
                  aria-label="Saturação da cor"
                  min={0}
                  max={100}
                  step={1}
                  value={activeHsl.s}
                  onChange={e => applyStyleEdit({ [effectiveColorField]: hslToHex(activeHsl.h, Number(e.target.value), activeHsl.l) })}
                />
              </label>
              <label>
                <span>Brilho ({activeHsl.l}%)</span>
                <input
                  type="range"
                  aria-label="Luminosidade da cor"
                  min={10}
                  max={92}
                  step={1}
                  value={Math.max(10, Math.min(92, activeHsl.l))}
                  onChange={e => applyStyleEdit({ [effectiveColorField]: hslToHex(activeHsl.h, activeHsl.s, Number(e.target.value)) })}
                />
              </label>
            </div>
            <Button
              type="button"
              className="scene-toolbar-trigger"
              aria-label="Distribuir arco-íris nas juntas"
              onClick={applyRainbowColorsToJoints}
            >
              <span>Arco-íris J1–J{joints.length} (360°)</span>
            </Button>
          </div>
        )}
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
    </div>
  );
});

export default ModelingPanel;
