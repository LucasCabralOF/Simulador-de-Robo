export function Slider({
  min = 0,
  max = 100,
  step = 'any',
  value,
  onChange,
  onPointerDown,
  onPointerUp,
  onPointerCancel,
  onBlur,
  disabled = false,
  ariaLabel,
  ariaValueText,
  markers = [],
  className = '',
}) {
  const span = max - min;
  const pct = span > 0 && Number.isFinite(value)
    ? Math.max(0, Math.min(100, ((value - min) / span) * 100))
    : 0;
  const validMarkers = span > 0 && Array.isArray(markers) && markers.length > 0
    ? markers.filter(m => {
        const s = Number.isFinite(m?.start) ? m.start : m?.value;
        const e = Number.isFinite(m?.end) ? m.end : m?.value;
        return Number.isFinite(s) && Number.isFinite(e) && e >= min - 1e-4 && s <= max + 1e-4;
      })
    : [];

  return (
    <div className={`ui-slider-wrap${disabled ? ' is-disabled' : ''}${className ? ` ${className}` : ''}`}>
      <input
        type="range"
        className="ui-slider"
        aria-label={ariaLabel}
        aria-valuetext={ariaValueText}
        min={min}
        max={max}
        step={step}
        disabled={disabled}
        value={value}
        style={{ '--slider-progress': `${pct}%` }}
        onChange={e => onChange?.(Number(e.target.value), e)}
        onPointerDown={onPointerDown}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerCancel}
        onBlur={onBlur}
      />
      {validMarkers.length > 0 && (
        <div className="ui-slider-markers" aria-hidden="true">
          {validMarkers.map((m, idx) => {
            const rawStart = Number.isFinite(m.start) ? m.start : m.value;
            const rawEnd = Number.isFinite(m.end) ? m.end : m.value;
            const sClamped = Math.max(min, Math.min(max, Math.min(rawStart, rawEnd)));
            const eClamped = Math.max(min, Math.min(max, Math.max(rawStart, rawEnd)));
            const startPct = ((sClamped - min) / span) * 100;
            const endPct = ((eClamped - min) / span) * 100;
            const centerPct = (startPct + endPct) / 2;
            const widthPct = Math.max(0, endPct - startPct);
            const isInterval = widthPct >= 1.5;
            return (
              <span
                key={`${m.kind || 's'}-${idx}`}
                className={`ui-slider-singularity-dot${m.kind === 'boundary' ? ' is-boundary' : ''}${isInterval ? ' is-interval' : ''}`}
                data-testid="slider-singularity-dot"
                style={{ left: `${centerPct}%`, ...(widthPct > 0 ? { width: `${widthPct}%` } : {}) }}
                title={m.label || `Singularidade em ${m.value}`}
              />
            );
          })}
        </div>
      )}
    </div>
  );
}
