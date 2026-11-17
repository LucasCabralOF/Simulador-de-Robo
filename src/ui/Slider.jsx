export function Slider({
  min = 0,
  max = 100,
  step = 'any',
  value,
  onChange,
  disabled = false,
  ariaLabel,
  ariaValueText,
  className = '',
}) {
  const span = max - min;
  const pct = span > 0 && Number.isFinite(value)
    ? Math.max(0, Math.min(100, ((value - min) / span) * 100))
    : 0;

  return (
    <input
      type="range"
      className={`ui-slider${className ? ` ${className}` : ''}`}
      aria-label={ariaLabel}
      aria-valuetext={ariaValueText}
      min={min}
      max={max}
      step={step}
      disabled={disabled}
      value={value}
      style={{ '--slider-progress': `${pct}%` }}
      onChange={e => onChange?.(Number(e.target.value), e)}
    />
  );
}
