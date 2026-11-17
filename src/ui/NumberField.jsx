import { useId } from 'react';

export function NumberField({
  label,
  value,
  onCommit,
  min = -1e6,
  max = 1e6,
  onError,
  compact = false,
  className = '',
}) {
  const inputId = useId();

  return (
    <label htmlFor={inputId} className={`ui-field number-field${compact ? ' compact' : ''}${className ? ` ${className}` : ''}`}>
      {!compact && <span>{label}</span>}
      <input
        id={inputId}
        key={`${label}:${value}`}
        className={`ui-input ui-input-number${compact ? ' compact' : ''}`}
        aria-label={label}
        title={label}
        type="number"
        defaultValue={value}
        min={min}
        max={max}
        step="any"
        required
        onInput={e => { e.currentTarget.setCustomValidity(''); }}
        onKeyDown={e => { if (e.key === 'Enter') e.currentTarget.blur(); }}
        onBlur={e => {
          const input = e.currentTarget;
          if (!input.validity.valid || !Number.isFinite(input.valueAsNumber)) {
            onError?.(`${label}: informe um número entre ${min} e ${max}.`);
            input.value = String(value);
            return;
          }
          if (onCommit(input.valueAsNumber) === false) input.value = String(value);
        }}
      />
    </label>
  );
}
