import { Check } from 'lucide-react';

export function Checkbox({ checked, onChange, children, className = '' }) {
  return (
    <label className={`ui-checkbox${className ? ` ${className}` : ''}`}>
      <input
        type="checkbox"
        className="ui-checkbox-input"
        checked={checked}
        onChange={e => onChange?.(e.target.checked, e)}
      />
      <span className="ui-checkbox-box" aria-hidden="true">
        {checked && <Check size={12} strokeWidth={3} />}
      </span>
      <span className="ui-checkbox-text">{children}</span>
    </label>
  );
}
