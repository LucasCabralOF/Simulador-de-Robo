import { Select } from '../ui';

export default function TaskSettingsFields({ mode, setMode, length, setLength, damping, setDamping, onChangeSetting, numeric }) {
  return (
    <div className="task-settings">
      <label>Tarefa<Select
        ariaLabel="Tarefa"
        value={mode}
        onChange={val => onChangeSetting(setMode, val)}
        options={[
          { value: 'xy', label: 'Posição XY' },
          { value: 'position', label: 'Posição XYZ' },
          { value: 'pose', label: 'Pose XYZ + orientação' },
        ]}
      /></label>
      {numeric('Comprimento de referência (u)', length, v => onChangeSetting(setLength, v), { min: 0.001, max: 1e4 })}
      {numeric('Amortecimento λ', damping, v => onChangeSetting(setDamping, v), { min: 0, max: 10 })}
    </div>
  );
}
