import { memo } from 'react';
import { Activity, Check } from 'lucide-react';
import { Button, MatrixView, Notice } from '../ui';
import { TASK_ROWS } from '../kinematics';
import { fmt, vectorText } from '../format';
import TaskSettingsFields from './TaskSettingsFields';

const velocityLabels = ['vx (u/s)', 'vy (u/s)', 'vz (u/s)', 'ωx (rad/s)', 'ωy (rad/s)', 'ωz (rad/s)'];
const wrenchLabels = ['Fx (N)', 'Fy (N)', 'Fz (N)', 'Mx (N·u)', 'My (N·u)', 'Mz (N·u)'];

const DifferentialTab = memo(function DifferentialTab({
  joints,
  fk,
  mode,
  setMode,
  length,
  setLength,
  damping,
  setDamping,
  onChangeSetting,
  diagnostics,
  velocity,
  differential,
  statics,
  dt,
  setDt,
  step,
  desiredVelocity,
  setDesiredVelocity,
  payloadMass,
  setPayloadMass,
  gravity,
  setGravity,
  wrench,
  setWrench,
  updateJoint,
  commitJoints,
  setMessage,
  numeric,
}) {
  return (
    <>
      <section>
        <h2>Jacobiano geométrico</h2>
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
        <MatrixView label="J · linhas [vx, vy, vz, ωx, ωy, ωz]" headers={joints.map((_, i) => `q̇${i + 1}`)} values={fk.jacobian} />
        <dl className="readings">
          <dt>Posto / máximo algébrico</dt><dd>{diagnostics.rank} / {diagnostics.maxRank}</dd>
          <dt>Dimensões da tarefa</dt><dd>{diagnostics.dimensions}</dd>
          <dt>Condição normalizada</dt><dd>{fmt(diagnostics.condition)}</dd>
          <dt>Valores singulares normalizados</dt><dd>{vectorText(diagnostics.singularValues)}</dd>
          <dt>det(J da tarefa)</dt><dd>{diagnostics.determinant === null ? 'Não se aplica (retangular)' : fmt(diagnostics.determinant, 6)}</dd>
        </dl>
        <Notice as="p" variant={diagnostics.fullTask ? 'success' : 'warning'}>
          {diagnostics.fullTask ? 'Tarefa localmente controlável.' : diagnostics.deficient ? 'Posto reduzido: singularidade ou restrição estrutural da tarefa.' : 'Graus de liberdade insuficientes para controlar toda a tarefa.'}
          {diagnostics.condition > 1000 && Number.isFinite(diagnostics.condition) ? ' Próximo de perda de posto.' : ''}
        </Notice>
      </section>
      <section>
        <h2>Velocidades articulares</h2>
        <div className="field-grid">
          {joints.map((j, i) => (
            <div key={j.id}>{numeric(`q̇${i + 1} (${j.type === 'R' ? 'rad/s' : 'u/s'})`, j.speed, speed => updateJoint(j.id, { speed }))}</div>
          ))}
        </div>
        <MatrixView label="v = J q̇ · base global" values={velocity.map(v => [v])} />
        <div className="action-row">
          {numeric('Δt (s)', dt, setDt, { min: 0.001, max: 10 })}
          <Button onClick={step}><Activity size={16} />Aplicar passo</Button>
        </div>
      </section>
      <section>
        <h2>Inversa diferencial</h2>
        <div className="field-grid">
          {TASK_ROWS[mode].map(i => (
            <div key={i}>{numeric(velocityLabels[i], desiredVelocity[i], value => setDesiredVelocity(prev => prev.map((v, k) => (k === i ? value : v))))}</div>
          ))}
        </div>
        <MatrixView label="q̇ calculado · R: rad/s; P: u/s" values={differential.rates.map(v => [v])} />
        <MatrixView label="Velocidade obtida [u/s; rad/s]" values={differential.achieved.map(v => [v])} />
        <div className="metric">Resíduo normalizado: <output>{fmt(differential.error, 6)}</output></div>
        <Button onClick={() => { if (commitJoints(joints.map((j, i) => ({ ...j, speed: differential.rates[i] })))) setMessage('Velocidades articulares atualizadas.'); }}>
          <Check size={16} />Usar velocidades
        </Button>
      </section>
      <section>
        <h2>Estática (τ = Jᵀ F)</h2>
        <p className="muted">Calcula os torques (Tᵢ para juntas R) e forças (Fᵢ para juntas P) resultantes nas juntas para equilibrar o vetor força/momento no efetuador.</p>
        <div className="field-grid">
          {numeric('Massa da carga (kg)', payloadMass, setPayloadMass, { min: 0, max: 1e4 })}
          {numeric('Gravidade g (m/s²)', gravity, setGravity, { min: 0, max: 100 })}
        </div>
        <div className="action-row">
          <Button onClick={() => setWrench([0, -(payloadMass * gravity), 0, 0, 0, 0])}>Aplicar −m·g em Y</Button>
          <Button onClick={() => setWrench([0, 0, -(payloadMass * gravity), 0, 0, 0])}>Aplicar −m·g em Z</Button>
          <Button onClick={() => setWrench([0, 0, 0, 0, 0, 0])}>Zerar F⃗</Button>
        </div>
        <div className="field-grid">
          {wrenchLabels.map((lbl, i) => (
            <div key={lbl}>{numeric(lbl, wrench[i], value => setWrench(prev => prev.map((v, k) => (k === i ? value : v))))}</div>
          ))}
        </div>
        <MatrixView
          label="Esforços nas juntas · R: Torque T (N·u); P: Força F (N)"
          headers={statics.labels.map((lbl, i) => `${lbl} (${statics.units[i]})`)}
          values={[statics.efforts]}
        />
      </section>
    </>
  );
});

export default DifferentialTab;
