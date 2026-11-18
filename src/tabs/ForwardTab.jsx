import { memo } from 'react';
import { MatrixView, Notice } from '../ui';
import { matrixRows } from '../kinematics';
import { fmt, vectorText } from '../format';

const ForwardTab = memo(function ForwardTab({ joints, fk }) {
  return (
    <>
      <section>
        <h2>Coordenadas globais (u)</h2>
        <MatrixView label="Origens dos referenciais" headers={['i', 'X', 'Y', 'Z']} values={fk.positions.map((p, i) => [i, ...p])} />
      </section>
      <section>
        <h2>Transformações homogêneas</h2>
        <div className="formula">Aᵢ = Rz(θᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)</div>
        <MatrixView label="T⁰ₙ = A₁ · … · Aₙ" values={matrixRows(fk.total)} />
        {fk.links.map((matrix, i) => (
          <details key={joints[i].id}>
            <summary>A{i + 1} · referencial {i} → {i + 1}</summary>
            <MatrixView label={`A${i + 1}`} values={matrixRows(matrix)} />
            <MatrixView label={`T⁰${i + 1}`} values={matrixRows(fk.transforms[i + 1])} />
          </details>
        ))}
      </section>
      <section>
        <h2>Orientação</h2>
        <MatrixView label="R⁰ₙ" values={fk.orientation.rotation} />
        <dl className="readings">
          <dt>RPY ZYX (°)</dt><dd>{vectorText(fk.orientation.rpy)}</dd>
          <dt>Composição</dt><dd>Rz(yaw) Ry(pitch) Rx(roll)</dd>
          <dt>Eixo de rotação</dt><dd>{fk.orientation.axis ? vectorText(fk.orientation.axis) : 'Indeterminado (rotação nula)'}</dd>
          <dt>Ângulo (°)</dt><dd>{fmt(fk.orientation.angle)}</dd>
          <dt>det(R)</dt><dd>{fmt(fk.orientation.determinant, 6)}</dd>
          <dt>‖RᵀR − I‖</dt><dd>{fk.orientation.orthogonality.toExponential(2)}</dd>
        </dl>
        {fk.orientation.gimbalLock && <Notice as="p">Singularidade RPY: roll e yaw não são únicos.</Notice>}
      </section>
    </>
  );
});

export default ForwardTab;
