import { memo } from 'react';
import { Button, Checkbox, Notice, Select } from '../ui';
import { fmt } from '../format';

const WorkspaceTab = memo(function WorkspaceTab({
  sampleCount,
  setSampleCount,
  computingCloud,
  computeWorkspace,
  workspaceCloud,
  showWorkspace,
  setShowWorkspace,
}) {
  return (
    <>
      <section>
        <h2>Espaço de Trabalho 3D (Monte Carlo)</h2>
        <p className="muted">Geração do envelope de alcance com cálculo de destreza e índice de manipulabilidade de Yoshikawa.</p>
        <div className="field-grid">
          <label>Amostras
            <Select
              ariaLabel="Número de amostras"
              value={sampleCount}
              onChange={val => setSampleCount(Number(val))}
              options={[
                { value: 1000, label: '1.000 pontos (Rápido)' },
                { value: 2500, label: '2.500 pontos (Equilibrado)' },
                { value: 5000, label: '5.000 pontos (Alta densidade)' },
              ]}
            />
          </label>
          <div className="field-action">
            <Button variant="primary" disabled={computingCloud} onClick={computeWorkspace}>
              {computingCloud ? 'Calculando...' : 'Calcular alcance'}
            </Button>
          </div>
        </div>
      </section>
      {workspaceCloud && (
        <section>
          <h2>Estatísticas do Envelope</h2>
          <dl className="readings">
            <dt>Pontos amostrados</dt><dd>{workspaceCloud.count}</dd>
            <dt>Envelope X (u)</dt><dd>[{fmt(workspaceCloud.bounds.minX, 2)}, {fmt(workspaceCloud.bounds.maxX, 2)}] (Δ {fmt(workspaceCloud.bounds.dx, 2)})</dd>
            <dt>Envelope Y (u)</dt><dd>[{fmt(workspaceCloud.bounds.minY, 2)}, {fmt(workspaceCloud.bounds.maxY, 2)}] (Δ {fmt(workspaceCloud.bounds.dy, 2)})</dd>
            <dt>Envelope Z (u)</dt><dd>[{fmt(workspaceCloud.bounds.minZ, 2)}, {fmt(workspaceCloud.bounds.maxZ, 2)}] (Δ {fmt(workspaceCloud.bounds.dz, 2)})</dd>
            <dt>Volume envolvente (u³)</dt><dd>{fmt(workspaceCloud.boundingVolume, 2)}</dd>
            <dt>Máx. manipulabilidade w</dt><dd>{fmt(workspaceCloud.maxManipulability, 4)}</dd>
          </dl>
          <div className="action-row">
            <Checkbox checked={showWorkspace} onChange={setShowWorkspace}>
              Exibir pontos na cena 3D
            </Checkbox>
          </div>
          <div className="workspace-legend-card">
            <div className="workspace-gradient-bar" aria-hidden="true" />
            <div className="workspace-gradient-labels">
              <span><strong style={{ color: '#4da6ff' }}>Azul</strong> · Singularidade (w → 0)</span>
              <span><strong style={{ color: '#00d6e6' }}>Ciano</strong> · Média</span>
              <span><strong style={{ color: '#24eb74' }}>Verde</strong> · Alta destreza</span>
            </div>
          </div>
          <Notice as="p">
            Regiões em <strong style={{ color: '#4da6ff' }}>azul</strong> indicam proximidade de singularidades (baixa manipulabilidade); regiões em <strong style={{ color: '#24eb74' }}>verde</strong> indicam maior destreza articular.
          </Notice>
        </section>
      )}
    </>
  );
});

export default WorkspaceTab;
