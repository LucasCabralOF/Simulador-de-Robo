import { memo } from 'react';
import { RotateCcw } from 'lucide-react';
import { Button, Checkbox, SegmentedTabs } from '../ui';
import { DEFAULT_ENVIRONMENT } from '../kinematics';
import CollapsibleSection from './CollapsibleSection';

const EnvironmentSection = memo(function EnvironmentSection({
  environment,
  applyEnvironment,
  placeObstacleInFront,
  showObstacleGizmo,
  setShowObstacleGizmo,
  numeric,
  open,
  onToggle,
}) {
  const isDefaultEnv = environment.floorEnabled
    && Math.abs(environment.floorZ - DEFAULT_ENVIRONMENT.floorZ) < 1e-6
    && !environment.obstacle.enabled;

  return (
    <CollapsibleSection
      title="Ambiente e Colisão Externa (Piso / Obstáculo 3D)"
      open={open}
      onToggle={onToggle}
      actions={
        <Button
          icon
          title="Restaurar ambiente padrão"
          aria-label="Restaurar ambiente padrão"
          disabled={isDefaultEnv}
          onClick={() => applyEnvironment({
            ...DEFAULT_ENVIRONMENT,
            obstacle: { ...DEFAULT_ENVIRONMENT.obstacle, position: [...DEFAULT_ENVIRONMENT.obstacle.position] },
          })}
        >
          <RotateCcw size={16} />
        </Button>
      }
    >
      <div className="env-card">
        <div className="env-row">
          <Checkbox
            checked={environment.floorEnabled}
            onChange={floorEnabled => applyEnvironment({ ...environment, floorEnabled })}
          >
            Piso / Bancada ativo
          </Checkbox>
          {numeric('Nível do piso Z (u)', environment.floorZ, z => applyEnvironment({ ...environment, floorZ: z }), { min: -20, max: 20 })}
        </div>
        <div className="env-obstacle-header">
          <Checkbox
            checked={environment.obstacle.enabled}
            onChange={enabled => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, enabled } })}
          >
            Obstáculo 3D ativo
          </Checkbox>
          <Button onClick={placeObstacleInFront}>
            Posicionar à frente do efetuador
          </Button>
        </div>
        <Checkbox
          checked={showObstacleGizmo}
          onChange={setShowObstacleGizmo}
        >
          Mover obstáculo no 3D (Gizmo)
        </Checkbox>
        <SegmentedTabs
          ariaLabel="Forma do obstáculo 3D"
          activeId={environment.obstacle.shape}
          onChange={shape => applyEnvironment({ ...environment, obstacle: { ...environment.obstacle, shape } })}
          items={[
            { id: 'sphere', label: 'Esfera 3D' },
            { id: 'box', label: 'Caixa 3D (Cubo)' },
          ]}
        />
        <div className="env-obstacle-grid">
          {['X', 'Y', 'Z'].map((axis, idx) => (
            <div key={axis}>
              {numeric(`Obstáculo ${axis} (u)`, environment.obstacle.position[idx], val => applyEnvironment({
                ...environment,
                obstacle: {
                  ...environment.obstacle,
                  position: environment.obstacle.position.map((p, i) => (i === idx ? val : p)),
                },
              }), { min: -30, max: 30 })}
            </div>
          ))}
          <div>
            {numeric('Raio / Tamanho (u)', environment.obstacle.radius, radius => applyEnvironment({
              ...environment,
              obstacle: { ...environment.obstacle, radius },
            }), { min: 0.1, max: 10 })}
          </div>
        </div>
      </div>
    </CollapsibleSection>
  );
});

export default EnvironmentSection;
