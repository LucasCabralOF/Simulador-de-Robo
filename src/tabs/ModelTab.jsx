import { memo, useState } from 'react';
import { Plus, RotateCcw, Trash2 } from 'lucide-react';
import { Button, Select, Slider } from '../ui';
import { normalizeJoints } from '../kinematics';
import { syncWaypointsToJoints } from '../trajectory';
import { fmt } from '../format';
import CollapsibleSection from './CollapsibleSection';
import EnvironmentSection from './EnvironmentSection';

const ModelTab = memo(function ModelTab({
  joints,
  resetDefaultRobot,
  updateJoint,
  commitJoints,
  setIsPlaying,
  setWaypoints,
  environment,
  applyEnvironment,
  placeObstacleInFront,
  showObstacleGizmo,
  setShowObstacleGizmo,
  numeric,
}) {
  const [sectionOpen, setSectionOpen] = useState({
    env: true,
    dh: true,
  });

  const allAdvancedClosed = !sectionOpen.env && !sectionOpen.dh;
  const toggleAdvanced = () => {
    const next = allAdvancedClosed;
    setSectionOpen({ env: next, dh: next });
  };

  return (
    <>
      <section>
        <div className="section-title">
          <h2>Atuadores</h2>
          <div className="section-header-actions">
            <Button onClick={toggleAdvanced}>
              {allAdvancedClosed ? 'Expandir avançados' : 'Recolher avançados'}
            </Button>
            <Button icon title="Restaurar Cobot UR5 padrão" aria-label="Restaurar Cobot UR5 padrão" onClick={resetDefaultRobot}>
              <RotateCcw size={17} />
            </Button>
          </div>
        </div>
        {joints.map((joint, index) => {
          const unit = joint.type === 'R' ? '°' : 'u';
          return (
            <div className="actuator" key={joint.id}>
              <div className="control-header">
                <div className="joint-label-group">
                  <strong>Junta {index + 1} · {joint.type}</strong>
                  <span className={`joint-type-badge ${joint.type === 'P' ? 'is-prismatic' : 'is-rotational'}`}>
                    {joint.type === 'P' ? 'Prismática' : 'Rotativa'} · Δ {fmt(joint.max - joint.min, joint.type === 'R' ? 0 : 1)} {unit}
                  </span>
                </div>
                {numeric(`q${index + 1} (${unit})`, joint.val, val => updateJoint(joint.id, { val }), { min: joint.min, max: joint.max })}
              </div>
              <Slider
                ariaLabel={`Atuador ${index + 1}`}
                ariaValueText={`${fmt(joint.val, joint.type === 'R' ? 1 : 2)} ${unit}`}
                min={joint.min}
                max={joint.max}
                value={joint.val}
                onChange={val => updateJoint(joint.id, { val }, { continuous: true })}
              />
              <div className="limit-fields">
                {numeric(`Mín. ${index + 1} (${unit})`, joint.min, min => updateJoint(joint.id, { min }))}
                {numeric(`Máx. ${index + 1} (${unit})`, joint.max, max => updateJoint(joint.id, { max }))}
              </div>
            </div>
          );
        })}
      </section>

      <EnvironmentSection
        environment={environment}
        applyEnvironment={applyEnvironment}
        placeObstacleInFront={placeObstacleInFront}
        showObstacleGizmo={showObstacleGizmo}
        setShowObstacleGizmo={setShowObstacleGizmo}
        numeric={numeric}
        open={sectionOpen.env}
        onToggle={open => setSectionOpen(prev => ({ ...prev, env: open }))}
      />

      <CollapsibleSection
        title="Denavit-Hartenberg padrão"
        open={sectionOpen.dh}
        onToggle={open => setSectionOpen(prev => ({ ...prev, dh: open }))}
        actions={
          <Button icon title="Adicionar junta" aria-label="Adicionar junta" disabled={joints.length >= 24} onClick={() => {
            const nextJoints = [...joints, { id: Math.max(...joints.map(j => j.id)) + 1, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 0, min: -180, max: 180, speed: 0 }];
            if (commitJoints(nextJoints)) {
              setIsPlaying(false);
              setWaypoints(prev => syncWaypointsToJoints(prev, normalizeJoints(nextJoints)));
            }
          }}>
            <Plus size={18} />
          </Button>
        }
      >
        <div className="table-scroll">
          <table className="dh-table">
            <thead><tr><th>i</th><th>Tipo</th><th>a (u)</th><th>α (°)</th><th>θ (°)</th><th>d (u)</th><th></th></tr></thead>
            <tbody>
              {joints.map((j, i) => (
                <tr key={j.id}>
                  <td>{i + 1}</td>
                  <td>
                    <Select
                      compact
                      ariaLabel={`Tipo ${i + 1}`}
                      value={j.type}
                      options={[{ value: 'R', label: 'R' }, { value: 'P', label: 'P' }]}
                      onChange={nextType => {
                        const nextJoints = joints.map(item => (item.id === j.id ? { ...item, type: nextType, val: 0, speed: 0, min: nextType === 'R' ? -180 : 0, max: nextType === 'R' ? 180 : 10 } : item));
                        if (commitJoints(nextJoints)) {
                          setIsPlaying(false);
                          setWaypoints(prev => syncWaypointsToJoints(prev, normalizeJoints(nextJoints)));
                        }
                      }}
                    />
                  </td>
                  {['a', 'alpha', 'theta', 'd'].map(key => (
                    <td key={key}>
                      {numeric(`${key}${i + 1}`, (key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? j.val : j[key], v => updateJoint(j.id, { [(key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? 'val' : key]: v }), { compact: true })}
                    </td>
                  ))}
                  <td>
                    <Button icon variant="danger" title={`Remover junta ${i + 1}`} aria-label={`Remover junta ${i + 1}`} disabled={joints.length === 1} onClick={() => {
                      const nextJoints = joints.filter(item => item.id !== j.id);
                      if (commitJoints(nextJoints)) {
                        setIsPlaying(false);
                        const normalized = normalizeJoints(nextJoints);
                        setWaypoints(prev => syncWaypointsToJoints(
                          prev.map(wp => ({ ...wp, q: Array.isArray(wp.q) ? wp.q.filter((_, idx) => idx !== i) : [] })),
                          normalized
                        ));
                      }
                    }}>
                      <Trash2 size={15} />
                    </Button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </CollapsibleSection>
    </>
  );
});

export default ModelTab;
