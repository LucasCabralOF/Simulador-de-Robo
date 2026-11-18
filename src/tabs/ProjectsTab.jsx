import { memo } from 'react';
import { CheckCheck, Copy, Download, RotateCcw, Save, Share2, Trash2, Upload } from 'lucide-react';
import { Button, FileUploadButton, Notice, Select } from '../ui';
import { normalizeJoints, normalizeModelStyle } from '../kinematics';
import { PRESET_LIST } from '../presets';
import { defaultWaypointsForJoints, syncWaypointsToJoints } from '../trajectory';

const ProjectsTab = memo(function ProjectsTab({
  joints,
  applyPreset,
  handleUrdfUpload,
  toolType,
  toolOffsetRpy,
  toolGrip,
  toolScale,
  modelStyle,
  resetDefaultRobot,
  commitJoints,
  setIsPlaying,
  setPlayTime,
  setWaypoints,
  setViewKey,
  simName,
  setSimName,
  busy,
  persist,
  request,
  waypoints,
  refresh,
  setMessage,
  handleShare,
  shareLoading,
  shareUrl,
  copyShareUrl,
  shareCopied,
  apiError,
  simulations,
  setToolTypeLoaded,
  setToolOffsetRpyLoaded,
  setToolGripLoaded,
  setToolScaleLoaded,
  setModelStyleLoaded,
  syncTargetAndModeForJoints,
  exportProjectJson,
  handleProjectJsonUpload,
}) {
  return (
    <>
      <section>
        <div className="section-title">
          <h2>Presets industriais</h2>
          <Button icon title="Restaurar Cobot UR5 padrão" aria-label="Restaurar Cobot UR5 padrão" onClick={resetDefaultRobot}>
            <RotateCcw size={17} />
          </Button>
        </div>
        <div className="preset-row">
          <Select
            ariaLabel="Selecionar preset industrial"
            value=""
            placeholder="Carregar arquitetura consagrada..."
            onChange={val => { if (val) applyPreset(val); }}
            options={PRESET_LIST.map(p => ({ value: p.id, label: `${p.name} (${p.dof} DOF)` }))}
          />
        </div>
        <div className="model-actions">
          <FileUploadButton accept=".urdf,.xml" onChange={handleUrdfUpload} inputAriaLabel="Upload de arquivo URDF">
            <Upload size={14} /> Importar robô (URDF)
          </FileUploadButton>
          <Button aria-label="Exportar projeto JSON" onClick={exportProjectJson}>
            <Download size={14} /> Exportar projeto (.json)
          </Button>
          <FileUploadButton accept=".json,application/json" onChange={handleProjectJsonUpload} inputAriaLabel="Upload de projeto JSON">
            <Upload size={14} /> Importar projeto (.json)
          </FileUploadButton>
        </div>
      </section>

      <section>
        <div className="section-title">
          <h2>Simulações</h2>
        </div>
        <form
          className="save-row"
          onSubmit={e => {
            e.preventDefault();
            void persist(async () => {
              await request('', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                  name: simName.trim(),
                  data: joints,
                  waypoints,
                  toolConfig: { toolType, toolOffsetRpy, toolGrip, toolScale, modelStyle },
                }),
              });
              setSimName('');
              await refresh();
              setMessage('Simulação salva.');
            });
          }}
        >
          <input
            className="ui-input"
            aria-label="Nome da simulação"
            placeholder="Nome da simulação"
            value={simName}
            maxLength={120}
            onChange={e => setSimName(e.target.value)}
            required
          />
          <Button type="submit" variant="primary" disabled={busy || !simName.trim()}>
            <Save size={16} />Salvar
          </Button>
        </form>

        <div className="action-row">
          <Button disabled={busy || shareLoading} onClick={handleShare}>
            <Share2 size={16} /> Compartilhar link público
          </Button>
        </div>

        {shareUrl && (
          <div className="share-box">
            <strong>Link público de compartilhamento:</strong>
            <div className="share-url-row">
              <input className="ui-input" readOnly value={shareUrl} aria-label="Link de compartilhamento" />
              <Button onClick={copyShareUrl}>
                {shareCopied ? <CheckCheck size={14} /> : <Copy size={14} />}
                {shareCopied ? 'Copiado' : 'Copiar'}
              </Button>
            </div>
          </div>
        )}

        {apiError && (
          <Notice variant="error" role="alert">
            {apiError}
            <Button onClick={() => void refresh()}>Reconectar</Button>
          </Notice>
        )}
        {!apiError && simulations.length === 0 && <p className="muted">Nenhuma simulação salva.</p>}

        <ul className="saved-list">
          {simulations.map(sim => (
            <li key={sim.id}>
              <span>{sim.name}</span>
              <Button
                icon
                disabled={busy}
                title={`Carregar ${sim.name}`}
                aria-label={`Carregar ${sim.name}`}
                onClick={() => void persist(async () => {
                  const data = await request(`/${sim.id}`);
                  const normalized = normalizeJoints(data.data);
                  const loadedTool = data.toolConfig || null;
                  if (loadedTool) {
                    setToolTypeLoaded(loadedTool.toolType || 'gripper');
                    setToolOffsetRpyLoaded(loadedTool.toolOffsetRpy || [0, 0, 0]);
                    setToolGripLoaded(loadedTool.toolGrip ?? 50);
                    setToolScaleLoaded?.(Number.isFinite(Number(loadedTool.toolScale)) ? Math.max(0.4, Math.min(4.0, Number(loadedTool.toolScale))) : 1);
                    setModelStyleLoaded(normalizeModelStyle(loadedTool.modelStyle));
                  }
                  commitJoints(normalized);
                  syncTargetAndModeForJoints(normalized, null, loadedTool);
                  setIsPlaying(false);
                  setPlayTime(0);
                  setWaypoints(
                    Array.isArray(data.waypoints) && data.waypoints.length > 0
                      ? syncWaypointsToJoints(data.waypoints, normalized)
                      : defaultWaypointsForJoints(normalized)
                  );
                  setViewKey(k => k + 1);
                  setMessage('Simulação carregada.');
                })}
              >
                <Download size={16} />
              </Button>
              <Button
                icon
                variant="danger"
                disabled={busy}
                title={`Apagar ${sim.name}`}
                aria-label={`Apagar ${sim.name}`}
                onClick={() => {
                  if (window.confirm(`Apagar "${sim.name}"?`)) {
                    void persist(async () => {
                      await request(`/${sim.id}`, { method: 'DELETE' });
                      await refresh();
                    });
                  }
                }}
              >
                <Trash2 size={16} />
              </Button>
            </li>
          ))}
        </ul>
      </section>
    </>
  );
});

export default ProjectsTab;
