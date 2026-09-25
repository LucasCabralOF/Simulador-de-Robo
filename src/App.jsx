import { useEffect, useMemo, useState } from 'react';
import { Activity, Check, CheckCheck, Copy, Crosshair, Download, Focus, Pause, Play, Plus, Printer, Repeat, RotateCcw, Save, Settings2, Share2, Square, Trash2, Upload } from 'lucide-react';
import RobotScene from './RobotScene';
import { DEFAULT_JOINTS, TASK_ROWS, forwardKinematics, integrateJoints, inverseKinematics, inverseVelocity, jacobianDiagnostics, matrixRows, multiplyVector, normalizeJoints, validateJoints } from './kinematics';
import { PRESETS, PRESET_LIST } from './presets';
import { generateWorkspaceCloud } from './workspace';
import { generateCpp, generatePython, generateRos2 } from './codeGenerator';
import { evaluateTrajectory, sampleTrajectoryPath } from './trajectory';
import { openAcademicReport } from './reportGenerator';
import { exportToURDF, parseURDF } from './urdfParser';
import { parseStlBuffer } from './cadLoader';
import './App.css';

const API = (import.meta.env.VITE_API_URL || '/api').replace(/\/$/, '');
const fmt = (n, digits = 4) => !Number.isFinite(n) ? '∞' : (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits);
const vectorText = (v) => v.map(n => fmt(n)).join(', ');
const velocityLabels = ['vx (u/s)', 'vy (u/s)', 'vz (u/s)', 'ωx (rad/s)', 'ωy (rad/s)', 'ωz (rad/s)'];

async function request(path = '', options = {}) {
  const res = await fetch(`${API}/simulations${path}`, options);
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || `Falha na API (${res.status}).`);
  return data;
}

function NumberField({ label, value, onCommit, min = -1e6, max = 1e6, onError, compact = false }) {
  return <label className={compact ? 'number-field compact' : 'number-field'}>
    {!compact && <span>{label}</span>}
    <input key={`${label}:${value}`} aria-label={label} title={label} type="number" defaultValue={value}
      min={min} max={max} step="any" required onKeyDown={e => { if (e.key === 'Enter') e.currentTarget.blur(); }}
      onBlur={e => {
        const input = e.currentTarget;
        if (!input.validity.valid || !Number.isFinite(input.valueAsNumber)) {
          onError?.(`${label}: informe um número entre ${min} e ${max}.`);
          input.value = String(value);
          return;
        }
        if (onCommit(input.valueAsNumber) === false) input.value = String(value);
      }} />
  </label>;
}

function MatrixView({ label, values, headers }) {
  return <figure className="matrix-view">
    <figcaption>{label}</figcaption>
    <div className="table-scroll"><table aria-label={label}>
      {headers && <thead><tr>{headers.map(h => <th key={h}>{h}</th>)}</tr></thead>}
      <tbody>{values.map((row, i) => <tr key={i}>{row.map((v, j) => <td key={j}>{fmt(v)}</td>)}</tr>)}</tbody>
    </table></div>
  </figure>;
}

function App() {
  const [joints, setJoints] = useState(() => DEFAULT_JOINTS.map(j => ({ ...j })));
  const [tab, setTab] = useState('model');
  const [simulations, setSimulations] = useState([]);
  const [simName, setSimName] = useState('');
  const [busy, setBusy] = useState(false);
  const [apiError, setApiError] = useState('');
  const [message, setMessage] = useState('');
  const [mode, setMode] = useState('xy');
  const [length, setLength] = useState(5);
  const [damping, setDamping] = useState(0.02);
  const [dt, setDt] = useState(0.1);
  const [desiredVelocity, setDesiredVelocity] = useState([0, 0, 0, 0, 0, 0]);
  const [target, setTarget] = useState({ position: [5, 0, 0], rpy: [0, 0, 0] });
  const [solution, setSolution] = useState(null);
  const [showFrames, setShowFrames] = useState(true);
  const [viewKey, setViewKey] = useState(0);
  const [workspaceCloud, setWorkspaceCloud] = useState(null);
  const [showWorkspace, setShowWorkspace] = useState(true);
  const [sampleCount, setSampleCount] = useState(2500);
  const [computingCloud, setComputingCloud] = useState(false);
  const [codeTarget, setCodeTarget] = useState('cpp');
  const [copied, setCopied] = useState(false);
  const [waypoints, setWaypoints] = useState(() => [
    { id: 1, name: 'Home', q: DEFAULT_JOINTS.map(j => j.val), duration: 0 },
    { id: 2, name: 'Ponto A', q: DEFAULT_JOINTS.map((j, i) => i === 0 ? j.val + 35 : i === 1 ? j.val + 2 : j.val - 25), duration: 2.0 },
    { id: 3, name: 'Ponto B', q: DEFAULT_JOINTS.map((j, i) => i === 0 ? j.val - 35 : i === 1 ? j.val - 2 : j.val + 25), duration: 2.5 },
  ]);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playTime, setPlayTime] = useState(0);
  const [playbackSpeed, setPlaybackSpeed] = useState(1.0);
  const [isLooping, setIsLooping] = useState(false);
  const [interpType, setInterpType] = useState('quintic');
  const [showTrajectory, setShowTrajectory] = useState(true);
  const [shareUrl, setShareUrl] = useState('');
  const [shareLoading, setShareLoading] = useState(false);
  const [shareCopied, setShareCopied] = useState(false);
  const [customStlGeometry, setCustomStlGeometry] = useState(null);
  const [stlFileName, setStlFileName] = useState('');
  const [reportMeta, setReportMeta] = useState({
    studentName: 'Lucas Engenharia',
    courseName: 'Robótica Industrial e Cinemática',
    institution: 'Universidade / Faculdade de Engenharia',
    robotName: '',
  });
  const fk = useMemo(() => forwardKinematics(joints), [joints]);
  const diagnostics = useMemo(() => jacobianDiagnostics(joints, fk.jacobian, mode, length), [joints, fk, mode, length]);
  const velocity = multiplyVector(fk.jacobian, joints.map(j => j.speed));
  const differential = inverseVelocity(joints, fk.jacobian, desiredVelocity, { mode, length, damping });

  async function refresh(signal) {
    try { setSimulations(await request('', { signal })); setApiError(''); }
    catch (error) { if (error.name !== 'AbortError') setApiError('API indisponível. ' + error.message); }
  }
  useEffect(() => {
    const controller = new AbortController();
    request('', { signal: controller.signal }).then(data => {
      setSimulations(data);
      setApiError('');
    }).catch(error => {
      if (error.name !== 'AbortError') setApiError('API indisponível. ' + error.message);
    });

    const params = new URLSearchParams(window.location.search);
    const slug = params.get('share');
    if (slug) {
      fetch(`${API}/share/${slug}`, { signal: controller.signal })
        .then(res => res.json())
        .then(resData => {
          if (resData.data) {
            commitJoints(resData.data);
            if (resData.waypoints) setWaypoints(resData.waypoints);
            setViewKey(k => k + 1);
            setMessage(`Modelo compartilhado "${resData.name}" carregado.`);
          }
        })
        .catch(() => {});
    }

    return () => controller.abort();
  }, []);

  function commitJoints(next) {
    const errors = validateJoints(next);
    if (errors.length) { setMessage(errors.join(' ')); return false; }
    setJoints(normalizeJoints(next)); setSolution(null); setMessage('');
    return true;
  }
  function updateJoint(id, updates) { return commitJoints(joints.map(j => j.id === id ? { ...j, ...updates } : j)); }
  function changeSetting(setter, value) { setter(value); setSolution(null); }
  function changeTarget(key, index, value) {
    setTarget(prev => ({ ...prev, [key]: prev[key].map((v, i) => i === index ? value : v) }));
    setSolution(null);
  }
  async function persist(action) {
    setBusy(true); setApiError('');
    try { await action(); }
    catch (error) { setApiError(error.message); }
    finally { setBusy(false); }
  }
  function step() {
    const result = integrateJoints(joints, joints.map(j => j.speed), dt);
    commitJoints(result.joints);
    setMessage(result.limited.length ? `Limite atingido nas juntas ${result.limited.join(', ')}.` : `Passo de ${dt} s aplicado.`);
  }

  function applyPreset(presetId) {
    const p = PRESETS[presetId];
    if (!p) return;
    commitJoints(p.joints);
    setViewKey(k => k + 1);
    setWorkspaceCloud(null);
    setIsPlaying(false);
    setPlayTime(0);
    setWaypoints([
      { id: 1, name: 'Home', q: p.joints.map(j => j.val), duration: 0 },
      { id: 2, name: 'Ponto A', q: p.joints.map((j, i) => i === 0 ? j.val + (j.type === 'R' ? 30 : 1) : j.val), duration: 2.0 },
      { id: 3, name: 'Ponto B', q: p.joints.map((j, i) => i === 1 ? j.val + (j.type === 'R' ? -25 : -1) : j.val), duration: 2.0 },
    ]);
    setMessage(`Robô ${p.name} carregado.`);
  }

  const trajectoryData = useMemo(() => {
    return sampleTrajectoryPath(waypoints, joints, 80);
  }, [waypoints, joints]);

  function seekTrajectory(time) {
    setPlayTime(time);
    if (waypoints.length < 2) return;
    const evaluated = evaluateTrajectory(waypoints, time, interpType);
    if (!evaluated) return;
    setJoints(prev => prev.map((j, i) => ({
      ...j,
      val: evaluated.q[i] !== undefined ? evaluated.q[i] : j.val,
      speed: evaluated.velocities[i] !== undefined ? evaluated.velocities[i] : 0,
    })));
  }

  useEffect(() => {
    if (!isPlaying) return;
    let lastTime = performance.now();
    let animId;

    const tick = (now) => {
      const deltaSec = (now - lastTime) / 1000;
      lastTime = now;

      setPlayTime(prev => {
        const total = trajectoryData.totalDuration;
        if (total <= 0) {
          setIsPlaying(false);
          return 0;
        }
        let next = prev + deltaSec * playbackSpeed;
        if (next >= total) {
          if (isLooping) {
            next = next % total;
          } else {
            setIsPlaying(false);
            next = total;
          }
        }
        const evaluated = evaluateTrajectory(waypoints, next, interpType);
        if (evaluated) {
          setJoints(pj => pj.map((j, i) => ({
            ...j,
            val: evaluated.q[i] !== undefined ? evaluated.q[i] : j.val,
            speed: evaluated.velocities[i] !== undefined ? evaluated.velocities[i] : 0,
          })));
        }
        return next;
      });

      animId = requestAnimationFrame(tick);
    };

    animId = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animId);
  }, [isPlaying, playbackSpeed, isLooping, trajectoryData.totalDuration, waypoints, interpType]);

  function addCurrentWaypoint() {
    const count = waypoints.length;
    const newWp = {
      id: Date.now(),
      name: `Ponto ${count + 1}`,
      q: joints.map(j => j.val),
      duration: count === 0 ? 0 : 2.0,
    };
    setWaypoints(prev => [...prev, newWp]);
    setMessage(`Waypoint "${newWp.name}" adicionado.`);
  }

  function removeWaypoint(id) {
    if (waypoints.length <= 1) return;
    setWaypoints(prev => prev.filter(w => w.id !== id));
  }

  function moveToWaypoint(wp) {
    setIsPlaying(false);
    commitJoints(joints.map((j, i) => ({
      ...j,
      val: wp.q[i] !== undefined ? wp.q[i] : j.val,
    })));
    setMessage(`Manipulador posicionado em "${wp.name}".`);
  }

  function updateWaypointDuration(id, duration) {
    setWaypoints(prev => prev.map(w => w.id === id ? { ...w, duration: Math.max(0.1, duration) } : w));
  }

  function computeWorkspace() {
    setComputingCloud(true);
    setTimeout(() => {
      try {
        const cloud = generateWorkspaceCloud(joints, sampleCount);
        setWorkspaceCloud(cloud);
        setShowWorkspace(true);
        setMessage(`Nuvem de trabalho calculada com ${cloud.count} amostras.`);
      } catch (err) {
        setMessage('Falha ao calcular espaço de trabalho: ' + err.message);
      } finally {
        setComputingCloud(false);
      }
    }, 20);
  }

  const generatedCode = useMemo(() => {
    const robotName = `Manipulador_${joints.map(j => j.type).join('')}_${joints.length}DOF`;
    if (codeTarget === 'cpp') return generateCpp(joints, robotName);
    if (codeTarget === 'python') return generatePython(joints, robotName);
    if (codeTarget === 'urdf') return exportToURDF(joints, robotName);
    return generateRos2(joints, robotName);
  }, [joints, codeTarget]);

  function downloadUrdf() {
    const urdfContent = exportToURDF(joints, `Manipulador_${joints.map(j => j.type).join('')}`);
    const blob = new Blob([urdfContent], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `robot_${joints.map(j => j.type).join('')}.urdf`;
    a.click();
    URL.revokeObjectURL(url);
    setMessage('Arquivo .urdf baixado com sucesso.');
  }

  function handleUrdfUpload(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const parsed = parseURDF(ev.target.result);
        if (commitJoints(parsed)) {
          setViewKey(k => k + 1);
          setMessage(`Robô importado via URDF com sucesso (${parsed.length} juntas).`);
        }
      } catch (err) {
        setMessage('Erro ao importar URDF: ' + err.message);
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  }

  function handleStlUpload(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const geom = parseStlBuffer(ev.target.result);
        setCustomStlGeometry(geom);
        setStlFileName(file.name);
        setMessage(`Malha STL "${file.name}" carregada no efetuador.`);
      } catch (err) {
        setMessage('Erro ao processar arquivo STL: ' + err.message);
      }
    };
    reader.readAsArrayBuffer(file);
    e.target.value = '';
  }

  function removeStlMesh() {
    if (customStlGeometry) customStlGeometry.dispose();
    setCustomStlGeometry(null);
    setStlFileName('');
    setMessage('Malha STL removida. Efetuador padrão restaurado.');
  }

  function copyCode() {
    navigator.clipboard.writeText(generatedCode).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  }

  async function handleShare() {
    setShareLoading(true);
    try {
      const res = await request('/share', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: simName.trim() || `Robô ${joints.map(j => j.type).join('')}`,
          data: joints,
          waypoints,
        }),
      });
      const fullUrl = `${window.location.origin}${window.location.pathname}?share=${res.slug}`;
      setShareUrl(fullUrl);
      setMessage('Link público gerado com sucesso!');
      await refresh();
    } catch (err) {
      setApiError('Erro ao compartilhar: ' + err.message);
    } finally {
      setShareLoading(false);
    }
  }

  function copyShareUrl() {
    if (!shareUrl) return;
    navigator.clipboard.writeText(shareUrl).then(() => {
      setShareCopied(true);
      setTimeout(() => setShareCopied(false), 2000);
    });
  }

  const numeric = (label, value, onCommit, extra = {}) => <NumberField label={label} value={value} onCommit={onCommit} onError={setMessage} {...extra} />;
  const taskSettings = <div className="task-settings">
    <label>Tarefa<select aria-label="Tarefa" value={mode} onChange={e => changeSetting(setMode, e.target.value)}>
      <option value="xy">Posição XY</option><option value="position">Posição XYZ</option><option value="pose">Pose XYZ + orientação</option>
    </select></label>
    {numeric('Comprimento de referência (u)', length, v => changeSetting(setLength, v), { min: 0.001, max: 1e4 })}
    {numeric('Amortecimento λ', damping, v => changeSetting(setDamping, v), { min: 0, max: 10 })}
  </div>;

  return <main className="app-container">
    <aside className="sidebar">
      <header className="title-container"><Settings2 size={25} /><div><h1>Cinemática DH</h1><span className="muted">Manipulador {joints.map(j => j.type).join('')} · {joints.length} juntas</span></div></header>
      <nav className="tabs" aria-label="Análises">
        {[['model', 'Modelo'], ['forward', 'Direta'], ['differential', 'Diferencial'], ['inverse', 'Inversa'], ['trajectory', 'Trajetória'], ['workspace', 'Alcance 3D'], ['export', 'Código'], ['report', 'Relatório']].map(([id, label]) =>
          <button key={id} aria-current={tab === id ? 'page' : undefined} onClick={() => { setTab(id); setMessage(''); }}>{label}</button>)}
      </nav>
      {message && <div className="notice" role="status">{message}</div>}

      {tab === 'model' && <>
        <section>
          <div className="section-title"><h2>Presets industriais</h2></div>
          <div className="preset-row">
            <select aria-label="Selecionar preset industrial" defaultValue="" onChange={e => { if (e.target.value) applyPreset(e.target.value); e.target.value = ''; }}>
              <option value="" disabled>Carregar arquitetura consagrada...</option>
              {PRESET_LIST.map(p => <option key={p.id} value={p.id}>{p.name} ({p.dof} DOF)</option>)}
            </select>
          </div>
          <div style={{ marginTop: '10px' }}>
            <label className="secondary-btn" style={{ display: 'inline-flex', alignItems: 'center', gap: '6px', cursor: 'pointer', padding: '6px 12px', fontSize: '13px', borderRadius: '4px', background: '#242b31', border: '1px solid #3a454d' }}>
              <Upload size={14} /> Importar robô (URDF)
              <input type="file" accept=".urdf,.xml" style={{ display: 'none' }} onChange={handleUrdfUpload} aria-label="Upload de arquivo URDF" />
            </label>
          </div>
        </section>
        <section><div className="section-title"><h2>Atuadores</h2><button className="icon-btn" title="Restaurar RPR das aulas" aria-label="Restaurar RPR das aulas" onClick={() => { commitJoints(DEFAULT_JOINTS); setViewKey(k => k + 1); }}><RotateCcw size={17} /></button></div>
          {joints.map((joint, index) => <div className="actuator" key={joint.id}>
            <div className="control-header"><strong>Junta {index + 1} · {joint.type}</strong>{numeric(`q${index + 1} (${joint.type === 'R' ? '°' : 'u'})`, joint.val, val => updateJoint(joint.id, { val }), { min: joint.min, max: joint.max })}</div>
            <input aria-label={`Atuador ${index + 1}`} type="range" min={joint.min} max={joint.max} step="any" value={joint.val} onChange={e => updateJoint(joint.id, { val: Number(e.target.value) })} />
            <div className="limit-fields">
              {numeric(`Mín. ${index + 1} (${joint.type === 'R' ? '°' : 'u'})`, joint.min, min => updateJoint(joint.id, { min }))}
              {numeric(`Máx. ${index + 1} (${joint.type === 'R' ? '°' : 'u'})`, joint.max, max => updateJoint(joint.id, { max }))}
            </div>
          </div>)}
        </section>
        <section><div className="section-title"><h2>Denavit-Hartenberg padrão</h2><button className="icon-btn" title="Adicionar junta" aria-label="Adicionar junta" disabled={joints.length >= 24} onClick={() => commitJoints([...joints, { id: Math.max(...joints.map(j => j.id)) + 1, type: 'R', a: 0, alpha: 0, d: 0, theta: 0, val: 0, min: -180, max: 180, speed: 0 }])}><Plus size={18} /></button></div>
          <div className="table-scroll"><table className="dh-table"><thead><tr><th>i</th><th>Tipo</th><th>a (u)</th><th>α (°)</th><th>θ (°)</th><th>d (u)</th><th></th></tr></thead>
            <tbody>{joints.map((j, i) => <tr key={j.id}><td>{i + 1}</td><td><select aria-label={`Tipo ${i + 1}`} value={j.type} onChange={e => updateJoint(j.id, { type: e.target.value, val: 0, speed: 0, min: e.target.value === 'R' ? -180 : 0, max: e.target.value === 'R' ? 180 : 10 })}><option>R</option><option>P</option></select></td>
              {['a', 'alpha', 'theta', 'd'].map(key => <td key={key}>{numeric(`${key}${i + 1}`, (key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? j.val : j[key], v => updateJoint(j.id, { [(key === 'theta' && j.type === 'R') || (key === 'd' && j.type === 'P') ? 'val' : key]: v }), { compact: true })}</td>)}
              <td><button className="icon-btn danger" title={`Remover junta ${i + 1}`} aria-label={`Remover junta ${i + 1}`} disabled={joints.length === 1} onClick={() => commitJoints(joints.filter(item => item.id !== j.id))}><Trash2 size={15} /></button></td></tr>)}</tbody>
          </table></div>
        </section>
        <section><h2>Simulações</h2><form className="save-row" onSubmit={e => { e.preventDefault(); void persist(async () => { await request('', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: simName.trim(), data: joints, waypoints }) }); setSimName(''); await refresh(); setMessage('Simulação salva.'); }); }}>
          <input aria-label="Nome da simulação" placeholder="Nome da simulação" value={simName} maxLength={120} onChange={e => setSimName(e.target.value)} required />
          <button className="primary" disabled={busy || !simName.trim()}><Save size={16} />Salvar</button></form>
          <div className="action-row" style={{ marginTop: '10px' }}>
            <button disabled={busy || shareLoading} onClick={handleShare}>
              <Share2 size={16} /> Compartilhar link público
            </button>
          </div>
          {shareUrl && <div className="share-box">
            <strong>Link público de compartilhamento:</strong>
            <div className="share-url-row">
              <input readOnly value={shareUrl} aria-label="Link de compartilhamento" />
              <button onClick={copyShareUrl}>
                {shareCopied ? <CheckCheck size={14} /> : <Copy size={14} />}
                {shareCopied ? 'Copiado' : 'Copiar'}
              </button>
            </div>
          </div>}
          {apiError && <div className="notice error" role="alert">{apiError}<button onClick={() => void refresh()}>Reconectar</button></div>}
          {!apiError && simulations.length === 0 && <p className="muted">Nenhuma simulação salva.</p>}
          <ul className="saved-list">{simulations.map(sim => <li key={sim.id}><span>{sim.name}</span><button disabled={busy} className="icon-btn" title={`Carregar ${sim.name}`} aria-label={`Carregar ${sim.name}`} onClick={() => void persist(async () => { const data = await request(`/${sim.id}`); commitJoints(normalizeJoints(data.data)); if (data.waypoints) setWaypoints(data.waypoints); setViewKey(k => k + 1); setMessage('Simulação carregada.'); })}><Download size={16} /></button><button disabled={busy} className="icon-btn danger" title={`Apagar ${sim.name}`} aria-label={`Apagar ${sim.name}`} onClick={() => { if (window.confirm(`Apagar "${sim.name}"?`)) void persist(async () => { await request(`/${sim.id}`, { method: 'DELETE' }); await refresh(); }); }}><Trash2 size={16} /></button></li>)}</ul>
        </section>
      </>}

      {tab === 'forward' && <>
        <section><h2>Coordenadas globais (u)</h2><MatrixView label="Origens dos referenciais" headers={['i', 'X', 'Y', 'Z']} values={fk.positions.map((p, i) => [i, ...p])} /></section>
        <section><h2>Transformações homogêneas</h2><div className="formula">Aᵢ = Rz(θᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)</div>
          <MatrixView label="T⁰ₙ = A₁ · … · Aₙ" values={matrixRows(fk.total)} />
          {fk.links.map((matrix, i) => <details key={joints[i].id}><summary>A{i + 1} · referencial {i} → {i + 1}</summary><MatrixView label={`A${i + 1}`} values={matrixRows(matrix)} /><MatrixView label={`T⁰${i + 1}`} values={matrixRows(fk.transforms[i + 1])} /></details>)}
        </section>
        <section><h2>Orientação</h2><MatrixView label="R⁰ₙ" values={fk.orientation.rotation} />
          <dl className="readings"><dt>RPY ZYX (°)</dt><dd>{vectorText(fk.orientation.rpy)}</dd><dt>Composição</dt><dd>Rz(yaw) Ry(pitch) Rx(roll)</dd><dt>Eixo de rotação</dt><dd>{fk.orientation.axis ? vectorText(fk.orientation.axis) : 'Indeterminado (rotação nula)'}</dd><dt>Ângulo (°)</dt><dd>{fmt(fk.orientation.angle)}</dd><dt>det(R)</dt><dd>{fmt(fk.orientation.determinant, 6)}</dd><dt>‖RᵀR − I‖</dt><dd>{fk.orientation.orthogonality.toExponential(2)}</dd></dl>
          {fk.orientation.gimbalLock && <p className="notice">Singularidade RPY: roll e yaw não são únicos.</p>}
        </section>
      </>}

      {tab === 'differential' && <>
        <section><h2>Jacobiano geométrico</h2>{taskSettings}
          <MatrixView label="J · linhas [vx, vy, vz, ωx, ωy, ωz]" headers={joints.map((_, i) => `q̇${i + 1}`)} values={fk.jacobian} />
          <dl className="readings"><dt>Posto / máximo algébrico</dt><dd>{diagnostics.rank} / {diagnostics.maxRank}</dd><dt>Dimensões da tarefa</dt><dd>{diagnostics.dimensions}</dd><dt>Condição normalizada</dt><dd>{fmt(diagnostics.condition)}</dd><dt>Valores singulares normalizados</dt><dd>{vectorText(diagnostics.singularValues)}</dd><dt>det(J da tarefa)</dt><dd>{diagnostics.determinant === null ? 'Não se aplica (retangular)' : fmt(diagnostics.determinant, 6)}</dd></dl>
          <p className={`notice ${diagnostics.fullTask ? 'success' : ''}`}>
            {diagnostics.fullTask ? 'Tarefa localmente controlável.' : diagnostics.deficient ? 'Posto reduzido: singularidade ou restrição estrutural da tarefa.' : 'Graus de liberdade insuficientes para controlar toda a tarefa.'}
            {diagnostics.condition > 1000 && Number.isFinite(diagnostics.condition) ? ' Próximo de perda de posto.' : ''}
          </p>
        </section>
        <section><h2>Velocidades articulares</h2><div className="field-grid">{joints.map((j, i) => <div key={j.id}>{numeric(`q̇${i + 1} (${j.type === 'R' ? 'rad/s' : 'u/s'})`, j.speed, speed => updateJoint(j.id, { speed }))}</div>)}</div>
          <MatrixView label="v = J q̇ · base global" values={velocity.map(v => [v])} />
          <div className="action-row">{numeric('Δt (s)', dt, setDt, { min: 0.001, max: 10 })}<button onClick={step}><Activity size={16} />Aplicar passo</button></div>
        </section>
        <section><h2>Inversa diferencial</h2><div className="field-grid">{TASK_ROWS[mode].map(i => <div key={i}>{numeric(velocityLabels[i], desiredVelocity[i], value => setDesiredVelocity(prev => prev.map((v, k) => k === i ? value : v)))}</div>)}</div>
          <MatrixView label="q̇ calculado · R: rad/s; P: u/s" values={differential.rates.map(v => [v])} />
          <MatrixView label="Velocidade obtida [u/s; rad/s]" values={differential.achieved.map(v => [v])} />
          <div className="metric">Resíduo normalizado: <output>{fmt(differential.error, 6)}</output></div>
          <button onClick={() => { if (commitJoints(joints.map((j, i) => ({ ...j, speed: differential.rates[i] })))) setMessage('Velocidades articulares atualizadas.'); }}><Check size={16} />Usar velocidades</button>
        </section>
      </>}

      {tab === 'inverse' && <>
        <section><h2>Alvo cartesiano</h2>{taskSettings}<div className="field-grid">{['X', 'Y', 'Z'].map((axis, i) => (mode !== 'xy' || i < 2) && <div key={axis}>{numeric(`Alvo ${axis} (u)`, target.position[i], v => changeTarget('position', i, v))}</div>)}</div>
          {mode === 'pose' && <div className="field-grid">{['Roll', 'Pitch', 'Yaw'].map((axis, i) => <div key={axis}>{numeric(`${axis} alvo (°)`, target.rpy[i], v => changeTarget('rpy', i, v))}</div>)}</div>}
          <div className="action-row"><button onClick={() => { setTarget({ position: fk.positions.at(-1), rpy: fk.orientation.rpy }); setSolution(null); }}><Crosshair size={16} />Capturar pose atual</button><button className="primary" onClick={() => { setSolution(inverseKinematics(joints, target, { mode, length, damping })); setMessage(''); }}>Resolver</button></div>
        </section>
        {solution && <section><h2>Resultado</h2><p className={`notice ${solution.converged ? 'success' : ''}`} role="status">{solution.converged ? 'Solução convergiu dentro dos limites.' : 'Não convergiu. Possíveis causas: alvo incompatível, limites ou mínimo local.'}</p>
          <dl className="readings"><dt>Iterações totais</dt><dd>{solution.iterations}</dd><dt>Resíduo normalizado</dt><dd>{fmt(solution.error, 7)}</dd><dt>Erro de posição XYZ (u)</dt><dd>{vectorText(solution.residual.slice(0, 3))}</dd>{mode === 'pose' && <><dt>Erro angular XYZ (rad)</dt><dd>{vectorText(solution.residual.slice(3))}</dd></>}</dl>
          <MatrixView label="Configuração encontrada · R: °; P: u" values={solution.joints.map(j => [j.val])} />
          <button className="primary" disabled={!solution.converged} onClick={() => { const next = solution.joints; commitJoints(next); setMessage('Solução aplicada ao manipulador.'); }}><Check size={16} />Aplicar solução</button>
        </section>}
      </>}

      {tab === 'trajectory' && <>
        <section>
          <h2>Sequenciador de Trajetórias</h2>
          <p className="muted">Interpolação contínua de movimentos articulares via splines quínticas e cúbicas sem descontinuidades nos atuadores.</p>

          <div className="timeline-panel">
            <div className="timeline-slider-row">
              <span>{fmt(playTime, 2)} s</span>
              <input
                type="range"
                className="timeline-slider"
                aria-label="Timeline da trajetória"
                min={0}
                max={Math.max(0.01, trajectoryData.totalDuration)}
                step="any"
                value={playTime}
                onChange={e => seekTrajectory(Number(e.target.value))}
              />
              <span>{fmt(trajectoryData.totalDuration, 2)} s</span>
            </div>

            <div className="timeline-controls">
              <div className="timeline-btns">
                <button
                  className="primary"
                  aria-label={isPlaying ? "Pausar reprodução" : "Iniciar reprodução"}
                  onClick={() => {
                    if (playTime >= trajectoryData.totalDuration) setPlayTime(0);
                    setIsPlaying(p => !p);
                  }}
                >
                  {isPlaying ? <Pause size={16} /> : <Play size={16} />}
                  {isPlaying ? 'Pausar' : 'Reproduzir'}
                </button>
                <button
                  aria-label="Parar e reiniciar timeline"
                  onClick={() => {
                    setIsPlaying(false);
                    seekTrajectory(0);
                  }}
                >
                  <Square size={16} /> Parar
                </button>
              </div>

              <div className="timeline-opts">
                <label>
                  Velocidade:
                  <select aria-label="Velocidade de reprodução" value={playbackSpeed} onChange={e => setPlaybackSpeed(Number(e.target.value))}>
                    <option value={0.5}>0.5x</option>
                    <option value={1.0}>1.0x</option>
                    <option value={1.5}>1.5x</option>
                    <option value={2.0}>2.0x</option>
                  </select>
                </label>
                <label style={{ display: 'flex', alignItems: 'center', gap: '5px', cursor: 'pointer' }}>
                  <input type="checkbox" checked={isLooping} onChange={e => setIsLooping(e.target.checked)} />
                  <Repeat size={14} /> Loop
                </label>
              </div>
            </div>
          </div>

          <div className="field-grid">
            <label>Polinômio de Interpolação
              <select aria-label="Tipo de interpolação" value={interpType} onChange={e => setInterpType(e.target.value)}>
                <option value="quintic">Quíntico (q̈ = 0 nas pontas, sem solavanco)</option>
                <option value="cubic">Cúbico (q̇ = 0 nas pontas)</option>
              </select>
            </label>
            <div style={{ display: 'flex', alignItems: 'flex-end' }}>
              <button style={{ width: '100%' }} onClick={addCurrentWaypoint}>
                <Plus size={16} /> Adicionar pose atual
              </button>
            </div>
          </div>
        </section>

        <section>
          <div className="section-title">
            <h2>Waypoints da sequência ({waypoints.length})</h2>
            <label style={{ display: 'flex', alignItems: 'center', gap: '6px', fontSize: '12px', cursor: 'pointer' }}>
              <input type="checkbox" checked={showTrajectory} onChange={e => setShowTrajectory(e.target.checked)} />
              Trilha 3D
            </label>
          </div>
          <ul className="waypoint-list">
            {waypoints.map((wp, idx) => (
              <li key={wp.id} className="waypoint-item">
                <div className="waypoint-info">
                  <strong>{idx + 1}. {wp.name}</strong>
                  <span className="waypoint-coords">
                    q: [{wp.q.map(v => fmt(v, 1)).join(', ')}]
                  </span>
                </div>
                {idx > 0 && (
                  <div className="waypoint-duration">
                    <span className="muted">Δt:</span>
                    <input
                      type="number"
                      aria-label={`Duração do waypoint ${idx + 1}`}
                      min={0.1}
                      max={60}
                      step={0.5}
                      defaultValue={wp.duration}
                      onBlur={e => updateWaypointDuration(wp.id, Number(e.target.value))}
                    />
                    <span className="muted">s</span>
                  </div>
                )}
                <div style={{ display: 'flex', gap: '4px' }}>
                  <button className="icon-btn" title="Ir para waypoint" aria-label={`Ir para ${wp.name}`} onClick={() => moveToWaypoint(wp)}>
                    <Focus size={15} />
                  </button>
                  <button
                    className="icon-btn danger"
                    title="Remover waypoint"
                    aria-label={`Remover ${wp.name}`}
                    disabled={waypoints.length <= 1}
                    onClick={() => removeWaypoint(wp.id)}
                  >
                    <Trash2 size={15} />
                  </button>
                </div>
              </li>
            ))}
          </ul>
        </section>
      </>}

      {tab === 'workspace' && <>
        <section>
          <h2>Espaço de Trabalho 3D (Monte Carlo)</h2>
          <p className="muted">Geração do envelope de alcance com cálculo de destreza e índice de manipulabilidade de Yoshikawa.</p>
          <div className="field-grid">
            <label>Amostras
              <select aria-label="Número de amostras" value={sampleCount} onChange={e => setSampleCount(Number(e.target.value))}>
                <option value={1000}>1.000 pontos (Rápido)</option>
                <option value={2500}>2.500 pontos (Equilibrado)</option>
                <option value={5000}>5.000 pontos (Alta densidade)</option>
              </select>
            </label>
            <div style={{ display: 'flex', alignItems: 'flex-end' }}>
              <button className="primary" style={{ width: '100%' }} disabled={computingCloud} onClick={computeWorkspace}>
                {computingCloud ? 'Calculando...' : 'Calcular alcance'}
              </button>
            </div>
          </div>
        </section>
        {workspaceCloud && <section>
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
            <label style={{ display: 'flex', alignItems: 'center', gap: '8px', fontSize: '13px', cursor: 'pointer' }}>
              <input type="checkbox" checked={showWorkspace} onChange={e => setShowWorkspace(e.target.checked)} />
              Exibir pontos na cena 3D
            </label>
          </div>
          <p className="notice" style={{ marginTop: '16px' }}>
            Gradiente térmico: <strong style={{ color: '#4da6ff' }}>Azul</strong> = proximidade de singularidade (baixa destreza); <strong style={{ color: '#ffea79' }}>Amarelo/Vermelho</strong> = alta destreza articular.
          </p>
        </section>}
      </>}

      {tab === 'export' && <>
        <section>
          <h2>Exportação de Código de Controle</h2>
          <p className="muted">Gera implementação autônoma e autocontida da cinemática para microcontroladores ou robótica em Python.</p>
          <div className="code-target-tabs">
            {[['cpp', 'C++ / Arduino'], ['python', 'Python / NumPy'], ['ros2', 'ROS 2 (Node)'], ['urdf', 'URDF (ROS)']].map(([id, lbl]) =>
              <button key={id} aria-current={codeTarget === id ? 'page' : undefined} onClick={() => setCodeTarget(id)}>{lbl}</button>)}
          </div>
          <div className="code-box-header">
            <span className="muted">{codeTarget === 'cpp' ? 'Arduino / ESP32 / C++11' : codeTarget === 'python' ? 'NumPy script' : codeTarget === 'ros2' ? 'rclpy node' : 'URDF XML (ROS)'}</span>
            <div style={{ display: 'flex', gap: '6px' }}>
              {codeTarget === 'urdf' && (
                <button onClick={downloadUrdf}>
                  <Download size={16} /> Baixar .urdf
                </button>
              )}
              <button onClick={copyCode}>{copied ? <CheckCheck size={16} /> : <Copy size={16} />}{copied ? 'Copiado!' : 'Copiar código'}</button>
            </div>
          </div>
          <pre className="code-block"><code>{generatedCode}</code></pre>
        </section>
      </>}

      {tab === 'report' && <>
        <section>
          <h2>Relatório Acadêmico (PDF / Impressão)</h2>
          <p className="muted">Geração de documentação formal para aulas e exercícios de robótica com parâmetros DH, matrizes de transformação e Jacobiano.</p>
          <div className="field-grid">
            <label>Nome do Aluno
              <input aria-label="Nome do Aluno" value={reportMeta.studentName} onChange={e => setReportMeta({ ...reportMeta, studentName: e.target.value })} placeholder="Lucas Engenharia" />
            </label>
            <label>Disciplina / Curso
              <input aria-label="Disciplina / Curso" value={reportMeta.courseName} onChange={e => setReportMeta({ ...reportMeta, courseName: e.target.value })} placeholder="Robótica e Cinemática" />
            </label>
            <label>Instituição
              <input aria-label="Instituição" value={reportMeta.institution} onChange={e => setReportMeta({ ...reportMeta, institution: e.target.value })} placeholder="Universidade Federal / Faculdade" />
            </label>
            <label>Título / Modelo
              <input aria-label="Título / Modelo" value={reportMeta.robotName} onChange={e => setReportMeta({ ...reportMeta, robotName: e.target.value })} placeholder={`Manipulador ${joints.map(j => j.type).join('')}`} />
            </label>
          </div>
          <div style={{ marginTop: '16px' }}>
            <button className="primary" onClick={() => openAcademicReport(joints, { ...reportMeta, robotName: reportMeta.robotName || `Manipulador ${joints.map(j => j.type).join('')}` })}>
              <Printer size={16} /> Gerar e Imprimir Relatório (PDF)
            </button>
          </div>
        </section>
        <section>
          <h2>Resumo dos Cálculos Gerados</h2>
          <dl className="readings">
            <dt>Manipulador</dt><dd>{joints.map(j => j.type).join('')} ({joints.length} DOF)</dd>
            <dt>Posição XYZ (u)</dt><dd>{vectorText(fk.positions.at(-1))}</dd>
            <dt>Orientação RPY (°)</dt><dd>{vectorText(fk.orientation.rpy)}</dd>
            <dt>Posto do Jacobiano</dt><dd>{diagnostics.rank} de {diagnostics.maxRank}</dd>
            <dt>Condicionamento</dt><dd>{fmt(diagnostics.condition)}</dd>
          </dl>
        </section>
      </>}
    </aside>
    <div className="canvas-container">
      <RobotScene
        joints={joints}
        showFrames={showFrames}
        viewKey={viewKey}
        workspaceCloud={workspaceCloud}
        showWorkspace={showWorkspace}
        trajectoryPathPoints={trajectoryData.pathPoints}
        trajectoryWaypointCoords={trajectoryData.waypointCoords}
        showTrajectory={showTrajectory}
        customStlGeometry={customStlGeometry}
      />
      <div className="scene-toolbar">
        <label><input type="checkbox" checked={showFrames} onChange={e => setShowFrames(e.target.checked)} />Referenciais</label>
        {trajectoryData.pathPoints.length > 0 && <label><input type="checkbox" checked={showTrajectory} onChange={e => setShowTrajectory(e.target.checked)} />Trilha 3D</label>}
        {workspaceCloud && <label><input type="checkbox" checked={showWorkspace} onChange={e => setShowWorkspace(e.target.checked)} />Nuvem 3D</label>}
        {customStlGeometry ? (
          <button className="icon-btn danger" title={`Remover malha STL (${stlFileName})`} aria-label="Remover malha STL" onClick={removeStlMesh}>
            <Trash2 size={16} />
          </button>
        ) : (
          <label className="icon-btn" title="Carregar malha STL (Garra / Ferramenta)" aria-label="Carregar malha STL" style={{ cursor: 'pointer', display: 'inline-flex', alignItems: 'center' }}>
            <Upload size={17} />
            <input type="file" accept=".stl" style={{ display: 'none' }} onChange={handleStlUpload} aria-label="Upload de malha STL" />
          </label>
        )}
        <button className="icon-btn" title="Enquadrar manipulador" aria-label="Enquadrar manipulador" onClick={() => setViewKey(k => k + 1)}><Focus size={19} /></button>
      </div>
      <div className="scene-readout"><span>Efetuador · base global</span><output data-testid="end-position">X {fmt(fk.positions.at(-1)[0], 2)} · Y {fmt(fk.positions.at(-1)[1], 2)} · Z {fmt(fk.positions.at(-1)[2], 2)} u</output></div>
    </div>
  </main>;
}

export default App;
