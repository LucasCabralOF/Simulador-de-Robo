import { memo } from 'react';
import { Printer } from 'lucide-react';
import { Button } from '../ui';
import { TOOL_TYPES } from '../kinematics';
import { openAcademicReport } from '../reportGenerator';
import { fmt, vectorText } from '../format';

const ReportTab = memo(function ReportTab({
  joints,
  fk,
  toolType,
  toolOffsetRpy,
  toolGrip,
  toolFrame,
  diagnostics,
  reportMeta,
  setReportMeta,
}) {
  return (
    <>
      <section>
        <h2>Relatório Acadêmico (PDF / Impressão)</h2>
        <p className="muted">Geração de documentação formal para aulas e exercícios de robótica com parâmetros DH, matrizes de transformação e Jacobiano.</p>
        <div className="field-grid">
          <label>Nome do Aluno
            <input className="ui-input" aria-label="Nome do Aluno" value={reportMeta.studentName} onChange={e => setReportMeta({ ...reportMeta, studentName: e.target.value })} placeholder="Lucas Engenharia" />
          </label>
          <label>Disciplina / Curso
            <input className="ui-input" aria-label="Disciplina / Curso" value={reportMeta.courseName} onChange={e => setReportMeta({ ...reportMeta, courseName: e.target.value })} placeholder="Robótica e Cinemática" />
          </label>
          <label>Instituição
            <input className="ui-input" aria-label="Instituição" value={reportMeta.institution} onChange={e => setReportMeta({ ...reportMeta, institution: e.target.value })} placeholder="Universidade Federal / Faculdade" />
          </label>
          <label>Título / Modelo
            <input className="ui-input" aria-label="Título / Modelo" value={reportMeta.robotName} onChange={e => setReportMeta({ ...reportMeta, robotName: e.target.value })} placeholder={`Manipulador ${joints.map(j => j.type).join('')}`} />
          </label>
        </div>
        <div className="section-actions">
          <Button variant="primary" onClick={() => openAcademicReport(joints, { ...reportMeta, robotName: reportMeta.robotName || `Manipulador ${joints.map(j => j.type).join('')}`, toolType, toolOffsetRpy, toolGrip })}>
            <Printer size={16} /> Gerar e Imprimir Relatório (PDF)
          </Button>
        </div>
      </section>
      <section>
        <h2>Resumo dos Cálculos Gerados</h2>
        <dl className="readings">
          <dt>Manipulador</dt><dd>{joints.map(j => j.type).join('')} ({joints.length} DOF)</dd>
          <dt>Ferramenta (TCP)</dt><dd>{TOOL_TYPES[toolType].shortLabel} · {fmt(TOOL_TYPES[toolType].length, 2)} u</dd>
          <dt>Posição XYZ (u)</dt><dd>{vectorText(fk.positions.at(-1))}</dd>
          <dt>Posição do TCP (u)</dt><dd>{vectorText(toolFrame.tcpPosition)}</dd>
          <dt>Orientação RPY (°)</dt><dd>{vectorText(toolFrame.orientation.rpy)}</dd>
          <dt>Posto do Jacobiano</dt><dd>{diagnostics.rank} de {diagnostics.maxRank}</dd>
          <dt>Condicionamento</dt><dd>{fmt(diagnostics.condition)}</dd>
        </dl>
      </section>
    </>
  );
});

export default ReportTab;
