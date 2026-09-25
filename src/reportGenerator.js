import { forwardKinematics, jacobianDiagnostics, matrixRows } from './kinematics.js';

const fmt = (n, digits = 4) => !Number.isFinite(n) ? 'inf' : (Math.abs(n) < 0.5 * 10 ** -digits ? 0 : n).toFixed(digits);

export function generateAcademicReportHtml(joints, metadata = {}) {
  const {
    studentName = 'Estudante de Engenharia',
    courseName = 'Robótica Industrial / Cinemática',
    institution = 'Universidade / Instituto Tecnológico',
    robotName = `Manipulador ${joints.map(j => j.type).join('')}`,
    date = new Date().toLocaleDateString('pt-BR'),
  } = metadata;

  const fk = forwardKinematics(joints);
  const diag = jacobianDiagnostics(joints, fk.jacobian, 'position', 5);
  const end = fk.positions.at(-1);

  const dhTableRows = joints.map((j, i) => `
    <tr>
      <td>${i + 1}</td>
      <td><strong>${j.type === 'R' ? 'Rotativa (R)' : 'Prismática (P)'}</strong></td>
      <td>${fmt(j.a)}</td>
      <td>${fmt(j.alpha)}°</td>
      <td>${j.type === 'R' ? `${fmt(j.val)}° (var)` : `${fmt(j.theta)}°`}</td>
      <td>${j.type === 'P' ? `${fmt(j.val)} (var)` : `${fmt(j.d)}`}</td>
      <td>[${j.min}, ${j.max}]</td>
    </tr>
  `).join('');

  const homogeneousSections = fk.links.map((link, i) => {
    const rows = matrixRows(link);
    const tableHtml = rows.map(r => `<tr>${r.map(v => `<td>${fmt(v)}</td>`).join('')}</tr>`).join('');
    return `
      <div class="matrix-card">
        <h4>Matriz A${i + 1} (Referencial ${i} para ${i + 1})</h4>
        <table class="math-table">${tableHtml}</table>
      </div>
    `;
  }).join('');

  const totalRows = matrixRows(fk.total);
  const totalTableHtml = totalRows.map(r => `<tr>${r.map(v => `<td>${fmt(v)}</td>`).join('')}</tr>`).join('');

  const jacobianRows = fk.jacobian.map((row, rIdx) => {
    const labels = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'];
    return `<tr><th>${labels[rIdx]}</th>${row.map(v => `<td>${fmt(v)}</td>`).join('')}</tr>`;
  }).join('');

  return `<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <title>Relatório Acadêmico - ${robotName}</title>
  <style>
    @page { size: A4; margin: 20mm; }
    body { font-family: 'Segoe UI', system-ui, sans-serif; color: #1a1a1a; line-height: 1.5; margin: 0; padding: 24px; background: #fff; }
    .header { border-bottom: 2px solid #005f73; padding-bottom: 12px; margin-bottom: 24px; }
    .header h1 { margin: 0 0 6px; color: #005f73; font-size: 22px; }
    .meta-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; font-size: 13px; color: #4a4a4a; margin-top: 10px; }
    h2 { color: #0a9396; font-size: 16px; border-bottom: 1px solid #e0e0e0; padding-bottom: 4px; margin-top: 24px; }
    table { width: 100%; border-collapse: collapse; margin: 12px 0; font-size: 12px; }
    th, td { border: 1px solid #ccc; padding: 6px 8px; text-align: center; }
    th { background: #f4f6f8; color: #333; }
    .math-table { font-family: 'Courier New', monospace; font-size: 11px; }
    .math-table td { width: 25%; text-align: right; padding: 4px 6px; }
    .matrices-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin: 12px 0; }
    .matrix-card { border: 1px solid #ddd; border-radius: 4px; padding: 10px; background: #fafafa; }
    .matrix-card h4 { margin: 0 0 8px; font-size: 12px; color: #333; }
    .summary-box { background: #e8f4f8; border-left: 4px solid #0a9396; padding: 12px; margin: 14px 0; font-size: 13px; }
    .btn-bar { margin-bottom: 20px; }
    .print-btn { background: #005f73; color: white; border: 0; padding: 10px 18px; border-radius: 4px; font-weight: 600; cursor: pointer; }
    @media print { .btn-bar { display: none; } body { padding: 0; } }
  </style>
</head>
<body>
  <div class="btn-bar">
    <button class="print-btn" onclick="window.print()">Imprimir / Salvar em PDF</button>
  </div>

  <div class="header">
    <h1>Relatório de Análise Cinemática - Robótica</h1>
    <div class="meta-grid">
      <div><strong>Manipulador:</strong> ${robotName} (${joints.length} Graus de Liberdade)</div>
      <div><strong>Data:</strong> ${date}</div>
      <div><strong>Aluno:</strong> ${studentName}</div>
      <div><strong>Disciplina:</strong> ${courseName}</div>
      <div><strong>Instituição:</strong> ${institution}</div>
      <div><strong>Arquitetura:</strong> ${joints.map(j => j.type).join('')}</div>
    </div>
  </div>

  <h2>1. Parâmetros de Denavit-Hartenberg (Padrão)</h2>
  <p style="font-size: 12px; color: #555;">Convenção padrão: A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)</p>
  <table>
    <thead>
      <tr>
        <th>Elo (i)</th><th>Tipo</th><th>a_i (comprimento)</th><th>alpha_i (torção)</th><th>theta_i (ângulo)</th><th>d_i (deslocamento)</th><th>Limites [mín, máx]</th>
      </tr>
    </thead>
    <tbody>${dhTableRows}</tbody>
  </table>

  <h2>2. Transformação Homogênea Resultante (T0_n)</h2>
  <div class="summary-box">
    <strong>Posição do Efetuador:</strong> X = ${fmt(end[0])} u, Y = ${fmt(end[1])} u, Z = ${fmt(end[2])} u<br>
    <strong>Orientação (RPY ZYX):</strong> Roll = ${fmt(fk.orientation.rpy[0])}°, Pitch = ${fmt(fk.orientation.rpy[1])}°, Yaw = ${fmt(fk.orientation.rpy[2])}°<br>
    <strong>Ortogonalidade:</strong> det(R) = ${fmt(fk.orientation.determinant, 6)}, ||R^T*R - I|| = ${fk.orientation.orthogonality.toExponential(2)}
  </div>
  <table class="math-table" style="max-width: 480px;">${totalTableHtml}</table>

  <h2>3. Matrizes Homogêneas Intermediárias</h2>
  <div class="matrices-grid">${homogeneousSections}</div>

  <h2>4. Jacobiano Geométrico e Análise de Singularidades</h2>
  <div class="summary-box">
    <strong>Posto do Jacobiano:</strong> ${diag.rank} de ${diag.maxRank} (Dimensões da tarefa: ${diag.dimensions})<br>
    <strong>Estado:</strong> ${diag.fullTask ? 'Posto completo (tarefa controlável)' : 'Posto reduzido / sob restrição'}<br>
    <strong>Número de Condicionamento:</strong> ${fmt(diag.condition)}
  </div>
  <table class="math-table">${jacobianRows}</table>
</body>
</html>`;
}

export function openAcademicReport(joints, metadata = {}) {
  const html = generateAcademicReportHtml(joints, metadata);
  const printWindow = window.open('', '_blank');
  if (printWindow) {
    printWindow.document.open();
    printWindow.document.write(html);
    printWindow.document.close();
  }
}
