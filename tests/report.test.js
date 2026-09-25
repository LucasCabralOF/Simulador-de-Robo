import test from 'node:test';
import assert from 'node:assert/strict';
import { generateAcademicReportHtml } from '../src/reportGenerator.js';
import { PRESETS } from '../src/presets.js';

test('academic report generator outputs comprehensive technical document', () => {
  const puma = PRESETS.puma560.joints;
  const metadata = {
    studentName: 'Lucas Engenharia',
    courseName: 'Controle e Cinemática de Robôs',
    institution: 'Universidade Federal',
    robotName: 'Puma 560 Laboratório',
  };

  const html = generateAcademicReportHtml(puma, metadata);

  assert.ok(html.includes('Lucas Engenharia'));
  assert.ok(html.includes('Controle e Cinemática de Robôs'));
  assert.ok(html.includes('Puma 560 Laboratório'));
  assert.ok(html.includes('Parâmetros de Denavit-Hartenberg'));
  assert.ok(html.includes('Transformação Homogênea Resultante'));
  assert.ok(html.includes('Jacobiano Geométrico'));
  assert.ok(html.includes('window.print()'));

  // Ensure no LaTeX delimiters ($ or $$)
  assert.ok(!html.includes('$$'));
  assert.ok(!html.includes('$q_'));
});
