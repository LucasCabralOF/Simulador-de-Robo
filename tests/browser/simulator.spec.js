import { test, expect } from '@playwright/test';
import { Box3, PerspectiveCamera, Vector3 } from 'three';
import { forwardKinematics, hslToHex } from '../../src/kinematics.js';
import { PRESETS } from '../../src/presets.js';

async function number(page, label, value) {
  const input = page.getByRole('spinbutton', { name: label, exact: true });
  await input.fill(String(value));
  await input.press('Tab');
}

async function pickOption(page, label, optionText) {
  await page.getByRole('combobox', { name: label, exact: true }).click();
  await page.getByRole('option', { name: optionText, exact: true }).click();
}

async function pixels(page) {
  return page.locator('canvas').evaluate(canvas => {
    const gl = canvas.getContext('webgl2');
    if (!gl) throw new Error('WebGL2 unavailable');
    const data = new Uint8Array(canvas.width * canvas.height * 4);
    gl.readPixels(0, 0, canvas.width, canvas.height, gl.RGBA, gl.UNSIGNED_BYTE, data);
    let pink = 0, green = 0, gold = 0, checksum = 0;
    for (let i = 0; i < data.length; i += 4) {
      const [r, g, b] = [data[i], data[i + 1], data[i + 2]];
      if (r > 110 && r > g * 1.25 && b > g * 1.05) pink++;
      if (g > 90 && g > r * 1.2 && g > b * 1.03) green++;
      if (r > 140 && g > 110 && b < g * 0.85) gold++;
      checksum = (checksum + (r + g * 3 + b * 7) * ((i % 997) + 1)) % 2147483647;
    }
    return { pink, green, gold, checksum, width: canvas.width, height: canvas.height };
  });
}

test('numerical workflows, persistence, responsive layout and interactive WebGL', async ({ page }, info) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  // Default state boots into 100% 3D Cobot UR5 canvas with floating panels collapsed
  await expect(page.locator('.scene-popover')).toHaveCount(0);
  await expect(page.locator('.scene-brand')).toContainText('Manipulador RRRRRR · 6 juntas');
  const [ur5X, ur5Y, ur5Z] = forwardKinematics(PRESETS.ur5.joints).positions.at(-1).map(v => v.toFixed(2));
  await expect(page.getByTestId('end-position')).toContainText(`X ${ur5X} · Y ${ur5Y} · Z ${ur5Z}`);
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);
  const before = await pixels(page);
  expect(before.green).toBeGreaterThan(10);
  expect(before.gold).toBeGreaterThan(5);
  await page.screenshot({ path: info.outputPath('model.png'), fullPage: true });

  const canvas = page.locator('canvas');
  await canvas.scrollIntoViewIfNeeded();
  const bounds = await canvas.boundingBox();
  const orbitBefore = await pixels(page);
  await page.mouse.move(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2);
  await page.mouse.down();
  await page.mouse.move(bounds.x + bounds.width / 2 + 60, bounds.y + bounds.height / 2 + 30, { steps: 10 });
  await page.mouse.up();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(orbitBefore.checksum);
  await page.getByRole('button', { name: 'Enquadrar manipulador' }).click();

  // Open Modelo floating popover from slim top bar
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await expect(page.locator('.scene-popover')).toBeVisible();
  await number(page, 'q1 (°)', 35);
  await expect(page.getByTestId('end-position')).not.toContainText(`X ${ur5X} · Y ${ur5Y}`);
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(before.checksum);
  await page.getByRole('button', { name: 'Restaurar Cobot UR5 padrão' }).click();
  await expect(page.getByTestId('end-position')).toContainText(`X ${ur5X} · Y ${ur5Y} · Z ${ur5Z}`);

  await number(page, 'Mín. 1 (°)', 200);
  await expect(page.locator('.notice[role="status"]')).toContainText('mínimo deve ser menor');
  await expect(page.getByRole('spinbutton', { name: 'Mín. 1 (°)', exact: true })).toHaveValue('-180');

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  const simName = `browser-${info.project.name}`;
  await page.getByRole('textbox', { name: 'Nome da simulação' }).fill(simName);
  await page.getByRole('button', { name: 'Salvar', exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toBeVisible();
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await number(page, 'q1 (°)', 15);
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await page.getByRole('button', { name: `Carregar ${simName}`, exact: true }).click();
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'q1 (°)', exact: true })).toHaveValue('0');
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  page.once('dialog', dialog => dialog.accept());
  await page.getByRole('button', { name: `Apagar ${simName}`, exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toHaveCount(0);

  // Switch to Didático RPR (3-DOF) for analytical planar checks across Direta, Diferencial, and Inversa
  await pickOption(page, 'Selecionar preset industrial', 'Didático RPR (3-DOF) (3 DOF)');
  await expect(page.getByTestId('end-position')).toContainText('X 4.83 · Y 1.29 · Z 0.00');

  await page.getByRole('button', { name: 'Direta', exact: true }).click();
  await expect(page.getByRole('table', { name: 'T⁰ₙ = A₁ · … · Aₙ', exact: true })).toBeVisible();
  await page.getByText('A1 · referencial 0 → 1', { exact: true }).click();
  await expect(page.getByRole('table', { name: 'A1', exact: true })).toBeVisible();
  await expect(page.getByText('0.0000, 0.0000, 75.0000', { exact: true })).toBeVisible();
  await page.screenshot({ path: info.outputPath('forward.png'), fullPage: true });

  await page.getByRole('button', { name: 'Diferencial', exact: true }).click();
  await expect(page.getByText('Tarefa localmente controlável.', { exact: true })).toBeVisible();
  await pickOption(page, 'Tarefa', 'Posição XYZ');
  await expect(page.getByText('Posto reduzido:', { exact: false })).toBeVisible();
  await pickOption(page, 'Tarefa', 'Pose XYZ + orientação');
  await expect(page.getByText('Graus de liberdade insuficientes', { exact: false })).toBeVisible();
  await pickOption(page, 'Tarefa', 'Posição XY');
  await number(page, 'q̇1 (rad/s)', 1);
  await page.getByRole('button', { name: 'Aplicar passo', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Passo de 0.1 s aplicado');
  await number(page, 'vx (u/s)', 0.2);
  await page.getByRole('button', { name: 'Usar velocidades', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Velocidades articulares atualizadas');
  await expect(page.getByRole('heading', { name: 'Estática (τ = Jᵀ F)' })).toBeVisible();
  await page.getByRole('button', { name: 'Aplicar −m·g em Y', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'Fy (N)', exact: true })).toHaveValue('-20');
  await expect(page.getByRole('table', { name: 'Esforços nas juntas · R: Torque T (N·u); P: Força F (N)', exact: true })).toContainText('T1 (N·u)');
  await page.screenshot({ path: info.outputPath('differential.png'), fullPage: true });

  await page.getByRole('button', { name: 'Inversa', exact: true }).click();
  await number(page, 'Alvo X (u)', 4);
  await number(page, 'Alvo Y (u)', 2);
  await page.getByRole('button', { name: 'Resolver', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Solução convergiu');
  await expect(page.getByTestId('end-position')).toContainText('X 4.00 · Y 2.00 · Z 0.00');
  await expect(page.locator('.waypoint-item')).toHaveCount(3);
  await page.getByRole('button', { name: 'Salvar este ponto na trajetória', exact: true }).click();
  await expect(page.locator('.waypoint-item')).toHaveCount(4);
  await expect(page.locator('.notice[role="status"]')).toContainText('salvo na trajetória');
  await number(page, 'Alvo X (u)', 1000);
  await page.getByRole('button', { name: 'Resolver', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Não convergiu');
  await expect(page.getByTestId('end-position')).toContainText('X 4.00 · Y 2.00 · Z 0.00');
  await expect(page.getByRole('button', { name: 'Salvar este ponto na trajetória', exact: true })).toBeDisabled();
  await number(page, 'Alvo X (u)', 4);
  await expect(page.getByRole('heading', { name: 'Resultado', exact: true })).toHaveCount(0);
  await page.getByRole('checkbox', { name: 'Resolver e aplicar automaticamente ao editar alvo' }).check();
  await number(page, 'Alvo Y (u)', 1);
  await expect(page.getByTestId('end-position')).toContainText('X 4.00 · Y 1.00 · Z 0.00');
  const xSlider = page.getByRole('slider', { name: 'Deslizar Alvo X (u)', exact: true });
  await expect(xSlider).toBeVisible();
  expect(Number(await xSlider.getAttribute('max'))).toBeCloseTo(15, 0);
  await xSlider.fill('3');
  await expect(page.getByTestId('end-position')).toContainText('X 3.00 · Y 1.00 · Z 0.00');
  await number(page, 'Alvo Y (u)', 12);
  await expect(page.getByTestId('end-position')).toContainText('X 3.00 · Y 12.00 · Z 0.00');
  expect(Number(await xSlider.getAttribute('max'))).toBeLessThan(9.5);
  expect(Number(await xSlider.getAttribute('max'))).toBeGreaterThan(8.5);
  const xMax = await xSlider.getAttribute('max');
  const xMin = await xSlider.getAttribute('min');
  await xSlider.fill(String(xMax));
  await expect(page.getByRole('spinbutton', { name: 'Alvo Y (u)', exact: true })).toHaveValue('12');
  await expect(page.getByTestId('end-position')).toContainText('Y 12.00 · Z 0.00');
  await xSlider.fill(String(xMin));
  await expect(page.getByRole('spinbutton', { name: 'Alvo Y (u)', exact: true })).toHaveValue('12');
  await expect(page.getByTestId('end-position')).toContainText('Y 12.00 · Z 0.00');
  await page.screenshot({ path: info.outputPath('inverse.png'), fullPage: true });

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('API failures are visible and do not announce successful persistence', async ({ page }) => {
  await page.goto('/');
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await page.route('**/api/simulations', async route => {
    if (route.request().method() === 'POST') await route.fulfill({ status: 500, contentType: 'application/json', body: JSON.stringify({ error: 'Banco indisponível' }) });
    else await route.continue();
  });
  await page.getByRole('textbox', { name: 'Nome da simulação' }).fill('rejected');
  await page.getByRole('button', { name: 'Salvar', exact: true }).click();
  await expect(page.getByRole('alert')).toContainText('Banco indisponível');
  await expect(page.getByText('Simulação salva.', { exact: true })).toHaveCount(0);
});

async function motorPixels(page, joints, worldPoint) {
  const viewport = await page.locator('canvas').boundingBox();
  const fk = forwardKinematics(joints);
  const points = fk.positions.map(p => new Vector3(...p));
  joints.forEach((j, i) => points.push(new Vector3(0, 0, j.type === 'P' ? j.val : j.d).applyMatrix4(fk.transforms[i])));
  const box = new Box3().setFromPoints(points).expandByScalar(1.8);
  const center = box.getCenter(new Vector3());
  const camera = new PerspectiveCamera(45, viewport.width / viewport.height);
  const halfFov = Math.min(Math.PI / 8, Math.atan(Math.tan(Math.PI / 8) * camera.aspect));
  const distance = box.getSize(new Vector3()).length() / (2 * Math.sin(halfFov));
  camera.up.set(0, 0, 1);
  camera.position.copy(center).add(new Vector3(1, -1.4, 1.2).normalize().multiplyScalar(distance));
  camera.lookAt(center);
  camera.updateMatrixWorld();
  const projected = new Vector3(...worldPoint).project(camera);
  return page.locator('canvas').evaluate((canvas, point) => {
    const gl = canvas.getContext('webgl2');
    const radius = Math.ceil(5 * canvas.width / canvas.clientWidth);
    const x = Math.round((point.x + 1) * canvas.width / 2);
    const y = Math.round((point.y + 1) * canvas.height / 2);
    const width = radius * 2 + 1;
    const data = new Uint8Array(width * width * 4);
    gl.readPixels(x - radius, y - radius, width, width, gl.RGBA, gl.UNSIGNED_BYTE, data);
    let count = 0;
    for (let i = 0; i < data.length; i += 4) {
      const [r, g, b] = [data[i], data[i + 1], data[i + 2]];
      const isPinkMotor = r > 110 && r > g * 1.25 && b > g * 1.05;
      const isBlueMotor = b > 110 && b > g * 1.15 && b > r * 1.25;
      if (isPinkMotor || isBlueMotor) count++;
    }
    return count;
  }, { x: projected.x, y: projected.y });
}

test('revolute motors remain anchored to their frame origins while DH pose and d changes remain correct', async ({ page, request }, info) => {
  const specs = [
    ['R', 0, 90, 62, 4], ['R', 0, -90, -14.58, 2], ['P', 0, -90, -90, 5.2132],
    ['R', 2, 0, -49.89, 2], ['R', 3, 0, -18.42, 0],
  ];
  const joints = specs.map(([type, a, alpha, theta, d], i) => ({
    id: i + 1, type, a, alpha, theta, d, val: type === 'R' ? theta : d,
    min: type === 'R' ? -180 : 0, max: type === 'R' ? 180 : 12, speed: 0,
  }));
  const name = `shoulder-${info.project.name}`;
  const response = await request.post('/api/simulations', { data: { name, data: joints } });
  expect(response.status()).toBe(201);
  const { id } = await response.json();
  try {
    await page.goto('/');
    await page.getByRole('button', { name: 'Projetos', exact: true }).click();
    await page.getByRole('button', { name: `Carregar ${name}`, exact: true }).click();
    await page.getByRole('button', { name: 'Modelo', exact: true }).click();
    await expect(page.getByRole('spinbutton', { name: 'd2', exact: true })).toHaveValue('2');
    await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
    await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
    await page.getByRole('checkbox', { name: 'Referenciais', exact: true }).uncheck();
    await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
    for (const [d, angle] of [[2, -14.58], [-2, 40], [0, -20]]) {
      await number(page, 'd2', d);
      await number(page, 'q2 (°)', angle);
      joints[1] = { ...joints[1], d, val: angle };
      await page.getByRole('button', { name: 'Enquadrar manipulador' }).click();
      await expect.poll(() => motorPixels(page, joints, [0, 0, 4])).toBeGreaterThan(3);
      await expect.poll(() => motorPixels(page, joints, [0, 0, 0])).toBeGreaterThan(3);
      const [x, y, z] = forwardKinematics(joints).positions.at(-1).map(v => v.toFixed(2));
      await expect(page.getByTestId('end-position')).toHaveText(`X ${x} · Y ${y} · Z ${z} u`);
      await page.locator('.canvas-container').screenshot({ path: info.outputPath(`shoulder-${d}.png`) });
    }
  } finally {
    await request.delete(`/api/simulations/${id}`);
  }
});

test('preset selection, 3D workspace calculation, and code export work seamlessly', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();

  // Test loading PRP Espacial preset (P2 Question 3)
  await pickOption(page, 'Selecionar preset industrial', 'PRP Espacial (P2 / 3-DOF) (3 DOF)');
  await expect(page.locator('.notice[role="status"]')).toContainText('Robô PRP Espacial (P2 / 3-DOF) carregado.');
  await expect(page.locator('.scene-brand')).toContainText('Manipulador PRP · 3 juntas');
  await expect(page.getByTestId('end-position')).toContainText('X 3.46 · Y 2.00 · Z 2.50 u');

  // Test loading SCARA preset
  await pickOption(page, 'Selecionar preset industrial', 'SCARA (RRPR) (4 DOF)');
  await expect(page.locator('.notice[role="status"]')).toContainText('Robô SCARA (RRPR) carregado.');
  await expect(page.locator('.scene-brand')).toContainText('4 juntas');

  // Test 3D Workspace tab and calculation (compact popover)
  await page.getByRole('button', { name: 'Alcance 3D' }).click();
  await expect(page.getByRole('heading', { name: 'Espaço de Trabalho 3D (Monte Carlo)' })).toBeVisible();
  const workspaceBox = await page.locator('.scene-popover').boundingBox();
  await page.getByRole('button', { name: 'Calcular alcance' }).click();
  await expect(page.getByRole('heading', { name: 'Estatísticas do Envelope' })).toBeVisible();
  await expect(page.locator('.readings')).toContainText('Volume envolvente');
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
  await expect(page.locator('.scene-layers-dropup').getByRole('checkbox', { name: 'Nuvem 3D' })).toBeVisible();
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();

  // Test Code Export tab (wider popover on desktop)
  await page.getByRole('button', { name: 'Código' }).click();
  await expect(page.getByRole('heading', { name: 'Exportação de Código de Controle' })).toBeVisible();
  const exportBox = await page.locator('.scene-popover').boundingBox();
  if ((await page.evaluate(() => window.innerWidth)) > 800) {
    expect(exportBox.width).toBeGreaterThan(workspaceBox.width);
  }
  await expect(page.locator('.code-block')).toContainText('forwardKinematics');
  await page.getByRole('button', { name: 'Python / NumPy' }).click();
  await expect(page.locator('.code-block')).toContainText('def forward_kinematics');
  await page.getByRole('button', { name: 'ROS 2 (Node)' }).click();
  await expect(page.locator('.code-block')).toContainText('class RobotKinematicsNode');
});

test('trajectory sequencer, playback timeline, and 3D path trail operate correctly', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();

  // Open Trajectory tab
  await page.getByRole('button', { name: 'Trajetória' }).click();
  await expect(page.getByRole('heading', { name: 'Sequenciador de Trajetórias' })).toBeVisible();

  // Verify default waypoints exist
  await expect(page.locator('.waypoint-item')).toHaveCount(3);
  await expect(page.locator('.scene-popover').getByRole('checkbox', { name: 'Trilha 3D' })).toBeChecked();

  // Test adding a new waypoint
  await page.getByRole('button', { name: 'Adicionar pose atual' }).click();
  await expect(page.locator('.waypoint-item')).toHaveCount(4);

  // Test timeline playback
  const playBtn = page.getByRole('button', { name: 'Iniciar reprodução' });
  await playBtn.click();
  await expect(page.getByRole('button', { name: 'Pausar reprodução' })).toBeVisible();

  // Wait briefly for timeline to advance
  await page.waitForTimeout(400);

  // Pause playback
  await page.getByRole('button', { name: 'Pausar reprodução' }).click();
  await expect(page.getByRole('button', { name: 'Iniciar reprodução' })).toBeVisible();

  // Test stopping and seeking to zero
  await page.getByRole('button', { name: 'Parar e reiniciar timeline' }).click();
  await expect(page.locator('.timeline-slider-row span').first()).toHaveText('0.00 s');

  // Verify kinematic profile charts, metric switching, joint toggling, and chart playhead scrubbing
  const chart = page.getByTestId('trajectory-kinematic-chart');
  await expect(chart).toBeVisible();
  await expect(chart.locator('[data-series-key="j-0"]')).toBeVisible();
  await expect(chart.locator('[data-series-key="j-1"]')).toBeVisible();

  await page.getByRole('button', { name: 'Velocidade q̇(t)', exact: true }).click();
  await expect(page.locator('.chart-stats')).toContainText('Pico |q̇| (R)');

  await page.getByRole('button', { name: 'Aceleração q̈(t)', exact: true }).click();
  await expect(page.locator('.chart-stats')).toContainText('Pico |q̈| (R)');

  await page.getByRole('button', { name: 'Vel. TCP |v|', exact: true }).click();
  await expect(chart.locator('[data-series-key="tcp"]')).toBeVisible();

  await page.getByRole('button', { name: 'Posição q(t)', exact: true }).click();
  await page.getByRole('button', { name: /J1 \(R\)/ }).click();
  await expect(chart.locator('[data-series-key="j-0"]')).toHaveCount(0);
  await page.getByRole('button', { name: /J1 \(R\)/ }).click();
  await expect(chart.locator('[data-series-key="j-0"]')).toBeVisible();

  // Clicking near the middle of the SVG chart seeks playTime smoothly away from 0.00 s
  const box = await chart.boundingBox();
  expect(box).not.toBeNull();
  await page.mouse.click(box.x + box.width * 0.55, box.y + box.height * 0.5);
  await expect(page.locator('.timeline-slider-row span').first()).not.toHaveText('0.00 s');

  // Switch trajectory interpolation mode to Trapezoidal (LSPB)
  await pickOption(page, 'Tipo de interpolação', 'Trapezoidal — LSPB (aceleração e cruzeiro constantes)');
  await expect(page.locator('.chart-interp-note')).toContainText('Trapezoidal (LSPB)');

  // Toggle angular unit between degrees and radians
  await page.getByRole('button', { name: 'Radianos (rad)', exact: true }).click();
  await expect(page.locator('.chart-stats')).toContainText('rad/s');
  await expect(page.locator('.chart-stats')).toContainText('rad/s²');
  await page.getByRole('button', { name: 'Graus (°)', exact: true }).click();
  await expect(page.locator('.chart-stats')).toContainText('°/s');
});

test('public link sharing and academic report generation operate properly', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();

  // Test public link sharing
  await page.getByRole('button', { name: 'Compartilhar link público' }).click();
  const shareInput = page.getByRole('textbox', { name: 'Link de compartilhamento' });
  await expect(shareInput).toBeVisible();
  const shareUrl = await shareInput.inputValue();
  expect(shareUrl).toContain('?share=rob-');

  // Test academic report tab
  await page.getByRole('button', { name: 'Relatório' }).click();
  await expect(page.getByRole('heading', { name: 'Relatório Acadêmico (PDF / Impressão)' })).toBeVisible();
  await expect(page.getByRole('button', { name: 'Gerar e Imprimir Relatório (PDF)' })).toBeVisible();
  await expect(page.locator('.readings')).toContainText('Posto do Jacobiano');
});

test('URDF export/import workflows and STL CAD tool attachments work properly', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();

  // Test URDF code export tab
  await page.getByRole('button', { name: 'Código' }).click();
  await page.getByRole('button', { name: 'URDF (ROS)' }).click();
  await expect(page.locator('.code-block')).toContainText('<robot name="manipulador_');
  await expect(page.getByRole('button', { name: 'Baixar .urdf' })).toBeVisible();

  // Test URDF file upload into simulator
  await page.getByRole('button', { name: 'Projetos' }).click();
  const urdfXml = `<?xml version="1.0"?>
<robot name="test_arm">
  <link name="base_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="l1"/>
    <origin xyz="1.5 0 2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  <link name="l1"/>
</robot>`;
  await page.locator('input[aria-label="Upload de arquivo URDF"]').setInputFiles({
    name: 'test_arm.urdf',
    mimeType: 'application/xml',
    buffer: Buffer.from(urdfXml, 'utf-8'),
  });

  await expect(page.locator('.notice[role="status"]')).toContainText('Robô importado via URDF');
  await expect(page.locator('.scene-brand')).toContainText('1 juntas');

  // Verify STL CAD upload button exists inside Layers drop-up
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
  await expect(page.locator('label[aria-label="Carregar malha STL"]')).toBeVisible();
});

test('joint control interval limits auto-clamp positions, update IK slider bounds, and constrain solver', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Didático RPR (3-DOF) (3 DOF)');
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();

  // Model tab: tightening max below current q2 (5 u) clamps q2 automatically to 3
  await number(page, 'Máx. 2 (u)', 3);
  await expect(page.locator('.notice[role="status"]')).toContainText('posição ajustada para J2 (3.00 u)');
  await expect(page.getByRole('spinbutton', { name: 'q2 (u)', exact: true })).toHaveValue('3');

  // Switch to Inverse tab and inspect joint control limits + Cartesian slider bounds
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();
  await expect(page.getByRole('heading', { name: 'Limites de controle das juntas (R / P)' })).toBeVisible();
  await expect(page.locator('.ik-joint-limit-card').nth(1)).toContainText('No limite');
  const xSlider = page.getByRole('slider', { name: 'Deslizar Alvo X (u)', exact: true });
  expect(Number(await xSlider.getAttribute('max'))).toBeLessThan(8.2);
  expect(Number(await xSlider.getAttribute('max'))).toBeGreaterThan(7.2);

  // Restrict rotational joint 1 interval in Inversa and verify Cartesian Y bound tightens too
  await number(page, 'Limite mín. J1 (°)', -100);
  await number(page, 'Limite máx. J1 (°)', 100);
  const ySlider = page.getByRole('slider', { name: 'Deslizar Alvo Y (u)', exact: true });
  expect(Number(await ySlider.getAttribute('max'))).toBeLessThan(7);

  // Solve a reachable target within constrained bounds and verify joints obey [min, max]
  await number(page, 'Alvo X (u)', 7.5);
  await number(page, 'Alvo Y (u)', 0);
  await page.getByRole('button', { name: 'Resolver', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Solução convergiu');
  await expect(page.getByTestId('end-position')).toContainText('X 7.50 · Y 0.00 · Z 0.00');
  expect(Number(await xSlider.getAttribute('max'))).toBeCloseTo(8, 1);
  const q1Text = await page.locator('.ik-joint-limit-card').nth(0).locator('output').textContent();
  const q2Text = await page.locator('.ik-joint-limit-card').nth(1).locator('output').textContent();
  const q1Val = Number(q1Text.match(/=\s*(-?[\d.]+)/)[1]);
  const q2Val = Number(q2Text.match(/=\s*(-?[\d.]+)/)[1]);
  expect(q1Val).toBeGreaterThanOrEqual(-100);
  expect(q1Val).toBeLessThanOrEqual(100);
  expect(q2Val).toBeGreaterThanOrEqual(0);
  expect(q2Val).toBeLessThanOrEqual(3);

  // Target outside constrained prismatic max reach (8 u) must not converge or alter valid q2
  await number(page, 'Alvo X (u)', 12);
  await number(page, 'Alvo Y (u)', 0);
  await page.getByRole('button', { name: 'Resolver', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Não convergiu');
  await expect(page.getByTestId('end-position')).toContainText('X 7.50 · Y 0.00 · Z 0.00');
  await expect(page.locator('.ik-joint-limit-card').nth(1).locator('output')).toContainText(`q2 = ${q2Val.toFixed(2)} u`);
});

test('self-collision prevention blocks tunneling across links and notifies user', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Didático RPR (3-DOF) (3 DOF)');
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();

  // Attempt to jump q3 from 30 deg across 90 deg (link fold) to 120 deg
  await number(page, 'q3 (°)', 120);
  await expect(page.locator('.notice[role="status"]')).toContainText('Autocolisão detectada');
  await expect(page.locator('.collision-readout-pill')).toContainText('Bloqueio de colisão');
  const q3Val = Number(await page.getByRole('spinbutton', { name: 'q3 (°)', exact: true }).inputValue());
  expect(q3Val).toBeGreaterThan(80);
  expect(q3Val).toBeLessThan(83);

  // Pulling q3 back to a safe angle (20 deg) clears the collision warning
  await number(page, 'q3 (°)', 20);
  await expect(page.locator('.collision-readout-pill')).toHaveCount(0);
  await expect(page.getByRole('spinbutton', { name: 'q3 (°)', exact: true })).toHaveValue('20');
});

test('sliding UR5 target sliders to extremes keeps transverse fixed coordinates locked', async ({ page }) => {
  await page.goto('/');
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Universal Cobot UR5 (6R) (6 DOF)');
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();

  await expect(page.locator('[data-panel-id="inverse"] [data-testid="slider-singularity-dot"]')).toHaveCount(0);

  const xInput = page.getByRole('spinbutton', { name: 'Alvo X (u)', exact: true });
  const yInput = page.getByRole('spinbutton', { name: 'Alvo Y (u)', exact: true });
  const zInput = page.getByRole('spinbutton', { name: 'Alvo Z (u)', exact: true });
  const xSlider = page.getByRole('slider', { name: 'Deslizar Alvo X (u)', exact: true });
  const zSlider = page.getByRole('slider', { name: 'Deslizar Alvo Z (u)', exact: true });

  // Continuous pointer drag across Alvo X slider (including interior singular zone) must remain smooth and never snap back
  await xSlider.scrollIntoViewIfNeeded();
  const sliderBox = await xSlider.boundingBox();
  if (sliderBox && sliderBox.width > 80) {
    const midY = sliderBox.y + sliderBox.height / 2;
    await page.mouse.move(sliderBox.x + sliderBox.width * 0.5, midY);
    await page.mouse.down();
    await page.mouse.move(sliderBox.x + sliderBox.width * 0.2, midY, { steps: 10 });
    const leftVal = Number(await xInput.inputValue());
    await page.mouse.move(sliderBox.x + sliderBox.width * 0.85, midY, { steps: 14 });
    const rightVal = Number(await xInput.inputValue());
    await page.mouse.up();
    expect(rightVal).toBeGreaterThan(leftVal + 4.0);
  }

  const initialY = await yInput.inputValue();
  const initialZ = await zInput.inputValue();

  // Jam X slider to min and max extremes: fixed Y and Z must remain invariant
  const xMin = await xSlider.getAttribute('min');
  await xSlider.fill(String(xMin));
  expect(Number(await yInput.inputValue())).toBeCloseTo(Number(initialY), 4);
  expect(Number(await zInput.inputValue())).toBeCloseTo(Number(initialZ), 4);

  const xMax = await xSlider.getAttribute('max');
  await xSlider.fill(String(xMax));
  expect(Number(await yInput.inputValue())).toBeCloseTo(Number(initialY), 4);
  expect(Number(await zInput.inputValue())).toBeCloseTo(Number(initialZ), 4);

  // Move Z slider to its max extreme: fixed X and Y must remain invariant
  const fixedX = await xInput.inputValue();
  const fixedY = await yInput.inputValue();
  const zMax = await zSlider.getAttribute('max');
  await zSlider.fill(String(zMax));
  expect(Number(await xInput.inputValue())).toBeCloseTo(Number(fixedX), 4);
  expect(Number(await yInput.inputValue())).toBeCloseTo(Number(fixedY), 4);
});

test('customizable industrial tool selector and always-available tool orientation work end-to-end', async ({ page }) => {
  await page.goto('/');
  await page.getByRole('button', { name: /Ferramenta/i }).click();
  await expect(page.getByRole('heading', { name: 'Ferramenta do efetuador (TCP)' })).toBeVisible();

  // Switch between the 3 industrial tool types via top toolbar popover (works across tabs)
  await page.getByRole('button', { name: 'Ventosa a Vácuo', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Ventosa a Vácuo');
  await page.getByRole('button', { name: 'Tocha de Solda', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Tocha de Solda');

  // Switch to UR5 6-DOF robot and Inverse tab while confirming popover can close cleanly
  await page.getByRole('button', { name: 'Fechar painel', exact: true }).click();
  await expect(page.getByRole('heading', { name: 'Ferramenta do efetuador (TCP)' })).toHaveCount(0);
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Universal Cobot UR5 (6R) (6 DOF)');
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();

  // Orientation controls are visible immediately even when starting in 'position' mode
  const pitchSlider = page.getByRole('slider', { name: 'Deslizar Pitch alvo (°)', exact: true });
  await expect(pitchSlider).toBeVisible();
  await expect(page.getByRole('combobox', { name: 'Tarefa', exact: true })).toContainText('Posição XYZ');

  // Using an orientation preset automatically promotes task mode to Pose XYZ + orientação and solves cleanly
  await page.getByRole('button', { name: 'Vertical -90°', exact: true }).click();
  await expect(page.getByRole('combobox', { name: 'Tarefa', exact: true })).toContainText('Pose XYZ + orientação');
  await expect(page.locator('.notice[role="status"]')).toContainText('Solução convergiu');
  await expect(page.locator('.scene-readout')).toContainText('RPY [0.0°, -90.0°, 0.0°]');
});

test('interactive 3D target gizmo toggles translate and rotate modes cleanly without console errors', async ({ page }) => {
  const errors = [];
  page.on('pageerror', err => errors.push(err.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  const toolbarGizmoToggle = page.getByRole('checkbox', { name: 'Gizmo 3D', exact: true });
  const toolbarGizmoModeBtn = page.getByRole('button', { name: 'Alternar modo do Gizmo 3D', exact: true });

  // Available immediately outside Inverse tab cleanly from top CAD toolbar
  await expect(toolbarGizmoToggle).toBeChecked();
  await expect(toolbarGizmoModeBtn).toHaveText('XYZ');
  const lockOriBtn = page.getByRole('button', { name: 'Travar orientação RPY ao mover XYZ', exact: true });
  await expect(lockOriBtn).toHaveAttribute('aria-pressed', 'false');
  await lockOriBtn.click();
  await expect(lockOriBtn).toHaveAttribute('aria-pressed', 'true');
  await lockOriBtn.click();

  await toolbarGizmoModeBtn.click();
  await expect(toolbarGizmoModeBtn).toHaveText('RPY');
  const lockPosBtn = page.getByRole('button', { name: 'Travar posição XYZ ao girar RPY', exact: true });
  await expect(lockPosBtn).toHaveAttribute('aria-pressed', 'true');
  await lockPosBtn.click();
  await expect(lockPosBtn).toHaveAttribute('aria-pressed', 'false');
  await lockPosBtn.click();

  await toolbarGizmoModeBtn.click();
  await expect(toolbarGizmoModeBtn).toHaveText('XYZ');

  const outsideInverseBefore = await pixels(page);
  await toolbarGizmoToggle.uncheck();
  await expect(toolbarGizmoModeBtn).toHaveCount(0);
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(outsideInverseBefore.checksum);
  await toolbarGizmoToggle.check();
  await expect(toolbarGizmoModeBtn).toHaveText('XYZ');

  await page.getByRole('button', { name: 'Inversa', exact: true }).click();
  const sidebarGizmoToggle = page.getByRole('checkbox', { name: 'Gizmo 3D interativo', exact: true });
  await expect(sidebarGizmoToggle).toBeChecked();
  await expect(toolbarGizmoToggle).toBeChecked();
  await expect(page.getByRole('button', { name: 'Mover no 3D (XYZ)', exact: true })).toHaveAttribute('aria-current', 'page');

  const beforeGizmoToggle = await pixels(page);
  await toolbarGizmoToggle.uncheck();
  await expect(sidebarGizmoToggle).not.toBeChecked();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(beforeGizmoToggle.checksum);

  await sidebarGizmoToggle.check();
  await expect(toolbarGizmoToggle).toBeChecked();
  await page.getByRole('button', { name: 'Girar no 3D (RPY)', exact: true }).click();
  await expect(page.getByRole('button', { name: 'Girar no 3D (RPY)', exact: true })).toHaveAttribute('aria-current', 'page');
  await expect(toolbarGizmoModeBtn).toHaveText('RPY');
  await expect(page.getByRole('combobox', { name: 'Tarefa', exact: true })).toContainText('Pose XYZ + orientação');

  // Collapsing Inversa drawer preserves active IK context & 3D Gizmo on canvas
  await page.keyboard.press('Escape');
  await expect(page.locator('.scene-popover')).toHaveCount(0);
  await expect(toolbarGizmoToggle).toBeChecked();

  expect(errors).toEqual([]);
});

test('environment collision detects elevated workbench floor and blocks motion through 3D obstacles', async ({ page }) => {
  const errors = [];
  page.on('pageerror', err => errors.push(err.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Didático RPR (3-DOF) (3 DOF)');
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await expect(page.getByRole('heading', { name: 'Ambiente e Colisão Externa (Piso / Obstáculo 3D)' })).toBeVisible();

  // Position a 1.0u sphere obstacle ahead along the RPR sweep path at [4, 3, 0]
  await page.getByRole('button', { name: 'Posicionar à frente do efetuador', exact: true }).click();
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
  await expect(page.locator('.scene-layers-dropup').getByRole('checkbox', { name: 'Obstáculo 3D', exact: true })).toBeChecked();
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
  const obsGizmoToggle = page.getByRole('checkbox', { name: 'Mover obstáculo no 3D (Gizmo)', exact: true });
  await expect(obsGizmoToggle).toBeChecked();
  await obsGizmoToggle.uncheck();
  await obsGizmoToggle.check();
  await number(page, 'Obstáculo X (u)', 4);
  await number(page, 'Obstáculo Y (u)', 3);
  await number(page, 'Obstáculo Z (u)', 0);
  await number(page, 'Raio / Tamanho (u)', 1);
  await expect(page.locator('.collision-readout-pill')).toHaveCount(0);

  // Trying to swing q1 across [4, 3, 0] gets blocked safely at the obstacle boundary
  await number(page, 'q1 (°)', 110);
  await expect(page.locator('.notice[role="status"]')).toContainText('Obstáculo 3D');
  await expect(page.locator('.collision-readout-pill')).toContainText('Obstáculo 3D');
  const stoppedQ1 = Number(await page.getByRole('spinbutton', { name: 'q1 (°)', exact: true }).inputValue());
  expect(stoppedQ1).toBeGreaterThan(45);
  expect(stoppedQ1).toBeLessThan(85);

  // Switch shape to Box and then reset environment
  await page.getByRole('button', { name: 'Caixa 3D (Cubo)', exact: true }).click();
  await page.getByRole('button', { name: 'Restaurar ambiente padrão', exact: true }).click();
  await expect(page.locator('.collision-readout-pill')).toHaveCount(0);

  // Raising workbench floor Z to 0.5 u detects floor collision with planar RPR links at Z = 0
  await number(page, 'Nível do piso Z (u)', 0.5);
  await expect(page.locator('.collision-readout-pill')).toContainText('piso/bancada');
  await page.getByRole('button', { name: 'Restaurar ambiente padrão', exact: true }).click();
  await expect(page.locator('.collision-readout-pill')).toHaveCount(0);

  expect(errors).toEqual([]);
});

test('tool configuration (TCP) synchronizes across exported code, URDF, report summary, and SQLite save/load', async ({ page }, info) => {
  await page.goto('/');
  await page.getByRole('button', { name: /Ferramenta/i }).click();
  await page.getByRole('button', { name: 'Tocha de Solda', exact: true }).click();
  await number(page, 'Volume da ferramenta (escala)', 2.2);
  await page.getByRole('slider', { name: 'Deslizar Offset Yaw (°)', exact: true }).fill('30');
  await expect(page.getByRole('spinbutton', { name: 'Offset Yaw (°)', exact: true })).toHaveValue('30');
  await page.getByRole('slider', { name: 'Deslizar Offset Yaw (°)', exact: true }).fill('0');
  await number(page, 'Offset Pitch (°)', -45);

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  const simName = `tcp-sync-${info.project.name}`;
  await page.getByRole('textbox', { name: 'Nome da simulação' }).fill(simName);
  await page.getByRole('button', { name: 'Salvar', exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toBeVisible();

  // Exported C++, Python, ROS2, and URDF reflect welder tool and offset
  await page.getByRole('button', { name: 'Código', exact: true }).click();
  await expect(page.locator('.code-block')).toContainText('TOOL_TYPE = "welder"');
  await expect(page.locator('.code-block')).toContainText('{0f, -45f, 0f}');
  await page.getByRole('button', { name: 'URDF (ROS)', exact: true }).click();
  await expect(page.locator('.code-block')).toContainText('<tool_config type="welder" offset_rpy="0 -45 0"/>');
  await expect(page.locator('.code-block')).toContainText('<link name="tcp_link"/>');

  // Academic report summary shows active tool and TCP coordinates
  await page.getByRole('button', { name: 'Relatório', exact: true }).click();
  await expect(page.locator('.readings')).toContainText('Tocha de Solda');
  await expect(page.locator('.readings')).toContainText('Posição do TCP (u)');

  // Reset in Ferramenta popover, reload in Projetos popover, and verify restoration
  await page.getByRole('button', { name: /Ferramenta/i }).click();
  await page.getByRole('button', { name: 'Restaurar ferramenta padrão', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Garra Paralela');

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await page.getByRole('button', { name: `Carregar ${simName}`, exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Tocha de Solda');

  await page.getByRole('button', { name: /Ferramenta/i }).click();
  await expect(page.getByRole('spinbutton', { name: 'Volume da ferramenta (escala)', exact: true })).toHaveValue('2.2');
  await expect(page.getByRole('spinbutton', { name: 'Offset Pitch (°)', exact: true })).toHaveValue('-45');

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  page.once('dialog', dialog => dialog.accept());
  await page.getByRole('button', { name: `Apagar ${simName}`, exact: true }).click();
});

test('custom 3D robot modeling (per-link, per-axis, and global sizes) updates WebGL scene, URDF, and SQLite persistence', async ({ page }, info) => {
  await page.goto('/');
  await expect(page.locator('.scene-brand')).toContainText('Manipulador RRRRRR · 6 juntas');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);
  const beforeRender = await pixels(page);

  // Open floating 3D modeling popover from slim top toolbar and verify bounds + no horizontal overflow
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  const popover = page.locator('.scene-popover');
  await expect(popover).toBeVisible();
  const boundsCheck = await page.evaluate(() => {
    const canvas = document.querySelector('.canvas-container')?.getBoundingClientRect();
    const pop = document.querySelector('.scene-popover')?.getBoundingClientRect();
    return {
      noDocOverflow: document.documentElement.scrollWidth <= window.innerWidth,
      fitsCanvas: Boolean(canvas && pop && pop.left >= canvas.left - 1 && pop.right <= canvas.right + 1 && pop.bottom <= canvas.bottom + 1),
    };
  });
  expect(boundsCheck.noDocOverflow).toBe(true);
  expect(boundsCheck.fitsCanvas).toBe(true);

  // Test floating window controls: drag header, resize corner, dock-left, and fullscreen
  const wsBox = await page.locator('.scene-workspace-row').boundingBox();
  const initialPopBox = await popover.boundingBox();
  expect(wsBox).not.toBeNull();
  expect(initialPopBox).not.toBeNull();
  if (wsBox.width > initialPopBox.width + 90) {
    const headerBox = await popover.locator('.scene-popover-header').boundingBox();
    await page.mouse.move(headerBox.x + 24, headerBox.y + headerBox.height / 2);
    await page.mouse.down();
    await page.mouse.move(headerBox.x + 104, headerBox.y + headerBox.height / 2 + 24, { steps: 5 });
    await page.mouse.up();
    const movedBox = await popover.boundingBox();
    expect(movedBox.x).toBeGreaterThan(initialPopBox.x + 40);
    expect(movedBox.y).toBeGreaterThan(initialPopBox.y + 10);

    const resizeHandle = page.getByTestId('popover-resize-se');
    const resizeBox = await resizeHandle.boundingBox();
    await page.mouse.move(resizeBox.x + resizeBox.width / 2, resizeBox.y + resizeBox.height / 2);
    await page.mouse.down();
    await page.mouse.move(resizeBox.x + 64, resizeBox.y + 44, { steps: 5 });
    await page.mouse.up();
    const resizedBox = await popover.boundingBox();
    expect(resizedBox.width).toBeGreaterThan(movedBox.width + 30);
    expect(resizedBox.height).toBeGreaterThan(movedBox.height + 20);
  }

  await page.getByRole('button', { name: 'Fixar painel na lateral', exact: true }).click();
  await expect(popover).toHaveClass(/scene-popover--dock-left/);
  await page.getByRole('button', { name: 'Tela cheia do painel', exact: true }).click();
  await expect(popover).toHaveClass(/scene-popover--fullscreen/);
  const fullCheck = await page.evaluate(() => {
    const ws = document.querySelector('.scene-workspace-row')?.getBoundingClientRect();
    const pop = document.querySelector('.scene-popover')?.getBoundingClientRect();
    return {
      noDocOverflow: document.documentElement.scrollWidth <= window.innerWidth,
      fillsWidth: Boolean(ws && pop && Math.abs(pop.width - ws.width) < 4),
      fillsHeight: Boolean(ws && pop && Math.abs(pop.height - ws.height) < 4),
    };
  });
  expect(fullCheck.noDocOverflow).toBe(true);
  expect(fullCheck.fillsWidth).toBe(true);
  expect(fullCheck.fillsHeight).toBe(true);
  await pickOption(page, 'Alvo da personalização 3D', 'Padrão global (todos os elos e eixos)');
  await page.getByRole('button', { name: 'Tela cheia do painel', exact: true }).click();
  await expect(popover).toHaveClass(/scene-popover--floating/);

  // Customize Axis/Link 1 separately on default Cobot UR5 (Revolute joint only)
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 1 (R)');
  await expect(page.getByRole('spinbutton', { name: 'Largura da junta prismática P (u)', exact: true })).toHaveCount(0);
  await page.getByRole('button', { name: 'Quadrada', exact: true }).click();
  await number(page, 'Raio das hastes (u)', 0.22);
  await number(page, 'Raio da junta rotativa R (u)', 0.58);
  await number(page, 'Altura da junta rotativa R (u)', 0.72);

  // Customize Axis/Link 3 separately without affecting Axis 1 or global defaults
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 3 (R)');
  await page.getByRole('button', { name: 'Hexagonal', exact: true }).click();
  await number(page, 'Raio das hastes (u)', 0.16);

  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(beforeRender.pink);
  await expect.poll(async () => (await pixels(page)).green).toBeGreaterThan(beforeRender.green);

  // Switch to Direta wide popover and close cleanly with Escape
  await page.getByRole('button', { name: 'Direta', exact: true }).click();
  await expect(popover).toBeVisible();
  await page.keyboard.press('Escape');
  await expect(popover).toHaveCount(0);

  // Verify URDF exports custom per-joint <joint_style> tags and box link geometry
  await page.getByRole('button', { name: 'Código', exact: true }).click();
  await page.getByRole('button', { name: 'URDF (ROS)', exact: true }).click();
  await expect(page.locator('.code-block')).toContainText('<joint_style id="1" link_shape="square" link_radius="0.22" rev_radius="0.58" rev_length="0.72"/>');
  await expect(page.locator('.code-block')).toContainText('<joint_style id="3" link_shape="hex" link_radius="0.16"/>');
  await expect(page.locator('.code-block')).toContainText('<box size="0.44 0.44');

  // Verify SQLite save + reset + reload restores perJoint overrides via Projetos guide
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  const simName = `styled-cobot-${info.project.name}`;
  await page.getByRole('textbox', { name: 'Nome da simulação' }).fill(simName);
  await page.getByRole('button', { name: 'Salvar', exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toBeVisible();

  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await page.getByRole('button', { name: 'Restaurar modelagem padrão', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'Raio das hastes (u)', exact: true })).toHaveValue('0.065');

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await page.getByRole('button', { name: `Carregar ${simName}`, exact: true }).click();
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 1 (R) · Personalizado');
  await expect(page.getByRole('spinbutton', { name: 'Raio das hastes (u)', exact: true })).toHaveValue('0.22');
  await expect(page.getByRole('spinbutton', { name: 'Raio da junta rotativa R (u)', exact: true })).toHaveValue('0.58');

  // Clearing single-axis override falls back to global default without resetting Axis 3
  await page.getByRole('button', { name: 'Usar padrão global neste eixo', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'Raio das hastes (u)', exact: true })).toHaveValue('0.065');

  // Verify prismatic axis customization on RPR displays only link + P-joint controls
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'Didático RPR (3-DOF) (3 DOF)');
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 2 (P)');
  await expect(page.getByRole('spinbutton', { name: 'Raio da junta rotativa R (u)', exact: true })).toHaveCount(0);
  await number(page, 'Largura da junta prismática P (u)', 0.64);
  await expect(page.getByRole('spinbutton', { name: 'Largura da junta prismática P (u)', exact: true })).toHaveValue('0.64');

  // Switch back to Modelo and verify collapsible advanced controls keep core Atuadores visible
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await page.getByRole('button', { name: 'Recolher avançados', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'Nível do piso Z (u)', exact: true })).toHaveCount(0);
  await expect(page.getByRole('spinbutton', { name: 'q1 (°)', exact: true })).toBeVisible();
  await page.getByRole('button', { name: 'Expandir avançados', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'Nível do piso Z (u)', exact: true })).toBeVisible();

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  page.once('dialog', dialog => dialog.accept());
  await page.getByRole('button', { name: `Apagar ${simName}`, exact: true }).click();
});

test('CAD viewport shortcuts: +Ponto, Undo/Redo, camera views, ellipsoid/ghost layers, and JSON export/import', async ({ page }, info) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('.scene-brand')).toBeVisible();
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);

  // 1. + Ponto & Rodar/Pausar directly from bottom viewport dock with all drawers closed
  await page.getByRole('button', { name: 'Gravar ponto atual', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('salvo na trajetória');
  await page.getByRole('button', { name: 'Trajetória', exact: true }).click();
  await expect(page.locator('.waypoint-item')).toHaveCount(4);
  await page.getByRole('button', { name: 'Fechar painel', exact: true }).click();

  const beforePlay = await pixels(page);
  await page.getByRole('button', { name: 'Executar animação 3D', exact: true }).click();
  await expect(page.getByRole('button', { name: 'Pausar animação 3D', exact: true })).toBeVisible();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(beforePlay.checksum);
  await page.getByRole('button', { name: 'Pausar animação 3D', exact: true }).click();
  await expect(page.getByRole('button', { name: 'Executar animação 3D', exact: true })).toBeVisible();

  // 2. Undo / Redo via top bar buttons
  const undoBtn = page.getByRole('button', { name: 'Desfazer', exact: true });
  const redoBtn = page.getByRole('button', { name: 'Refazer', exact: true });
  await expect(undoBtn).toBeEnabled();

  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  const q1Input = page.getByRole('spinbutton', { name: 'q1 (°)', exact: true });
  const q1BeforeEdit = await q1Input.inputValue();
  await number(page, 'q1 (°)', 42);
  await expect(q1Input).toHaveValue('42');
  await undoBtn.click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Alteração desfeita.');
  await expect(q1Input).toHaveValue(q1BeforeEdit);
  await redoBtn.click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Alteração refeita.');
  await expect(q1Input).toHaveValue('42');
  await page.getByRole('button', { name: 'Fechar painel', exact: true }).click();

  // 3. CAD camera presets (XY, XZ, YZ, 3D) and 3D layers (Elipsoide 3D, Pose Fantasma)
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();
  const basePixels = await pixels(page);
  await page.getByRole('button', { name: 'Vista superior XY', exact: true }).click();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(basePixels.checksum);
  const topPixels = await pixels(page);
  await page.getByRole('button', { name: 'Vista frontal XZ', exact: true }).click();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(topPixels.checksum);
  await page.getByRole('button', { name: 'Vista lateral YZ', exact: true }).click();
  await page.getByRole('button', { name: 'Vista 3D isométrica', exact: true }).click();

  const beforeOverlay = await pixels(page);
  await page.locator('.scene-layers-dropup').getByRole('checkbox', { name: 'Elipsoide 3D', exact: true }).check();
  await page.locator('.scene-layers-dropup').getByRole('checkbox', { name: 'Pose Fantasma', exact: true }).check();
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(beforeOverlay.checksum);
  await page.getByRole('button', { name: 'Camadas da cena', exact: true }).click();

  // 4. JSON project export and import
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  const [download] = await Promise.all([
    page.waitForEvent('download'),
    page.getByRole('button', { name: 'Exportar projeto JSON', exact: true }).click(),
  ]);
  const jsonPath = info.outputPath('exported-project.json');
  await download.saveAs(jsonPath);
  await expect(page.locator('.notice[role="status"]')).toContainText('Projeto JSON exportado');

  await page.getByRole('button', { name: 'Restaurar Cobot UR5 padrão', exact: true }).click();
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'q1 (°)', exact: true })).toHaveValue('0');

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await page.getByLabel('Upload de projeto JSON', { exact: true }).setInputFiles(jsonPath);
  await expect(page.locator('.notice[role="status"]')).toContainText('importado com sucesso');
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'q1 (°)', exact: true })).toHaveValue('42');

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('custom joint colors in Modelagem 3D synchronize with Trajectory charts and actuator indicators', async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);

  const initialPixels = await pixels(page);
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 1 (R)');

  const jointColorInput = page.getByLabel('Cor da junta', { exact: true });
  await jointColorInput.fill('#ff5500');
  await expect(jointColorInput).toHaveValue('#ff5500');
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(initialPixels.checksum);

  // Modelo tab dot mirrors custom color
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  const j1Dot = page.locator('.actuator').first().locator('.joint-color-dot');
  await expect(j1Dot).toHaveCSS('background-color', 'rgb(255, 85, 0)');

  // Trajetória chart defaults J1 (#e65c93) matching 3D robot out of the box, and mirrors #ff5500 when customized
  await page.getByRole('button', { name: 'Trajetória', exact: true }).click();
  const j0Path = page.locator('[data-testid="trajectory-kinematic-chart"] path[data-series-key="j-0"]');
  await expect(j0Path).toHaveAttribute('stroke', '#ff5500');
  const firstChipSwatch = page.locator('.chart-legend-chip .chart-swatch').first();
  await expect(firstChipSwatch).toHaveCSS('background-color', 'rgb(255, 85, 0)');

  // One-click palette assignment colors J1..J6 consistently across 3D and chart
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await page.getByRole('button', { name: 'Colorir juntas pela paleta do gráfico', exact: true }).click();
  await page.getByRole('button', { name: 'Trajetória', exact: true }).click();
  await expect(j0Path).toHaveAttribute('stroke', '#e65c93');
  await expect(page.locator('[data-testid="trajectory-kinematic-chart"] path[data-series-key="j-1"]')).toHaveAttribute('stroke', '#4da6ff');

  // Restoring modeling resets 3D defaults and keeps J1 chart matching 3D (#e65c93)
  await page.getByRole('button', { name: /Modelagem 3D/i }).click();

  // Direct #HEX input, Hue rainbow slider, and expanded 24-color / 360-deg rainbow controls
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 1 (R) · Personalizado');
  const hexInput = page.getByLabel('Código hexadecimal da cor', { exact: true });
  await hexInput.fill('#18b4f0');
  await expect(jointColorInput).toHaveValue('#18b4f0');

  const hueSlider = page.getByRole('slider', { name: 'Matiz da cor (0 a 360 graus)', exact: true });
  await hueSlider.fill('120');
  await expect(hexInput).not.toHaveValue('#18b4f0');

  await page.getByRole('button', { name: 'Expandir seletor avançado de cores', exact: true }).click();
  await expect(page.getByRole('slider', { name: 'Saturação da cor', exact: true })).toBeVisible();
  await expect(page.getByRole('slider', { name: 'Luminosidade da cor', exact: true })).toBeVisible();
  await page.getByRole('button', { name: 'Distribuir arco-íris nas juntas', exact: true }).click();
  await page.getByRole('button', { name: 'Trajetória', exact: true }).click();
  await expect(j0Path).toHaveAttribute('stroke', hslToHex(0, 82, 62));
  await expect(page.locator('[data-testid="trajectory-kinematic-chart"] path[data-series-key="j-1"]')).toHaveAttribute('stroke', hslToHex(60, 82, 62));

  await page.getByRole('button', { name: /Modelagem 3D/i }).click();
  await page.getByRole('button', { name: 'Restaurar modelagem padrão', exact: true }).click();
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);
  await page.getByRole('button', { name: 'Trajetória', exact: true }).click();
  await expect(j0Path).toHaveAttribute('stroke', '#e65c93');

  expect(errors).toEqual([]);
});

test('multiple floating panels can be pinned or Ctrl-clicked simultaneously with side-by-side placement, live updates, and minimize', async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);

  // 1. Open Modelo and pin it
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  const modelPopover = page.locator('.scene-popover[data-panel-id="model"]');
  await expect(modelPopover).toBeVisible();
  await modelPopover.getByRole('button', { name: 'Manter janela aberta', exact: true }).click();

  // 2. Normal click on Direta keeps pinned Modelo open and places Direta on the right on desktop
  await page.getByRole('button', { name: 'Direta', exact: true }).click();
  const forwardPopover = page.locator('.scene-popover[data-panel-id="forward"]');
  await expect(forwardPopover).toBeVisible();
  await expect(page.locator('.scene-popover')).toHaveCount(2);

  const wsBox = await page.locator('.scene-workspace-row').boundingBox();
  if (wsBox.width > 960) {
    const modelBox = await modelPopover.boundingBox();
    const forwardBox = await forwardPopover.boundingBox();
    expect(forwardBox.x).toBeGreaterThan(modelBox.x + 150);
  }

  // 3. Editing q1 in Modelo updates Direta matrix live in parallel window
  const beforeEnd = await page.getByTestId('end-position').textContent();
  await number(page, 'q1 (°)', 30);
  await expect(page.getByTestId('end-position')).not.toHaveText(beforeEnd);
  await expect(forwardPopover).toContainText('Coordenadas globais (u)');

  // 4. Ctrl+Click opens a 3rd window (Diferencial) without closing the existing two, and computes Jacobian live
  await page.getByRole('button', { name: 'Diferencial', exact: true }).click({ modifiers: ['Control'] });
  const diffPopover = page.locator('.scene-popover[data-panel-id="differential"]');
  await expect(diffPopover).toBeVisible();
  await expect(page.locator('.scene-popover')).toHaveCount(3);
  await expect(diffPopover).toContainText('Condição normalizada');

  // 5. Minimize Diferencial collapses its body without closing it
  await diffPopover.getByRole('button', { name: 'Minimizar painel', exact: true }).click();
  await expect(diffPopover.locator('.scene-popover-body')).toHaveCount(0);
  await diffPopover.getByRole('button', { name: 'Fechar painel', exact: true }).click();
  await expect(page.locator('.scene-popover')).toHaveCount(2);

  // Escape closes only the focused panel, keeping mobile/desktop free of horizontal overflow
  await page.keyboard.press('Escape');
  await expect(page.locator('.scene-popover')).toHaveCount(1);
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('dedicated Singularidades tab displays Cartesian profiles, critical joint sweeps, and applies singular poses', async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);

  await page.getByRole('button', { name: 'Singularidades', exact: true }).click();
  const singPopover = page.locator('.scene-popover[data-panel-id="singularities"]');
  await expect(singPopover).toBeVisible();
  await expect(singPopover).toContainText('Estado atual e Gimbal Lock');
  await expect(singPopover).toContainText('Mapa por eixo cartesiano (X, Y, Z)');

  expect(await singPopover.locator('.singularity-axis-svg').count()).toBeGreaterThanOrEqual(3);
  expect(await singPopover.locator('.singularity-svg-band').count()).toBeGreaterThanOrEqual(1);
  expect(await singPopover.locator('.singularity-pose-item').count()).toBeGreaterThanOrEqual(1);

  const beforeEnd = await page.getByTestId('end-position').textContent();
  await singPopover.getByRole('button', { name: 'Aplicar pose singular 1', exact: true }).click();
  await expect(page.getByTestId('end-position')).not.toHaveText(beforeEnd);
  await expect(page.locator('.notice[role="status"]')).toContainText('Pose singular 1 aplicada');

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('analytical IK posture selector switches UR5 and SCARA branches cleanly while preserving TCP target', async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);

  await page.getByRole('button', { name: 'Inversa', exact: true }).click();
  const ikPopover = page.locator('.scene-popover[data-panel-id="inverse"]');
  await expect(ikPopover).toBeVisible();

  const postureSection = ikPopover.locator('.posture-section');
  await expect(postureSection).toContainText('Analítico · UR5 (6R)');
  await expect(postureSection).toContainText('Ramos sem colisão para este alvo (8)');

  const initialEnd = await page.getByTestId('end-position').textContent();
  await postureSection.getByRole('group', { name: 'Selecionar Cotovelo', exact: true })
    .getByRole('button', { name: 'Baixo', exact: true }).click();

  await expect(ikPopover).toContainText('Analítico fechado (UR5 (6R))');
  await expect(ikPopover).toContainText('Cotovelo: Baixo');
  await expect(page.getByTestId('end-position')).toHaveText(initialEnd);

  // Switch directly via a collision-free branch chip to Right shoulder + Up elbow + Inverted wrist
  const targetChip = postureSection.locator('.posture-branch-chip', {
    hasText: 'Ombro: Direito · Cotovelo: Cima · Punho: Invertido',
  });
  await targetChip.click();
  await expect(targetChip).toHaveAttribute('aria-pressed', 'true');
  await expect(page.getByTestId('end-position')).toHaveText(initialEnd);

  // Switch to SCARA (RRPR) and test both planar elbow posture branches
  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'SCARA (RRPR) (4 DOF)');
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();
  await expect(postureSection).toContainText('Analítico · SCARA (RRPR)');
  await expect(postureSection).toContainText('Ramos sem colisão para este alvo (2)');

  const scaraEnd = await page.getByTestId('end-position').textContent();
  await postureSection.getByRole('group', { name: 'Selecionar Cotovelo', exact: true })
    .getByRole('button', { name: 'Direita (-q₂)', exact: true }).click();
  await expect(ikPopover).toContainText('Analítico fechado (SCARA (RRPR))');
  await expect(ikPopover).toContainText('Cotovelo: Direita');
  await expect(page.getByTestId('end-position')).toHaveText(scaraEnd);

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('3D singularity highlights, per-link custom STL with URDF <mesh>, and medium-screen compact toolbar work end-to-end', async ({ page }, info) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);

  // 1. 3D Highlight direct from Singularidades tab
  await page.getByRole('button', { name: 'Singularidades', exact: true }).click();
  const singPopover = page.locator('.scene-popover[data-panel-id="singularities"]');
  await expect(singPopover).toBeVisible();
  const highlightCheck = singPopover.getByRole('checkbox', { name: 'Destaques 3D na cena' });
  await expect(highlightCheck).toBeChecked();

  const beforeHighlightPx = await pixels(page);
  const firstPoseCard = singPopover.locator('.singularity-pose-item').first();
  await firstPoseCard.hover();
  await expect(firstPoseCard).toHaveClass(/is-highlighted/);
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(beforeHighlightPx.checksum);

  await highlightCheck.uncheck();
  await expect(highlightCheck).not.toBeChecked();
  await highlightCheck.check();

  // 2. Per-link custom STL mesh in Modelagem 3D + <mesh> export in URDF
  await page.getByRole('button', { name: 'Modelagem 3D', exact: true }).click();
  const modelPopover = page.locator('.scene-popover[data-panel-id="modeling"]');
  await expect(modelPopover).toBeVisible();
  await pickOption(page, 'Alvo da personalização 3D', 'Eixo / Ligação 1 (R)');

  const stlContent = `solid custom_link
  facet normal 0 0 1
    outer loop
      vertex -0.5 -0.2 -0.2
      vertex 0.5 0.0 -0.2
      vertex 0.0 0.3 0.2
    endloop
  endfacet
  facet normal 0 1 0
    outer loop
      vertex -0.5 -0.2 0.2
      vertex 0.5 0.0 0.2
      vertex 0.0 -0.3 -0.2
    endloop
  endfacet
endsolid custom_link
`;
  await page.getByLabel('Upload STL do elo 1').setInputFiles({
    name: 'elo_1_cad.stl',
    mimeType: 'application/sla',
    buffer: Buffer.from(stlContent, 'utf8'),
  });
  await expect(modelPopover).toContainText('Ativa: elo_1_cad.stl');
  await expect(page.getByRole('button', { name: 'Modelagem 3D', exact: true })).toContainText('J1·STL');
  await number(page, 'Escala STL elo 1', 1.5);

  if (info.project.name === 'desktop') {
    // 3. Medium desktop compaction (1280px single row + 1080px overflow Mais menu)
    await page.setViewportSize({ width: 1280, height: 780 });
    const tb1280 = await page.locator('.scene-toolbar').boundingBox();
    expect(tb1280.height).toBeLessThanOrEqual(48);

    await page.setViewportSize({ width: 1080, height: 780 });
    const moreBtn = page.getByRole('button', { name: 'Mais', exact: true });
    await expect(moreBtn).toBeVisible();
    const tb1080 = await page.locator('.scene-toolbar').boundingBox();
    expect(tb1080.height).toBeLessThanOrEqual(48);

    await moreBtn.click();
    const moreMenu = page.locator('.scene-toolbar-more-menu');
    await expect(moreMenu).toBeVisible();
    await moreMenu.getByRole('menuitem', { name: 'Código', exact: true }).click();
  } else {
    await page.getByRole('button', { name: 'Código', exact: true }).click();
  }

  const exportPopover = page.locator('.scene-popover[data-panel-id="export"]');
  await expect(exportPopover).toBeVisible();
  await exportPopover.getByRole('button', { name: 'URDF (ROS)', exact: true }).click();
  await expect(exportPopover.locator('pre')).toContainText('meshes/elo_1_cad.stl');
  await expect(exportPopover.locator('pre')).toContainText('scale="1.5 1.5 1.5"');

  await page.getByRole('button', { name: 'Modelagem 3D', exact: true }).click();
  await page.getByRole('button', { name: 'Remover STL do elo 1', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Malha STL removida do elo 1');

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});

test('Web Worker computes 5000-point workspace cloud and asynchronous Singularidades maps cleanly across presets', async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', msg => { if (msg.type() === 'error') errors.push(msg.text()); });

  await page.goto('/');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);

  // 1. Async 5,000-point Monte Carlo workspace generation off main thread
  await page.getByRole('button', { name: 'Alcance 3D', exact: true }).click();
  const wsPopover = page.locator('.scene-popover[data-panel-id="workspace"]');
  await expect(wsPopover).toBeVisible();
  await pickOption(page, 'Número de amostras', '5.000 pontos (Alta densidade)');
  await wsPopover.getByRole('button', { name: 'Calcular alcance', exact: true }).click();
  await expect(wsPopover).toContainText('Estatísticas do Envelope', { timeout: 15000 });
  await expect(wsPopover).toContainText('Pontos amostrados');

  // 2. Async Singularidades map updates via worker across UR5 -> SCARA switch
  await page.getByRole('button', { name: 'Singularidades', exact: true }).click();
  const singPopover = page.locator('.scene-popover[data-panel-id="singularities"]');
  await expect(singPopover).toContainText('Mapa por eixo cartesiano (X, Y, Z)', { timeout: 15000 });
  expect(await singPopover.locator('.singularity-axis-svg').count()).toBeGreaterThanOrEqual(3);

  await page.getByRole('button', { name: 'Projetos', exact: true }).click();
  await pickOption(page, 'Selecionar preset industrial', 'SCARA (RRPR) (4 DOF)');
  await page.getByRole('button', { name: 'Singularidades', exact: true }).click();
  await expect(singPopover).toContainText('Mapa por eixo cartesiano (X, Y, Z)', { timeout: 15000 });
  await expect(singPopover).toContainText('Eixo Z');

  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
  expect(errors).toEqual([]);
});


