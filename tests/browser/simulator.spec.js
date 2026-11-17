import { test, expect } from '@playwright/test';
import { Box3, PerspectiveCamera, Vector3 } from 'three';
import { forwardKinematics } from '../../src/kinematics.js';

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
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();
  await expect(page.getByTestId('end-position')).toContainText('X 4.83 · Y 1.29 · Z 0.00');
  await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
  await expect.poll(async () => (await pixels(page)).pink).toBeGreaterThan(10);
  const before = await pixels(page);
  expect(before.green).toBeGreaterThan(10);
  expect(before.gold).toBeGreaterThan(5);
  await page.screenshot({ path: info.outputPath('model.png'), fullPage: true });

  await number(page, 'q1 (°)', 65);
  await expect(page.getByTestId('end-position')).not.toContainText('X 4.83');
  await expect.poll(async () => (await pixels(page)).checksum).not.toBe(before.checksum);
  await page.getByRole('button', { name: 'Restaurar RPR das aulas' }).click();
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

  await number(page, 'Mín. 1 (°)', 200);
  await expect(page.locator('.notice[role="status"]')).toContainText('mínimo deve ser menor');
  await expect(page.getByRole('spinbutton', { name: 'Mín. 1 (°)', exact: true })).toHaveValue('-180');

  const simName = `browser-${info.project.name}`;
  await page.getByRole('textbox', { name: 'Nome da simulação' }).fill(simName);
  await page.getByRole('button', { name: 'Salvar', exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toBeVisible();
  await number(page, 'q1 (°)', 10);
  await page.getByRole('button', { name: `Carregar ${simName}`, exact: true }).click();
  await expect(page.getByRole('spinbutton', { name: 'q1 (°)', exact: true })).toHaveValue('45');
  page.once('dialog', dialog => dialog.accept());
  await page.getByRole('button', { name: `Apagar ${simName}`, exact: true }).click();
  await expect(page.getByRole('button', { name: `Carregar ${simName}`, exact: true })).toHaveCount(0);

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
      if (data[i] > 110 && data[i] > data[i + 1] * 1.25 && data[i + 2] > data[i + 1] * 1.05) count++;
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
    await page.getByRole('button', { name: `Carregar ${name}`, exact: true }).click();
    await expect(page.getByRole('spinbutton', { name: 'd2', exact: true })).toHaveValue('2');
    await expect(page.locator('canvas')).toHaveAttribute('data-engine', /three.js/);
    await page.getByRole('checkbox', { name: 'Referenciais', exact: true }).uncheck();
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
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

  // Test loading SCARA preset
  await pickOption(page, 'Selecionar preset industrial', 'SCARA (RRPR) (4 DOF)');
  await expect(page.locator('.notice[role="status"]')).toContainText('Robô SCARA (RRPR) carregado.');
  await expect(page.locator('.sidebar')).toContainText('4 juntas');

  // Test 3D Workspace tab and calculation
  await page.getByRole('button', { name: 'Alcance 3D' }).click();
  await expect(page.getByRole('heading', { name: 'Espaço de Trabalho 3D (Monte Carlo)' })).toBeVisible();
  await page.getByRole('button', { name: 'Calcular alcance' }).click();
  await expect(page.getByRole('heading', { name: 'Estatísticas do Envelope' })).toBeVisible();
  await expect(page.locator('.readings')).toContainText('Volume envolvente');
  await expect(page.getByRole('checkbox', { name: 'Nuvem 3D' })).toBeVisible();

  // Test Code Export tab
  await page.getByRole('button', { name: 'Código' }).click();
  await expect(page.getByRole('heading', { name: 'Exportação de Código de Controle' })).toBeVisible();
  await expect(page.locator('.code-block')).toContainText('forwardKinematics');
  await page.getByRole('button', { name: 'Python / NumPy' }).click();
  await expect(page.locator('.code-block')).toContainText('def forward_kinematics');
  await page.getByRole('button', { name: 'ROS 2 (Node)' }).click();
  await expect(page.locator('.code-block')).toContainText('class RobotKinematicsNode');
});

test('trajectory sequencer, playback timeline, and 3D path trail operate correctly', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

  // Open Trajectory tab
  await page.getByRole('button', { name: 'Trajetória' }).click();
  await expect(page.getByRole('heading', { name: 'Sequenciador de Trajetórias' })).toBeVisible();

  // Verify default waypoints exist
  await expect(page.locator('.waypoint-item')).toHaveCount(3);
  await expect(page.locator('.sidebar').getByRole('checkbox', { name: 'Trilha 3D' })).toBeChecked();

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
});

test('public link sharing and academic report generation operate properly', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

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
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

  // Test URDF code export tab
  await page.getByRole('button', { name: 'Código' }).click();
  await page.getByRole('button', { name: 'URDF (ROS)' }).click();
  await expect(page.locator('.code-block')).toContainText('<robot name="manipulador_');
  await expect(page.getByRole('button', { name: 'Baixar .urdf' })).toBeVisible();

  // Test URDF file upload into simulator
  await page.getByRole('button', { name: 'Modelo' }).click();
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
  await expect(page.locator('.sidebar')).toContainText('1 juntas');

  // Verify STL CAD upload button exists in scene toolbar
  await expect(page.locator('label[aria-label="Carregar malha STL"]')).toBeVisible();
});

test('joint control interval limits auto-clamp positions, update IK slider bounds, and constrain solver', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

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
  await expect(page.getByRole('heading', { name: 'Cinemática DH' })).toBeVisible();

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
  await pickOption(page, 'Selecionar preset industrial', 'Universal Cobot UR5 (6R) (6 DOF)');
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();

  const xInput = page.getByRole('spinbutton', { name: 'Alvo X (u)', exact: true });
  const yInput = page.getByRole('spinbutton', { name: 'Alvo Y (u)', exact: true });
  const zInput = page.getByRole('spinbutton', { name: 'Alvo Z (u)', exact: true });
  const xSlider = page.getByRole('slider', { name: 'Deslizar Alvo X (u)', exact: true });
  const zSlider = page.getByRole('slider', { name: 'Deslizar Alvo Z (u)', exact: true });

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
  await expect(page.getByRole('heading', { name: 'Ferramenta do efetuador (TCP)' })).toBeVisible();

  // Switch between the 3 industrial tool types on Model tab
  await page.getByRole('button', { name: 'Ventosa a Vácuo', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Ventosa a Vácuo');
  await page.getByRole('button', { name: 'Tocha de Solda', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Tocha de Solda');

  // Switch to UR5 6-DOF robot and Inverse tab
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
  await page.getByRole('button', { name: 'Inversa', exact: true }).click();

  const sidebarGizmoToggle = page.getByRole('checkbox', { name: 'Gizmo 3D interativo', exact: true });
  const toolbarGizmoToggle = page.getByRole('checkbox', { name: 'Gizmo 3D', exact: true });
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
  await expect(page.getByRole('combobox', { name: 'Tarefa', exact: true })).toContainText('Pose XYZ + orientação');

  expect(errors).toEqual([]);
});

test('environment collision detects elevated workbench floor and blocks motion through 3D obstacles', async ({ page }) => {
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Ambiente e Colisão Externa (Piso / Obstáculo 3D)' })).toBeVisible();

  // Position a 1.0u sphere obstacle ahead along the RPR sweep path at [4, 3, 0]
  await page.getByRole('button', { name: 'Posicionar à frente do efetuador', exact: true }).click();
  await expect(page.locator('.scene-toolbar').getByRole('checkbox', { name: 'Obstáculo 3D', exact: true })).toBeChecked();
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
});

test('tool configuration (TCP) synchronizes across exported code, URDF, report summary, and SQLite save/load', async ({ page }, info) => {
  await page.goto('/');
  await page.getByRole('button', { name: 'Tocha de Solda', exact: true }).click();
  await number(page, 'Offset Pitch (°)', -45);

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

  // Reloading saved simulation restores the tool even after switching back to default gripper
  await page.getByRole('button', { name: 'Modelo', exact: true }).click();
  await page.getByRole('button', { name: 'Restaurar ferramenta padrão', exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Garra Paralela');
  await page.getByRole('button', { name: `Carregar ${simName}`, exact: true }).click();
  await expect(page.locator('.scene-readout')).toContainText('Tocha de Solda');
  await expect(page.getByRole('spinbutton', { name: 'Offset Pitch (°)', exact: true })).toHaveValue('-45');

  page.once('dialog', dialog => dialog.accept());
  await page.getByRole('button', { name: `Apagar ${simName}`, exact: true }).click();
});

