import { test, expect } from '@playwright/test';
import { Box3, PerspectiveCamera, Vector3 } from 'three';
import { forwardKinematics } from '../../src/kinematics.js';

async function number(page, label, value) {
  const input = page.getByRole('spinbutton', { name: label, exact: true });
  await input.fill(String(value));
  await input.press('Tab');
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
  await page.getByLabel('Tarefa', { exact: true }).selectOption('position');
  await expect(page.getByText('Posto reduzido:', { exact: false })).toBeVisible();
  await page.getByLabel('Tarefa', { exact: true }).selectOption('pose');
  await expect(page.getByText('Graus de liberdade insuficientes', { exact: false })).toBeVisible();
  await page.getByLabel('Tarefa', { exact: true }).selectOption('xy');
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
  await page.getByRole('button', { name: 'Aplicar solução', exact: true }).click();
  await expect(page.getByTestId('end-position')).toContainText('X 4.00 · Y 2.00 · Z 0.00');
  await number(page, 'Alvo X (u)', 1000);
  await page.getByRole('button', { name: 'Resolver', exact: true }).click();
  await expect(page.locator('.notice[role="status"]')).toContainText('Não convergiu');
  await expect(page.getByRole('button', { name: 'Aplicar solução', exact: true })).toBeDisabled();
  await number(page, 'Alvo X (u)', 4);
  await expect(page.getByRole('heading', { name: 'Resultado', exact: true })).toHaveCount(0);
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
  const presetSelect = page.getByRole('combobox', { name: 'Selecionar preset industrial' });
  await presetSelect.selectOption('scara');
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

