import { memo } from 'react';
import { CheckCheck, Copy, Download } from 'lucide-react';
import { Button, SegmentedTabs } from '../ui';

const ExportTab = memo(function ExportTab({ codeTarget, setCodeTarget, generatedCode, downloadUrdf, copyCode, copied }) {
  return (
    <section>
      <h2>Exportação de Código de Controle</h2>
      <p className="muted">Gera implementação autônoma e autocontida da cinemática para microcontroladores ou robótica em Python.</p>
      <SegmentedTabs
        ariaLabel="Linguagem de exportação"
        activeId={codeTarget}
        onChange={setCodeTarget}
        items={[
          { id: 'cpp', label: 'C++ / Arduino' },
          { id: 'python', label: 'Python / NumPy' },
          { id: 'ros2', label: 'ROS 2 (Node)' },
          { id: 'urdf', label: 'URDF (ROS)' },
        ]}
      />
      <div className="code-box-header">
        <span className="muted">{codeTarget === 'cpp' ? 'Arduino / ESP32 / C++11' : codeTarget === 'python' ? 'NumPy script' : codeTarget === 'ros2' ? 'rclpy node' : 'URDF XML (ROS)'}</span>
        <div className="btn-row">
          {codeTarget === 'urdf' && (
            <Button onClick={downloadUrdf}>
              <Download size={16} /> Baixar .urdf
            </Button>
          )}
          <Button onClick={copyCode}>{copied ? <CheckCheck size={16} /> : <Copy size={16} />}{copied ? 'Copiado!' : 'Copiar código'}</Button>
        </div>
      </div>
      <pre className="code-block"><code>{generatedCode}</code></pre>
    </section>
  );
});

export default ExportTab;
