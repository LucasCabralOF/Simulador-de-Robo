import { useEffect, useMemo, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Cylinder, Box, Sphere, Grid, TransformControls } from '@react-three/drei';
import { AxesHelper, Box3, BufferAttribute, CanvasTexture, Euler, MathUtils, Vector3 } from 'three';
import { DEG, dhMatrix, forwardKinematics, rotationFromRPY } from './kinematics';

function AxisLabel({ text, position, color }) {
  const texture = useMemo(() => {
    const canvas = document.createElement('canvas');
    canvas.width = 128; canvas.height = 64;
    const context = canvas.getContext('2d');
    context.font = 'bold 40px Arial';
    context.textAlign = 'center'; context.textBaseline = 'middle';
    context.fillStyle = color;
    context.fillText(text, 64, 32);
    return new CanvasTexture(canvas);
  }, [text, color]);
  useEffect(() => () => texture.dispose(), [texture]);
  return <sprite position={position} scale={[0.8, 0.4, 1]}><spriteMaterial map={texture} transparent depthTest={false} /></sprite>;
}

function FrameAxes({ size = 1.4 }) {
  const helper = useMemo(() => {
    const h = new AxesHelper(size);
    h.material.depthTest = false;
    h.material.transparent = true;
    h.renderOrder = 2;
    return h;
  }, [size]);
  useEffect(() => () => helper.dispose(), [helper]);
  return <primitive object={helper} />;
}

function Frame({ index }) {
  const labelDistance = 1.5 + (index % 4) * 0.45;
  return <>
    <FrameAxes size={1.4} />
    <AxisLabel text={`X${index}`} position={[labelDistance, 0, 0]} color="#ff7182" />
    <AxisLabel text={`Y${index}`} position={[0, labelDistance, 0]} color="#7ce39c" />
    <AxisLabel text={`Z${index}`} position={[0, 0, labelDistance]} color="#7bbdff" />
  </>;
}

function Gripper({ openFrac = 0.5 }) {
  const offset = 0.07 + 0.12 * Math.max(0, Math.min(1, openFrac));
  const tipOffset = Math.max(0.04, offset - 0.03);
  return <group>
    <Sphere args={[0.12, 16, 16]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Sphere>
    <Box args={[0.08, 0.34, 0.1]} position={[0.04, 0, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.22, 0.06, 0.08]} position={[0.16, offset, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.1, 0.05, 0.08]} position={[0.28, tipOffset, 0]} rotation={[0, 0, -0.45]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.22, 0.06, 0.08]} position={[0.16, -offset, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.1, 0.05, 0.08]} position={[0.28, -tipOffset, 0]} rotation={[0, 0, 0.45]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
  </group>;
}

function VacuumTool({ level = 0.5 }) {
  const active = Math.max(0, Math.min(1, level));
  return <group>
    <Sphere args={[0.11, 16, 16]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Sphere>
    <Cylinder args={[0.12, 0.12, 0.08, 20]} position={[0.05, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Cylinder>
    <Cylinder args={[0.06, 0.075, 0.14, 18]} position={[0.15, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#58b9e6" metalness={0.4} roughness={0.35} />
    </Cylinder>
    <Cylinder args={[0.17, 0.08, 0.1, 24]} position={[0.27, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#2a3640" roughness={0.7} />
    </Cylinder>
    <mesh position={[0.32, 0, 0]} rotation={[0, Math.PI / 2, 0]}>
      <torusGeometry args={[0.165, 0.02, 10, 24]} />
      <meshStandardMaterial color="#4dd4ff" emissive="#1b8fb8" emissiveIntensity={0.3 + 0.6 * active} />
    </mesh>
  </group>;
}

function WelderTool({ level = 0.5 }) {
  const active = Math.max(0, Math.min(1, level));
  return <group>
    <Sphere args={[0.11, 16, 16]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Sphere>
    <Cylinder args={[0.095, 0.095, 0.12, 20]} position={[0.07, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#2f3a44" metalness={0.3} roughness={0.5} />
    </Cylinder>
    <Cylinder args={[0.045, 0.085, 0.18, 20]} position={[0.21, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#e8a750" metalness={0.65} roughness={0.28} emissive="#4a2a08" />
    </Cylinder>
    <Cylinder args={[0.008, 0.025, 0.09, 14]} position={[0.34, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#dce6ee" metalness={0.7} roughness={0.2} />
    </Cylinder>
    <Sphere args={[0.03 + 0.02 * active, 14, 14]} position={[0.38, 0, 0]}>
      <meshStandardMaterial color="#80f2ff" emissive="#4dd4ff" emissiveIntensity={0.6 + 0.9 * active} />
    </Sphere>
  </group>;
}

function EndEffectorTool({ toolType = 'gripper', toolOffsetRpy = [0, 0, 0], toolGrip = 50, customStlGeometry }) {
  const offsetMatrix = useMemo(() => rotationFromRPY(toolOffsetRpy), [toolOffsetRpy]);
  const level = Math.max(0, Math.min(1, Number(toolGrip) / 100 || 0));
  const tipLength = toolType === 'welder' ? 0.38 : toolType === 'vacuum' ? 0.32 : 0.28;
  return <group matrix={offsetMatrix} matrixAutoUpdate={false}>
    {customStlGeometry ? (
      <mesh geometry={customStlGeometry}>
        <meshStandardMaterial color="#ffd36b" metalness={0.6} roughness={0.3} />
      </mesh>
    ) : toolType === 'vacuum' ? (
      <VacuumTool level={level} />
    ) : toolType === 'welder' ? (
      <WelderTool level={level} />
    ) : (
      <Gripper openFrac={level} />
    )}
    <Cylinder args={[0.012, 0.012, 0.28, 10]} position={[tipLength + 0.14, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#62d5b5" emissive="#279d84" emissiveIntensity={0.6} transparent opacity={0.8} />
    </Cylinder>
    <Cylinder args={[0, 0.038, 0.09, 12]} position={[tipLength + 0.30, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
      <meshStandardMaterial color="#62d5b5" emissive="#279d84" emissiveIntensity={0.75} />
    </Cylinder>
  </group>;
}

function DHLink({ joint, index, isLast, showFrames, customStlGeometry, toolType, toolOffsetRpy, toolGrip, children }) {
  const d = joint.type === 'P' ? joint.val : joint.d;
  const theta = (joint.type === 'R' ? joint.val : joint.theta) * DEG;
  return <group>
    {joint.type === 'R' ? <Cylinder args={[0.3, 0.3, 0.4, 24]} rotation={[Math.PI / 2, 0, 0]}>
      <meshStandardMaterial color="#e65c93" metalness={0.3} roughness={0.45} />
    </Cylinder> : <Box args={[0.35, 0.35, 0.6]} position={[0, 0, d / 2]}>
      <meshStandardMaterial color="#42b1e1" metalness={0.3} roughness={0.45} />
    </Box>}
    <group rotation={[0, 0, theta]}>
      {d !== 0 && <Cylinder args={[0.065, 0.065, Math.abs(d), 16]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, d / 2]}><meshStandardMaterial color="#70d9b0" /></Cylinder>}
      {joint.a !== 0 && <Cylinder args={[0.065, 0.065, Math.abs(joint.a), 16]} rotation={[0, 0, Math.PI / 2]} position={[joint.a / 2, 0, d]}><meshStandardMaterial color="#70d9b0" /></Cylinder>}
    </group>
    <group matrix={dhMatrix(joint)} matrixAutoUpdate={false}>
      {showFrames && <Frame index={index + 1} />}
      {isLast && <EndEffectorTool toolType={toolType} toolOffsetRpy={toolOffsetRpy} toolGrip={toolGrip} customStlGeometry={customStlGeometry} />}
      {children}
    </group>
  </group>;
}

function RobotBuilder({ joints, showFrames, customStlGeometry, toolType, toolOffsetRpy, toolGrip, index = 0 }) {
  if (index >= joints.length) return null;
  return <DHLink joint={joints[index]} index={index} isLast={index === joints.length - 1} showFrames={showFrames} customStlGeometry={customStlGeometry} toolType={toolType} toolOffsetRpy={toolOffsetRpy} toolGrip={toolGrip}>
    <RobotBuilder joints={joints} showFrames={showFrames} customStlGeometry={customStlGeometry} toolType={toolType} toolOffsetRpy={toolOffsetRpy} toolGrip={toolGrip} index={index + 1} />
  </DHLink>;
}

function CameraFit({ joints, viewKey, controls }) {
  const previous = useRef('');
  useFrame(({ camera, size }) => {
    const key = `${viewKey}:${size.width}:${size.height}:${joints.length}`;
    if (previous.current === key || !controls.current) return;
    previous.current = key;
    const fk = forwardKinematics(joints);
    const points = fk.positions.map(p => new Vector3(...p));
    joints.forEach((j, i) => points.push(new Vector3(0, 0, j.type === 'P' ? j.val : j.d).applyMatrix4(fk.transforms[i])));
    const box = new Box3().setFromPoints(points).expandByScalar(1.8);
    const center = box.getCenter(new Vector3());
    const radius = box.getSize(new Vector3()).length() / 2;
    const verticalFov = MathUtils.degToRad(camera.fov);
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * size.width / size.height);
    const distance = radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2);
    camera.position.copy(center).add(new Vector3(1, -1.4, 1.2).normalize().multiplyScalar(distance));
    camera.near = Math.max(distance / 10000, 0.001);
    camera.far = Math.max(distance * 20, 1000);
    camera.updateProjectionMatrix();
    controls.current.target.copy(center);
    controls.current.update();
  });
  return null;
}

function WorkspacePoints({ cloud, visible }) {
  const geomRef = useRef();
  useEffect(() => {
    if (!geomRef.current || !cloud) return;
    geomRef.current.setAttribute('position', new BufferAttribute(cloud.positions, 3));
    geomRef.current.setAttribute('color', new BufferAttribute(cloud.colors, 3));
    geomRef.current.computeBoundingSphere();
  }, [cloud]);

  if (!visible || !cloud) return null;
  return <points>
    <bufferGeometry ref={geomRef} />
    <pointsMaterial size={0.08} vertexColors transparent opacity={0.65} sizeAttenuation />
  </points>;
}

function TrajectoryVisualizer({ pathPoints, waypointCoords, visible }) {
  const lineGeomRef = useRef();
  useEffect(() => {
    if (!visible || !lineGeomRef.current || !pathPoints || pathPoints.length === 0) return;
    lineGeomRef.current.setAttribute('position', new BufferAttribute(pathPoints, 3));
    lineGeomRef.current.computeBoundingSphere();
  }, [pathPoints, visible]);

  if (!visible || !pathPoints || pathPoints.length === 0) return null;
  return <group>
    <line>
      <bufferGeometry ref={lineGeomRef} />
      <lineBasicMaterial color="#3dd6b5" />
    </line>
    {waypointCoords?.map((pos, idx) => (
      <group key={idx} position={pos}>
        <Sphere args={[0.09, 12, 12]}>
          <meshStandardMaterial color="#ff9f43" emissive="#d9534f" emissiveIntensity={0.5} />
        </Sphere>
        <mesh>
          <torusGeometry args={[0.16, 0.016, 10, 24]} />
          <meshStandardMaterial color="#ffb86c" emissive="#ff9f43" emissiveIntensity={0.6} transparent opacity={0.85} />
        </mesh>
      </group>
    ))}
  </group>;
}

function TargetGizmo({ position, rpy, enabled = true, gizmoMode = 'translate', taskMode = 'position', onDragTarget }) {
  const targetRef = useRef(null);
  const isDraggingRef = useRef(false);
  const validPos = Array.isArray(position) && position.length === 3 && position.every(Number.isFinite);
  const [px, py, pz] = validPos ? position : [0, 0, 0];
  const [r = 0, p = 0, y = 0] = Array.isArray(rpy) && rpy.length === 3 ? rpy : [0, 0, 0];

  useEffect(() => {
    if (!validPos || !targetRef.current || isDraggingRef.current) return;
    targetRef.current.position.set(px, py, pz);
    targetRef.current.rotation.set(r * DEG, p * DEG, y * DEG, 'ZYX');
    targetRef.current.updateMatrixWorld();
  }, [validPos, px, py, pz, r, p, y]);

  if (!validPos) return null;

  return <>
    <group ref={targetRef} position={[px, py, pz]} rotation={[r * DEG, p * DEG, y * DEG, 'ZYX']}>
      <Sphere args={[0.14, 16, 16]}>
        <meshStandardMaterial color="#4dd4ff" emissive="#1b8fb8" emissiveIntensity={0.6} transparent opacity={0.45} />
      </Sphere>
      <mesh>
        <torusGeometry args={[0.24, 0.015, 10, 28]} />
        <meshStandardMaterial color="#62d5b5" emissive="#279d84" emissiveIntensity={0.7} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <torusGeometry args={[0.24, 0.015, 10, 28]} />
        <meshStandardMaterial color="#62d5b5" emissive="#279d84" emissiveIntensity={0.7} />
      </mesh>
      <Cylinder args={[0.014, 0.014, 0.3, 10]} position={[0.22, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <meshStandardMaterial color="#4dd4ff" emissive="#1b8fb8" emissiveIntensity={0.7} />
      </Cylinder>
      <Cylinder args={[0, 0.04, 0.09, 12]} position={[0.39, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <meshStandardMaterial color="#4dd4ff" emissive="#1b8fb8" emissiveIntensity={0.85} />
      </Cylinder>
    </group>
    {enabled && (
      <TransformControls
        object={targetRef}
        mode={gizmoMode}
        size={0.78}
        space={gizmoMode === 'rotate' ? 'local' : 'world'}
        showZ={taskMode !== 'xy' || gizmoMode === 'rotate'}
        onMouseDown={() => { isDraggingRef.current = true; }}
        onMouseUp={() => { isDraggingRef.current = false; }}
        onObjectChange={() => {
          const obj = targetRef.current;
          if (!obj || !isDraggingRef.current) return;
          if (gizmoMode === 'translate') {
            const nextZ = taskMode === 'xy' ? pz : obj.position.z;
            if (taskMode === 'xy') obj.position.z = pz;
            const clamped = onDragTarget?.({ position: [obj.position.x, obj.position.y, nextZ] });
            if (Array.isArray(clamped) && clamped.length === 3) {
              obj.position.set(clamped[0], clamped[1], clamped[2]);
            }
          } else if (gizmoMode === 'rotate') {
            const euler = new Euler().setFromQuaternion(obj.quaternion, 'ZYX');
            const nextRpy = [
              Math.round((euler.x / DEG) * 10) / 10,
              Math.round((euler.y / DEG) * 10) / 10,
              Math.round((euler.z / DEG) * 10) / 10,
            ];
            onDragTarget?.({ rpy: nextRpy });
          }
        }}
      />
    )}
  </>;
}

function CollisionMarker({ position }) {
  if (!Array.isArray(position) || position.length !== 3 || !position.every(Number.isFinite)) return null;
  return <group position={position}>
    <Sphere args={[0.18, 18, 18]}>
      <meshStandardMaterial color="#ff5c6c" emissive="#d92b3a" emissiveIntensity={0.85} transparent opacity={0.78} />
    </Sphere>
    <mesh>
      <torusGeometry args={[0.32, 0.02, 12, 28]} />
      <meshStandardMaterial color="#ff9f43" emissive="#ff5c6c" emissiveIntensity={0.8} />
    </mesh>
    <mesh rotation={[Math.PI / 2, 0, 0]}>
      <torusGeometry args={[0.32, 0.02, 12, 28]} />
      <meshStandardMaterial color="#ff9f43" emissive="#ff5c6c" emissiveIntensity={0.8} />
    </mesh>
  </group>;
}

function EnvironmentObstacle({ environment }) {
  const floorEnabled = environment?.floorEnabled ?? true;
  const floorZ = Number.isFinite(environment?.floorZ) ? environment.floorZ : -0.22;
  const showPlate = floorEnabled && Math.abs(floorZ - (-0.22)) > 1e-3;
  const obs = environment?.obstacle;
  const showObstacle = Boolean(obs?.enabled && Array.isArray(obs.position) && obs.position.length === 3);
  const size = showObstacle ? Math.max(0.1, Number(obs.radius) || 0.8) : 0.8;

  return <>
    {showPlate && (
      <mesh position={[0, 0, floorZ - 0.01]}>
        <boxGeometry args={[28, 28, 0.02]} />
        <meshStandardMaterial color="#274047" transparent opacity={0.32} roughness={0.65} />
      </mesh>
    )}
    {showObstacle && (
      <group position={obs.position}>
        {obs.shape === 'box' ? <>
          <Box args={[size * 2, size * 2, size * 2]}>
            <meshStandardMaterial color="#e07a5f" emissive="#5e2114" emissiveIntensity={0.35} transparent opacity={0.78} roughness={0.4} />
          </Box>
          <Box args={[size * 2.02, size * 2.02, size * 2.02]}>
            <meshBasicMaterial color="#ffb097" wireframe />
          </Box>
        </> : <>
          <Sphere args={[size, 28, 20]}>
            <meshStandardMaterial color="#e07a5f" emissive="#5e2114" emissiveIntensity={0.35} transparent opacity={0.78} roughness={0.4} />
          </Sphere>
          <mesh>
            <torusGeometry args={[size * 1.01, 0.015, 8, 32]} />
            <meshBasicMaterial color="#ffb097" />
          </mesh>
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <torusGeometry args={[size * 1.01, 0.015, 8, 32]} />
            <meshBasicMaterial color="#ffb097" />
          </mesh>
        </>}
      </group>
    )}
  </>;
}

function Scene({ joints, showFrames, viewKey, workspaceCloud, showWorkspace, trajectoryPathPoints, trajectoryWaypointCoords, showTrajectory, customStlGeometry, toolType, toolOffsetRpy, toolGrip, ikTarget, ikRpy, gizmoEnabled, gizmoMode, taskMode, onGizmoDrag, collisionPoint, environment }) {
  const controls = useRef();
  const gridZ = environment?.floorEnabled ? (Number.isFinite(environment.floorZ) ? environment.floorZ : -0.22) : -0.22;
  return <>
    <color attach="background" args={['#171b1e']} />
    <ambientLight intensity={0.8} />
    <directionalLight position={[10, -10, 15]} intensity={2} />
    <directionalLight position={[-10, 10, 5]} intensity={0.7} />
    <Grid infiniteGrid fadeDistance={70} sectionColor="#424b50" cellColor="#292f33" rotation={[Math.PI / 2, 0, 0]} position={[0, 0, gridZ]} />
    {showFrames && <Frame index={0} />}
    <Cylinder args={[0.5, 0.6, 0.2, 24]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, -0.1]}><meshStandardMaterial color="#adb9bf" metalness={0.5} roughness={0.4} /></Cylinder>
    <EnvironmentObstacle environment={environment} />
    <RobotBuilder joints={joints} showFrames={showFrames} customStlGeometry={customStlGeometry} toolType={toolType} toolOffsetRpy={toolOffsetRpy} toolGrip={toolGrip} />
    <WorkspacePoints cloud={workspaceCloud} visible={showWorkspace} />
    <TrajectoryVisualizer pathPoints={trajectoryPathPoints} waypointCoords={trajectoryWaypointCoords} visible={showTrajectory} />
    <TargetGizmo position={ikTarget} rpy={ikRpy} enabled={gizmoEnabled} gizmoMode={gizmoMode} taskMode={taskMode} onDragTarget={onGizmoDrag} />
    <CollisionMarker position={collisionPoint} />
    <OrbitControls ref={controls} makeDefault />
    <CameraFit joints={joints} viewKey={viewKey} controls={controls} />
  </>;
}

export default function RobotScene(props) {
  return <Canvas camera={{ position: [10, -12, 10], up: [0, 0, 1], fov: 45 }} gl={{ preserveDrawingBuffer: true }}>
    <Scene {...props} />
  </Canvas>;
}
