import { useEffect, useMemo, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Cylinder, Box, Sphere, Grid } from '@react-three/drei';
import { AxesHelper, Box3, BufferAttribute, CanvasTexture, MathUtils, Vector3 } from 'three';
import { DEG, dhMatrix, forwardKinematics } from './kinematics';

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

function Gripper() {
  return <group>
    <Sphere args={[0.12, 16, 16]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Sphere>
    <Box args={[0.08, 0.32, 0.1]} position={[0.04, 0, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.22, 0.06, 0.08]} position={[0.16, 0.13, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.1, 0.05, 0.08]} position={[0.28, 0.1, 0]} rotation={[0, 0, -0.45]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.22, 0.06, 0.08]} position={[0.16, -0.13, 0]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
    <Box args={[0.1, 0.05, 0.08]} position={[0.28, -0.1, 0]} rotation={[0, 0, 0.45]}>
      <meshStandardMaterial color="#ffd36b" metalness={0.5} roughness={0.3} emissive="#473310" />
    </Box>
  </group>;
}

function DHLink({ joint, index, isLast, showFrames, customStlGeometry, children }) {
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
      {isLast && (customStlGeometry ? <mesh geometry={customStlGeometry}><meshStandardMaterial color="#ffd36b" metalness={0.6} roughness={0.3} /></mesh> : <Gripper />)}
      {children}
    </group>
  </group>;
}

function RobotBuilder({ joints, showFrames, customStlGeometry, index = 0 }) {
  if (index >= joints.length) return null;
  return <DHLink joint={joints[index]} index={index} isLast={index === joints.length - 1} showFrames={showFrames} customStlGeometry={customStlGeometry}>
    <RobotBuilder joints={joints} showFrames={showFrames} customStlGeometry={customStlGeometry} index={index + 1} />
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
    if (!lineGeomRef.current || !pathPoints || pathPoints.length === 0) return;
    lineGeomRef.current.setAttribute('position', new BufferAttribute(pathPoints, 3));
    lineGeomRef.current.computeBoundingSphere();
  }, [pathPoints]);

  if (!visible || !pathPoints || pathPoints.length === 0) return null;
  return <group>
    <line>
      <bufferGeometry ref={lineGeomRef} />
      <lineBasicMaterial color="#3dd6b5" />
    </line>
    {waypointCoords?.map((pos, idx) => (
      <Sphere key={idx} args={[0.09, 12, 12]} position={pos}>
        <meshStandardMaterial color="#ff9f43" emissive="#d9534f" emissiveIntensity={0.5} />
      </Sphere>
    ))}
  </group>;
}

function Scene({ joints, showFrames, viewKey, workspaceCloud, showWorkspace, trajectoryPathPoints, trajectoryWaypointCoords, showTrajectory, customStlGeometry }) {
  const controls = useRef();
  return <>
    <color attach="background" args={['#171b1e']} />
    <ambientLight intensity={0.8} />
    <directionalLight position={[10, -10, 15]} intensity={2} />
    <directionalLight position={[-10, 10, 5]} intensity={0.7} />
    <Grid infiniteGrid fadeDistance={70} sectionColor="#424b50" cellColor="#292f33" rotation={[Math.PI / 2, 0, 0]} position={[0, 0, -0.22]} />
    {showFrames && <Frame index={0} />}
    <Cylinder args={[0.5, 0.6, 0.2, 24]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, -0.1]}><meshStandardMaterial color="#adb9bf" metalness={0.5} roughness={0.4} /></Cylinder>
    <RobotBuilder joints={joints} showFrames={showFrames} customStlGeometry={customStlGeometry} />
    <WorkspacePoints cloud={workspaceCloud} visible={showWorkspace} />
    <TrajectoryVisualizer pathPoints={trajectoryPathPoints} waypointCoords={trajectoryWaypointCoords} visible={showTrajectory} />
    <OrbitControls ref={controls} makeDefault />
    <CameraFit joints={joints} viewKey={viewKey} controls={controls} />
  </>;
}

export default function RobotScene(props) {
  return <Canvas camera={{ position: [10, -12, 10], up: [0, 0, 1], fov: 45 }} gl={{ preserveDrawingBuffer: true }}>
    <Scene {...props} />
  </Canvas>;
}
