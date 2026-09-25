import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { Vector3 } from 'three';

/**
 * Parses an STL binary or ASCII ArrayBuffer into a normalized Three.js BufferGeometry.
 * Scales the mesh proportionally so it fits the robotic simulation scale (~0.6 units).
 * @param {ArrayBuffer} arrayBuffer - Raw file buffer from FileReader
 * @returns {import('three').BufferGeometry} Normalized BufferGeometry ready for rendering
 */
export function parseStlBuffer(arrayBuffer) {
  const loader = new STLLoader();
  const geometry = loader.parse(arrayBuffer);

  geometry.computeBoundingBox();
  const box = geometry.boundingBox;
  const size = new Vector3();
  box.getSize(size);

  const maxDim = Math.max(size.x, size.y, size.z);
  if (maxDim > 0) {
    const targetScale = 0.6 / maxDim;
    geometry.scale(targetScale, targetScale, targetScale);
    geometry.center();
  }
  geometry.computeVertexNormals();
  return geometry;
}
