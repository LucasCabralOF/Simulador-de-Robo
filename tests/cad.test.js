import test from 'node:test';
import assert from 'node:assert/strict';
import { parseStlBuffer } from '../src/cadLoader.js';

test('parseStlBuffer parses ASCII STL and normalizes bounding scale', () => {
  const asciiStl = `solid test_triangle
  facet normal 0.0 0.0 1.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 100.0 0.0 0.0
      vertex 0.0 100.0 0.0
    endloop
  endfacet
endsolid test_triangle
`;
  const buffer = new TextEncoder().encode(asciiStl).buffer;
  const geometry = parseStlBuffer(buffer);

  assert.ok(geometry);
  assert.equal(geometry.attributes.position.count, 3);

  // Maximum dimension should be normalized close to 0.6 units
  geometry.computeBoundingBox();
  const box = geometry.boundingBox;
  const dx = box.max.x - box.min.x;
  const dy = box.max.y - box.min.y;
  assert.ok(Math.abs(Math.max(dx, dy) - 0.6) < 0.001);
});
