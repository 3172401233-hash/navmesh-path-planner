'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const { NavMesh, NavMeshError } = require('../src/index');
const { flatSquare } = require('../support/helpers');

// Two triangles sharing edge (1, 2); a minimal valid mesh.
function twoTriangles() {
  return {
    vertices: [
      [0, 0, 0],
      [1, 0, 0],
      [0, 0, 1],
      [1, 0, 1],
    ],
    triangles: [
      [0, 1, 2],
      [1, 3, 2],
    ],
  };
}

test('valid mesh with explicit adjacency is accepted and matches auto-build', () => {
  const def = twoTriangles();
  const auto = new NavMesh(def);
  const manual = new NavMesh({
    ...def,
    adjacency: [
      [-1, 1, -1],
      [-1, -1, 0],
    ],
  });
  assert.deepEqual(manual.adjacency, auto.adjacency);
});

test('duplicate vertices are rejected', () => {
  const def = twoTriangles();
  def.vertices[3] = [1, 0, 0]; // duplicates vertex 1
  assert.throws(() => new NavMesh(def), (err) => {
    assert.ok(err instanceof NavMeshError);
    assert.match(err.message, /duplicate vertex/i);
    return true;
  });
});

test('degenerate triangles are rejected', () => {
  assert.throws(
    () =>
      new NavMesh({
        vertices: [
          [0, 0, 0],
          [1, 0, 0],
          [2, 0, 0],
        ],
        triangles: [[0, 1, 2]],
      }),
    /degenerate/i,
  );
  assert.throws(
    () =>
      new NavMesh({
        vertices: [
          [0, 0, 0],
          [1, 0, 0],
          [0, 0, 1],
        ],
        triangles: [[0, 0, 1]],
      }),
    /degenerate|reuses/i,
  );
});

test('triangle vertex index out of range is rejected', () => {
  const def = twoTriangles();
  def.triangles[0] = [0, 1, 7];
  assert.throws(() => new NavMesh(def), /out of range/i);
});

test('invalid region costs are rejected', () => {
  const def = twoTriangles();
  assert.throws(() => new NavMesh({ ...def, costs: [1, 0] }), /cost/i);
  assert.throws(() => new NavMesh({ ...def, costs: [1, -2] }), /cost/i);
  assert.throws(() => new NavMesh({ ...def, costs: [1, NaN] }), /cost/i);
  assert.throws(() => new NavMesh({ ...def, costs: [1] }), /cost/i);
});

test('missing adjacency entries are reported, not silently rebuilt', () => {
  const def = twoTriangles();
  assert.throws(
    () =>
      new NavMesh({
        ...def,
        adjacency: [
          [-1, -1, -1],
          [-1, -1, -1],
        ],
      }),
    /missing adjacency/i,
  );
});

test('inconsistent shared edges are rejected', () => {
  const def = twoTriangles();
  // Claims a neighbor on an edge that is actually a border.
  assert.throws(
    () =>
      new NavMesh({
        ...def,
        adjacency: [
          [1, 1, -1],
          [-1, -1, 0],
        ],
      }),
    /inconsistent shared edge/i,
  );
  // One-sided link: 0 -> 1 exists but 1 -> 0 is missing.
  assert.throws(
    () =>
      new NavMesh({
        ...def,
        adjacency: [
          [-1, 1, -1],
          [-1, -1, -1],
        ],
      }),
    /missing adjacency|inconsistent/i,
  );
  // Self reference.
  assert.throws(
    () =>
      new NavMesh({
        ...def,
        adjacency: [
          [0, 1, -1],
          [-1, -1, 0],
        ],
      }),
    /itself|invalid/i,
  );
});

test('non-manifold edges are rejected', () => {
  assert.throws(
    () =>
      new NavMesh({
        vertices: [
          [0, 0, 0],
          [1, 0, 0],
          [0, 0, 1],
          [0, 1, 0],
          [2, 0, 1],
        ],
        triangles: [
          [0, 1, 2],
          [0, 1, 3],
          [0, 1, 4],
        ],
      }),
    /non-manifold/i,
  );
});

test('all issues are collected and exposed on NavMeshError', () => {
  const def = twoTriangles();
  def.vertices[3] = [1, 0, 0]; // duplicate
  def.triangles[1] = [1, 3, 9]; // out of range
  try {
    new NavMesh(def);
    assert.fail('should have thrown');
  } catch (err) {
    assert.ok(err instanceof NavMeshError);
    assert.ok(err.issues.length >= 2, `expected multiple issues, got ${err.issues.length}`);
  }
});

test('invalid query inputs throw instead of being silently ignored', () => {
  const { mesh } = flatSquare(2);
  assert.throws(() => mesh.findPath({ start: [0, 0], goal: [1, 0, 1] }), TypeError);
  assert.throws(() => mesh.findPath({ start: [0, 0, 0], goal: [1, 0, NaN] }), TypeError);
  assert.throws(
    () => mesh.findPath({ start: [0, 0, 0], goal: [1, 0, 1], maxSnapDistance: -1 }),
    TypeError,
  );
  assert.throws(() => mesh.setBlocked([999]), NavMeshError);
  assert.throws(() => mesh.setBlocked([-1]), NavMeshError);
});
