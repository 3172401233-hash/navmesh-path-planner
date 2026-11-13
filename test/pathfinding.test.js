'use strict';

const { test } = require('node:test');
const assert = require('node:assert/strict');
const {
  lCorridor,
  uShape,
  twoCorridors,
  islands,
  flatSquare,
  trianglesOfCells,
  pathLength,
  assertPointClose,
  assertPathOnMesh,
  assertSegmentsOnMesh,
} = require('../support/helpers');

test('L-shaped corridor: funnel produces a single turn at the reflex corner', () => {
  const { mesh } = lCorridor();
  const result = mesh.findPath({
    start: [0.5, 0, 0.5],
    goal: [8.5, 0, 8.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(result.status, 'success');
  assert.equal(result.path.length, 3);
  assert.equal(result.diagnostics.turnPoints, 1);
  assertPointClose(result.path[0], [0.5, 0, 0.5], 1e-9, assert, 'start');
  assertPointClose(result.path[1], [8, 0, 2], 1e-9, assert, 'corner');
  assertPointClose(result.path[2], [8.5, 0, 8.5], 1e-9, assert, 'goal');
  assert.ok(Math.abs(result.cost - pathLength(result.path)) < 1e-9);
  assertPathOnMesh(mesh, result.path, assert);
});

test('concave U-shaped region: path stays on mesh and turns around the notch', () => {
  const { mesh } = uShape();
  const start = [0.5, 0, 4.5];
  const goal = [6.5, 0, 4.5];
  const result = mesh.findPath({ start, goal, maxSnapDistance: 0.5 });
  assert.equal(result.status, 'success');
  const straight = 6; // euclidean distance crosses the void
  const len = pathLength(result.path);
  assert.ok(len > straight + 0.5, `path (${len}) must detour around the notch`);
  assert.ok(len < 13, `path (${len}) should not wander`);
  assert.ok(result.diagnostics.turnPoints >= 2, 'needs at least two turns');
  assertPathOnMesh(mesh, result.path, assert);
  assertSegmentsOnMesh(mesh, result.path, assert);
});

test('multiple exits: the shorter corridor is preferred', () => {
  const { mesh } = twoCorridors();
  const result = mesh.findPath({
    start: [1, 0, 0.5],
    goal: [8, 0, 0.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(result.status, 'success');
  const maxZ = Math.max(...result.path.map((p) => p[2]));
  assert.ok(maxZ <= 2 + 1e-9, `must use the lower corridor A (maxZ=${maxZ})`);
  assert.ok(result.cost < 8, `cost ${result.cost} should be near the 7.0 optimum`);
  assert.ok(result.diagnostics.turnPoints <= 2);
  assertPathOnMesh(mesh, result.path, assert);
});

test('region costs: expensive shortcut is avoided in favour of the cheap detour', () => {
  const cheap = twoCorridors();
  const fast = cheap.mesh.findPath({
    start: [1, 0, 0.5],
    goal: [8, 0, 0.5],
    maxSnapDistance: 0.5,
  });

  const expensive = twoCorridors({ corridorACost: 50 });
  const detour = expensive.mesh.findPath({
    start: [1, 0, 0.5],
    goal: [8, 0, 0.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(detour.status, 'success');
  const maxZ = Math.max(...detour.path.map((p) => p[2]));
  assert.ok(maxZ >= 4 - 1e-6, 'must route through corridor B when A is expensive');
  assert.ok(detour.cost > fast.cost, 'detour is longer than the direct path');
  assert.ok(detour.cost < 50, 'but still far cheaper than crossing the expensive region');
  assertPathOnMesh(expensive.mesh, detour.path, assert);
});

test('disconnected islands: goal on another island is unreachable', () => {
  const { mesh } = islands();
  const result = mesh.findPath({
    start: [1.5, 0, 1.5],
    goal: [6.5, 0, 1.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(result.status, 'no-path');
  assert.equal(result.reason, 'unreachable');
  assert.deepEqual(result.path, []);
  assert.equal(result.cost, null);
});

test('dynamic blocking: reroutes around blocked corridor and never serves stale cache', () => {
  const { mesh, cellTriangles, corridorACells } = twoCorridors();
  const query = { start: [1, 0, 0.5], goal: [8, 0, 0.5], maxSnapDistance: 0.5 };

  const r1 = mesh.findPath(query);
  assert.equal(r1.status, 'success');
  assert.equal(r1.diagnostics.cacheHit, false);
  assert.ok(Math.max(...r1.path.map((p) => p[2])) <= 2 + 1e-9, 'uses corridor A');

  const r2 = mesh.findPath(query);
  assert.equal(r2.diagnostics.cacheHit, true);
  assert.deepEqual(r2.path, r1.path);

  // Mutating a returned result must not corrupt the cache.
  r1.path[0][0] = 999;
  const r2b = mesh.findPath(query);
  assert.notEqual(r2b.path[0][0], 999);

  mesh.setBlocked(trianglesOfCells(cellTriangles, corridorACells));
  const r3 = mesh.findPath(query);
  assert.equal(r3.status, 'success');
  assert.equal(r3.diagnostics.cacheHit, false, 'block change must invalidate the cache');
  const maxZ = Math.max(...r3.path.map((p) => p[2]));
  assert.ok(maxZ >= 4 - 1e-6, 'must reroute through corridor B');
  assertPathOnMesh(mesh, r3.path, assert);

  mesh.clearBlocked();
  const r4 = mesh.findPath(query);
  assert.equal(r4.diagnostics.cacheHit, false, 'unblocking must invalidate the cache');
  assert.deepEqual(r4.path, r2b.path, 'restored mesh restores the original path');
});

test('unreachable: goal walled in by blocked triangles', () => {
  const { mesh, cellTriangles } = flatSquare(5);
  mesh.setBlocked(trianglesOfCells(cellTriangles, [[2, 0], [2, 1], [2, 2], [2, 3], [2, 4]]));
  const result = mesh.findPath({
    start: [0.5, 0, 0.5],
    goal: [4.5, 0, 0.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(result.status, 'no-path');
  assert.equal(result.reason, 'unreachable');
});

test('edge snapping: off-mesh endpoints project within maxSnapDistance only', () => {
  const { mesh } = flatSquare(3);
  const goal = [1.5, 0, 1.5];

  const ok = mesh.findPath({ start: [-0.4, 0, 1.5], goal, maxSnapDistance: 0.5 });
  assert.equal(ok.status, 'success');
  assertPointClose(ok.path[0], [0, 0, 1.5], 1e-9, assert, 'snapped start');
  assert.ok(ok.diagnostics.snap.start <= 0.4 + 1e-9);

  const tooFar = mesh.findPath({ start: [-0.4, 0, 1.5], goal, maxSnapDistance: 0.3 });
  assert.equal(tooFar.status, 'no-path');
  assert.equal(tooFar.reason, 'start-out-of-range');

  const okGoal = mesh.findPath({ start: goal, goal: [3.4, 0, 1.5], maxSnapDistance: 0.5 });
  assert.equal(okGoal.status, 'success');
  assertPointClose(okGoal.path[okGoal.path.length - 1], [3, 0, 1.5], 1e-9, assert, 'snapped goal');

  const farGoal = mesh.findPath({ start: goal, goal: [3.4, 0, 1.5], maxSnapDistance: 0.3 });
  assert.equal(farGoal.status, 'no-path');
  assert.equal(farGoal.reason, 'goal-out-of-range');
});

test('blocked endpoints report start-blocked / goal-blocked', () => {
  const { mesh, cellTriangles } = flatSquare(3);
  mesh.setBlocked(trianglesOfCells(cellTriangles, [[0, 0], [1, 0], [0, 1]]));

  const startBlocked = mesh.findPath({
    start: [0.5, 0, 0.5],
    goal: [2.5, 0, 2.5],
    maxSnapDistance: 0.4,
  });
  assert.equal(startBlocked.status, 'no-path');
  assert.equal(startBlocked.reason, 'start-blocked');

  const goalBlocked = mesh.findPath({
    start: [2.5, 0, 2.5],
    goal: [0.5, 0, 0.5],
    maxSnapDistance: 0.4,
  });
  assert.equal(goalBlocked.status, 'no-path');
  assert.equal(goalBlocked.reason, 'goal-blocked');
});

test('start and goal on the same triangle give a direct two-point path', () => {
  const { mesh } = flatSquare(1);
  const result = mesh.findPath({
    start: [0.3, 0, 0.1],
    goal: [0.8, 0, 0.5],
    maxSnapDistance: 0.5,
  });
  assert.equal(result.status, 'success');
  assert.equal(result.path.length, 2);
  assert.equal(result.diagnostics.turnPoints, 0);
  const expected = Math.hypot(0.8 - 0.3, 0, 0.5 - 0.1);
  assert.ok(Math.abs(result.cost - expected) < 1e-9);
});

test('determinism: repeated and fresh runs produce byte-identical results', () => {
  const query = { start: [1, 0, 0.5], goal: [8, 0, 0.5], maxSnapDistance: 0.5 };
  const signatures = new Set();
  for (let i = 0; i < 20; i++) {
    const { mesh } = twoCorridors({ corridorACost: 3 });
    const result = mesh.findPath(query);
    signatures.add(JSON.stringify({ path: result.path, cost: result.cost }));
  }
  assert.equal(signatures.size, 1, 'fresh meshes must yield identical paths');

  const { mesh } = twoCorridors({ corridorACost: 3 });
  const first = mesh.findPath(query);
  for (let i = 0; i < 10; i++) {
    const again = mesh.findPath(query);
    assert.equal(JSON.stringify(again.path), JSON.stringify(first.path));
    assert.equal(again.cost, first.cost);
  }
});
