'use strict';

const { NavMesh } = require('../src/index');

// Builds a NavMesh from a set of unit grid cells on the x-z plane (y = 0).
// Cell [cx, cz] covers x in [cx, cx+1], z in [cz, cz+1] and is split into
// two triangles. Vertices are shared between adjacent cells.
//
// Returns { mesh, cellTriangles } where cellTriangles maps "cx,cz" to the
// two triangle indices of that cell.
function gridMesh({ cells, cellCost, blocked }) {
  const vertexIds = new Map();
  const vertices = [];
  const vid = (x, z) => {
    const key = `${x},${z}`;
    if (!vertexIds.has(key)) {
      vertexIds.set(key, vertices.length);
      vertices.push([x, 0, z]);
    }
    return vertexIds.get(key);
  };

  const triangles = [];
  const costs = [];
  const cellTriangles = new Map();
  for (const [cx, cz] of cells) {
    const v00 = vid(cx, cz);
    const v10 = vid(cx + 1, cz);
    const v11 = vid(cx + 1, cz + 1);
    const v01 = vid(cx, cz + 1);
    const cost = cellCost ? cellCost(cx, cz) : 1;
    const base = triangles.length;
    triangles.push([v00, v10, v11]);
    costs.push(cost);
    triangles.push([v00, v11, v01]);
    costs.push(cost);
    cellTriangles.set(`${cx},${cz}`, [base, base + 1]);
  }

  const mesh = new NavMesh({ vertices, triangles, costs });
  if (blocked && blocked.length > 0) mesh.setBlocked(blocked);
  return { mesh, cellTriangles };
}

function cellRange(x0, x1, z0, z1) {
  const cells = [];
  for (let x = x0; x <= x1; x++) {
    for (let z = z0; z <= z1; z++) {
      cells.push([x, z]);
    }
  }
  return cells;
}

function trianglesOfCells(cellTriangles, cells) {
  const out = [];
  for (const [cx, cz] of cells) {
    const pair = cellTriangles.get(`${cx},${cz}`);
    if (!pair) throw new Error(`no such cell ${cx},${cz}`);
    out.push(pair[0], pair[1]);
  }
  return out;
}

// L-shaped corridor hand-triangulated with 5 triangles, so the corridor
// between any two points is unique. Horizontal arm x in [0,10], z in [0,2];
// vertical arm x in [8,10], z in [2,10]. Reflex (inner) corner at (8, 2).
function lCorridor() {
  const vertices = [
    [0, 0, 0], // 0: A
    [10, 0, 0], // 1: B
    [10, 0, 2], // 2: C
    [8, 0, 2], // 3: D (reflex corner)
    [0, 0, 2], // 4: E
    [10, 0, 10], // 5: F
    [8, 0, 10], // 6: G
  ];
  const triangles = [
    [0, 1, 3], // A, B, D
    [1, 2, 3], // B, C, D
    [0, 3, 4], // A, D, E
    [3, 2, 5], // D, C, F
    [3, 5, 6], // D, F, G
  ];
  return { mesh: new NavMesh({ vertices, triangles }) };
}

// U-shaped (concave) region: bottom bar plus two columns.
// Reflex corners at (2, 2) and (5, 2).
function uShape() {
  const cells = [
    ...cellRange(0, 6, 0, 1), // bottom bar: x in [0,7], z in [0,2]
    ...cellRange(0, 1, 2, 5), // left column: x in [0,2], z in [2,6]
    ...cellRange(5, 6, 2, 5), // right column: x in [5,7], z in [2,6]
  ];
  return gridMesh({ cells });
}

// Two rooms connected by two parallel corridors:
//   corridor A: x in [3,7], z in [0,2] (short, direct)
//   corridor B: x in [3,7], z in [4,5] (detour)
function twoCorridors({ corridorACost = 1 } = {}) {
  const corridorA = cellRange(3, 6, 0, 1);
  const corridorB = cellRange(3, 6, 4, 4);
  const cells = [
    ...cellRange(0, 2, 0, 4), // start room: x in [0,3], z in [0,5]
    ...cellRange(7, 9, 0, 4), // goal room: x in [7,10], z in [0,5]
    ...corridorA,
    ...corridorB,
  ];
  const aCells = new Set(corridorA.map(([x, z]) => `${x},${z}`));
  const built = gridMesh({
    cells,
    cellCost: (cx, cz) => (aCells.has(`${cx},${cz}`) ? corridorACost : 1),
  });
  return {
    ...built,
    corridorACells: corridorA,
    corridorBCells: corridorB,
  };
}

// Two disconnected islands.
function islands() {
  const cells = [...cellRange(0, 2, 0, 2), ...cellRange(5, 7, 0, 2)];
  return gridMesh({ cells });
}

// Simple flat square, x/z in [0, size].
function flatSquare(size = 3) {
  return gridMesh({ cells: cellRange(0, size - 1, 0, size - 1) });
}

function pathLength(path) {
  let len = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    len += Math.hypot(
      path[i + 1][0] - path[i][0],
      path[i + 1][1] - path[i][1],
      path[i + 1][2] - path[i][2],
    );
  }
  return len;
}

function assertPointClose(actual, expected, eps, assert, message) {
  for (let i = 0; i < 3; i++) {
    assert.ok(
      Math.abs(actual[i] - expected[i]) <= eps,
      `${message || 'point'}: coordinate ${i} expected ${expected[i]}, got ${actual[i]}`,
    );
  }
}

// Every path point must lie on the mesh (snap distance ~ 0).
function assertPathOnMesh(mesh, path, assert) {
  for (const p of path) {
    const snap = mesh.snapPoint(p);
    assert.ok(
      snap.distance <= 1e-6,
      `path point [${p.join(', ')}] is ${snap.distance} away from the mesh`,
    );
  }
}

// Every segment of the path must stay on the mesh (sampled).
function assertSegmentsOnMesh(mesh, path, assert) {
  for (let i = 0; i + 1 < path.length; i++) {
    for (let s = 1; s < 10; s++) {
      const t = s / 10;
      const p = [
        path[i][0] + (path[i + 1][0] - path[i][0]) * t,
        path[i][1] + (path[i + 1][1] - path[i][1]) * t,
        path[i][2] + (path[i + 1][2] - path[i][2]) * t,
      ];
      const snap = mesh.snapPoint(p);
      assert.ok(
        snap.distance <= 1e-6,
        `segment ${i} sample ${t} at [${p.join(', ')}] is ${snap.distance} away from the mesh`,
      );
    }
  }
}

module.exports = {
  gridMesh,
  cellRange,
  trianglesOfCells,
  lCorridor,
  uShape,
  twoCorridors,
  islands,
  flatSquare,
  pathLength,
  assertPointClose,
  assertPathOnMesh,
  assertSegmentsOnMesh,
};
