'use strict';

const {
  distance,
  distance2D,
  closestPointOnTriangle,
  barycentric2D,
  pointInTriangle2D,
} = require('./geometry');
const { MinHeap } = require('./heap');
const { funnel, triarea2 } = require('./funnel');

const CONTAIN_EPS = 1e-7;
const DEDUP_EPS = 1e-9;

const REASON = {
  START_OUT_OF_RANGE: 'start-out-of-range',
  GOAL_OUT_OF_RANGE: 'goal-out-of-range',
  START_BLOCKED: 'start-blocked',
  GOAL_BLOCKED: 'goal-blocked',
  UNREACHABLE: 'unreachable',
};

/**
 * Find a path across a NavMesh.
 * @param {NavMesh} mesh
 * @param {object} options
 * @param {number[]} options.start 3D start point (may be off-mesh).
 * @param {number[]} options.goal 3D goal point (may be off-mesh).
 * @param {number} [options.maxSnapDistance] Max distance to project
 *   off-mesh endpoints onto the nearest walkable triangle.
 */
function findPath(mesh, options) {
  const opts = options || {};
  const start = opts.start;
  const goal = opts.goal;
  assertPoint(start, 'start');
  assertPoint(goal, 'goal');
  const maxSnapDistance = opts.maxSnapDistance === undefined ? 0.5 : opts.maxSnapDistance;
  if (!Number.isFinite(maxSnapDistance) || maxSnapDistance < 0) {
    throw new TypeError('maxSnapDistance must be a finite number >= 0');
  }

  const cacheKey = `${mesh.version}|${start}|${goal}|${maxSnapDistance}`;
  const cached = mesh._cache.get(cacheKey);
  if (cached) {
    const clone = deepClone(cached);
    clone.diagnostics.cacheHit = true;
    return clone;
  }

  const result = computePath(mesh, start, goal, maxSnapDistance);
  mesh._cache.set(cacheKey, deepClone(result));
  return result;
}

function computePath(mesh, start, goal, maxSnapDistance) {
  const diagnostics = {
    version: mesh.version,
    cacheHit: false,
    snap: { start: null, goal: null },
    startTriangle: -1,
    goalTriangle: -1,
    corridor: [],
    visited: 0,
    turnPoints: 0,
  };

  const startSnap = snapToWalkable(mesh, start, maxSnapDistance);
  if (!startSnap.ok) {
    diagnostics.snap.start = startSnap.distance;
    return noPath(
      startSnap.reason === 'blocked' ? REASON.START_BLOCKED : REASON.START_OUT_OF_RANGE,
      diagnostics,
    );
  }
  diagnostics.snap.start = startSnap.distance;
  diagnostics.startTriangle = startSnap.triangle;

  const goalSnap = snapToWalkable(mesh, goal, maxSnapDistance);
  if (!goalSnap.ok) {
    diagnostics.snap.goal = goalSnap.distance;
    return noPath(
      goalSnap.reason === 'blocked' ? REASON.GOAL_BLOCKED : REASON.GOAL_OUT_OF_RANGE,
      diagnostics,
    );
  }
  diagnostics.snap.goal = goalSnap.distance;
  diagnostics.goalTriangle = goalSnap.triangle;

  const startPoint = startSnap.point;
  const goalPoint = goalSnap.point;

  if (startSnap.triangle === goalSnap.triangle) {
    const tri = startSnap.triangle;
    diagnostics.corridor = [tri];
    return success(
      [startPoint, goalPoint],
      distance(startPoint, goalPoint) * mesh.costs[tri],
      diagnostics,
    );
  }

  const search = astar(mesh, startSnap.triangle, goalSnap.triangle);
  diagnostics.visited = search.visited;
  if (!search.corridor) {
    return noPath(REASON.UNREACHABLE, diagnostics);
  }
  const corridor = search.corridor;
  diagnostics.corridor = corridor.slice();

  const start2 = project(mesh, startPoint);
  const goal2 = project(mesh, goalPoint);

  // Build the portal chain (left/right in walking direction).
  const portalLeft = [];
  const portalRight = [];
  for (let i = 0; i < corridor.length - 1; i++) {
    const t = corridor[i];
    const next = corridor[i + 1];
    let edge = -1;
    for (let e = 0; e < 3; e++) {
      if (mesh.adjacency[t][e] === next) {
        edge = e;
        break;
      }
    }
    const tri = mesh.triangles[t];
    const p = mesh.vertices2D[tri[edge]];
    const q = mesh.vertices2D[tri[(edge + 1) % 3]];
    const ci = mesh.centroids2D[t];
    const cj = mesh.centroids2D[next];
    if (triarea2(ci, cj, p) < 0) {
      portalLeft.push(p);
      portalRight.push(q);
    } else {
      portalLeft.push(q);
      portalRight.push(p);
    }
  }

  const path2 = dedupe(funnel(start2, goal2, portalLeft, portalRight));
  const path3 = path2.map((p) => lift(mesh, corridor, p));
  diagnostics.turnPoints = Math.max(0, path3.length - 2);

  return success(path3, pathCost(mesh, corridor, path2, path3), diagnostics);
}

function astar(mesh, startTri, goalTri) {
  const count = mesh.triangles.length;
  const g = new Float64Array(count).fill(Infinity);
  const cameFrom = new Int32Array(count).fill(-1);
  const closed = new Uint8Array(count);
  const heap = new MinHeap();
  const goalCentroid = mesh.centroids[goalTri];
  let counter = 0;
  let visited = 0;

  g[startTri] = 0;
  heap.push(startTri, distance(mesh.centroids[startTri], goalCentroid) * mesh.minCost, counter++);

  while (heap.size > 0) {
    const { node } = heap.pop();
    if (closed[node]) continue;
    closed[node] = 1;
    visited++;
    if (node === goalTri) break;
    for (let e = 0; e < 3; e++) {
      const nb = mesh.adjacency[node][e];
      if (nb < 0 || closed[nb] || mesh.blocked.has(nb)) continue;
      const ng = g[node] + mesh.costs[nb] * distance(mesh.centroids[node], mesh.centroids[nb]);
      if (ng < g[nb] - 1e-12) {
        g[nb] = ng;
        cameFrom[nb] = node;
        heap.push(nb, ng + distance(mesh.centroids[nb], goalCentroid) * mesh.minCost, counter++);
      }
    }
  }

  if (!closed[goalTri]) return { corridor: null, visited };

  const corridor = [goalTri];
  let cur = goalTri;
  while (cur !== startTri) {
    cur = cameFrom[cur];
    corridor.push(cur);
  }
  corridor.reverse();
  return { corridor, visited };
}

// Nearest walkable (non-blocked) triangle within maxDist. Also distinguishes
// "nearest triangle is blocked" from "nothing within range" for diagnostics.
function snapToWalkable(mesh, point, maxDist) {
  let bestAny = null;
  let bestOpen = null;
  for (let t = 0; t < mesh.triangles.length; t++) {
    const tri = mesh.triangles[t];
    const cp = closestPointOnTriangle(
      point,
      mesh.vertices[tri[0]],
      mesh.vertices[tri[1]],
      mesh.vertices[tri[2]],
    );
    const d = distance(point, cp);
    if (!bestAny || d < bestAny.distance - 1e-12) {
      bestAny = { point: cp, triangle: t, distance: d };
    }
    if (!mesh.blocked.has(t) && (!bestOpen || d < bestOpen.distance - 1e-12)) {
      bestOpen = { point: cp, triangle: t, distance: d };
    }
  }
  if (bestOpen && bestOpen.distance <= maxDist) {
    return { ok: true, ...bestOpen };
  }
  if (bestAny && bestAny.distance <= maxDist && mesh.blocked.has(bestAny.triangle)) {
    return { ok: false, reason: 'blocked', distance: bestAny.distance };
  }
  return { ok: false, reason: 'out-of-range', distance: bestAny ? bestAny.distance : Infinity };
}

function project(mesh, p) {
  return [p[mesh.axes[0]], p[mesh.axes[1]]];
}

// Lift a 2D projected point back to 3D on the corridor triangle that
// contains it (barycentric interpolation of the original vertices).
function lift(mesh, corridor, p2) {
  for (const t of corridor) {
    const tri2 = mesh.triangles2D[t];
    if (pointInTriangle2D(p2, tri2[0], tri2[1], tri2[2], CONTAIN_EPS)) {
      const [u, v, w] = barycentric2D(p2, tri2[0], tri2[1], tri2[2]);
      const tri = mesh.triangles[t];
      const a = mesh.vertices[tri[0]];
      const b = mesh.vertices[tri[1]];
      const c = mesh.vertices[tri[2]];
      return [
        u * a[0] + v * b[0] + w * c[0],
        u * a[1] + v * b[1] + w * c[1],
        u * a[2] + v * b[2] + w * c[2],
      ];
    }
  }
  // Defensive fallback: nearest corridor centroid (should not happen).
  let best = corridor[0];
  let bestD = Infinity;
  for (const t of corridor) {
    const d = distance2D(p2, mesh.centroids2D[t]);
    if (d < bestD) {
      bestD = d;
      best = t;
    }
  }
  return mesh.centroids[best].slice();
}

// Cost of the final polyline: segment length times the region cost of the
// corridor triangle containing the segment midpoint.
function pathCost(mesh, corridor, path2, path3) {
  let cost = 0;
  for (let i = 0; i + 1 < path3.length; i++) {
    const len = distance(path3[i], path3[i + 1]);
    const mid = [
      (path2[i][0] + path2[i + 1][0]) / 2,
      (path2[i][1] + path2[i + 1][1]) / 2,
    ];
    let tri = -1;
    for (const t of corridor) {
      const tri2 = mesh.triangles2D[t];
      if (pointInTriangle2D(mid, tri2[0], tri2[1], tri2[2], CONTAIN_EPS)) {
        tri = t;
        break;
      }
    }
    cost += len * (tri >= 0 ? mesh.costs[tri] : mesh.minCost);
  }
  return cost;
}

function dedupe(path) {
  const out = [path[0]];
  for (let i = 1; i < path.length; i++) {
    if (distance2D(path[i], out[out.length - 1]) > DEDUP_EPS) {
      out.push(path[i]);
    }
  }
  return out;
}

function success(path, cost, diagnostics) {
  return { status: 'success', reason: null, path, cost, diagnostics };
}

function noPath(reason, diagnostics) {
  return { status: 'no-path', reason, path: [], cost: null, diagnostics };
}

function assertPoint(p, name) {
  if (!Array.isArray(p) || p.length !== 3 || !p.every((v) => Number.isFinite(v))) {
    throw new TypeError(`${name} must be an array of 3 finite numbers`);
  }
}

function deepClone(value) {
  return JSON.parse(JSON.stringify(value));
}

module.exports = { findPath, REASON };
