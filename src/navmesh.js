'use strict';

const {
  sub,
  length,
  distance,
  triangleNormal,
  closestPointOnTriangle,
} = require('./geometry');

const DUPLICATE_VERTEX_EPS = 1e-9;
const DEGENERATE_AREA_EPS = 1e-12;

class NavMeshError extends Error {
  constructor(issues) {
    const list = Array.isArray(issues) ? issues : [issues];
    super(`Invalid navmesh (${list.length} issue${list.length === 1 ? '' : 's'}):\n - ${list.join('\n - ')}`);
    this.name = 'NavMeshError';
    this.issues = list;
  }
}

class NavMesh {
  /**
   * @param {object} def
   * @param {number[][]} def.vertices  Array of [x, y, z].
   * @param {number[][]} def.triangles Array of [i, j, k] vertex indices.
   * @param {number[][]} [def.adjacency] Optional per-triangle neighbor indices,
   *   adjacency[t][e] is the triangle sharing the edge from vertex e to
   *   vertex (e+1)%3 of triangle t, or -1 for a border edge. When provided it
   *   is strictly validated against the actual shared edges.
   * @param {number[]} [def.costs] Optional per-triangle traversal cost
   *   multiplier (finite, > 0). Defaults to 1 for every triangle.
   */
  constructor(def) {
    if (!def || typeof def !== 'object') {
      throw new NavMeshError(['navmesh definition must be an object']);
    }
    const issues = [];
    const vertices = validateVertices(def.vertices, issues);
    const triangles = validateTriangles(def.triangles, vertices ? vertices.length : 0, issues);
    const costs = validateCosts(def.costs, triangles ? triangles.length : 0, issues);
    if (vertices && triangles) checkDegenerate(vertices, triangles, issues);

    if (issues.length > 0) throw new NavMeshError(issues);

    this.vertices = vertices;
    this.triangles = triangles;
    this.costs = costs;

    const built = buildAdjacency(triangles, issues);
    if (def.adjacency !== undefined && def.adjacency !== null) {
      validateAdjacency(def.adjacency, triangles, built, issues);
    }
    if (issues.length > 0) throw new NavMeshError(issues);

    this.adjacency = built;

    // Derived geometry.
    this.centroids = triangles.map((tri) => {
      const a = vertices[tri[0]];
      const b = vertices[tri[1]];
      const c = vertices[tri[2]];
      return [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    });

    // Pick the projection plane by dropping the dominant axis of the
    // area-weighted average absolute normal.
    const acc = [0, 0, 0];
    for (const tri of triangles) {
      const n = triangleNormal(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]);
      acc[0] += Math.abs(n[0]);
      acc[1] += Math.abs(n[1]);
      acc[2] += Math.abs(n[2]);
    }
    const drop = acc[0] >= acc[1] && acc[0] >= acc[2] ? 0 : acc[1] >= acc[2] ? 1 : 2;
    this.dropAxis = drop;
    this.axes = drop === 0 ? [1, 2] : drop === 1 ? [0, 2] : [0, 1];
    this.vertices2D = vertices.map((v) => [v[this.axes[0]], v[this.axes[1]]]);
    this.centroids2D = this.centroids.map((c) => [c[this.axes[0]], c[this.axes[1]]]);
    this.triangles2D = triangles.map((tri) => [
      this.vertices2D[tri[0]],
      this.vertices2D[tri[1]],
      this.vertices2D[tri[2]],
    ]);

    this.minCost = Math.min(...costs);

    this.blocked = new Set();
    this.version = 0;
    this._cache = new Map();
  }

  /** Replace the set of blocked triangle indices. */
  setBlocked(indices) {
    const next = new Set();
    for (const idx of indices) {
      if (!Number.isInteger(idx) || idx < 0 || idx >= this.triangles.length) {
        throw new NavMeshError([`blocked triangle index out of range: ${idx}`]);
      }
      next.add(idx);
    }
    this.blocked = next;
    this._bumpVersion();
  }

  addBlocked(indices) {
    for (const idx of indices) {
      if (!Number.isInteger(idx) || idx < 0 || idx >= this.triangles.length) {
        throw new NavMeshError([`blocked triangle index out of range: ${idx}`]);
      }
      this.blocked.add(idx);
    }
    this._bumpVersion();
  }

  removeBlocked(indices) {
    for (const idx of indices) this.blocked.delete(idx);
    this._bumpVersion();
  }

  clearBlocked() {
    this.blocked.clear();
    this._bumpVersion();
  }

  isBlocked(triangleIndex) {
    return this.blocked.has(triangleIndex);
  }

  _bumpVersion() {
    this.version += 1;
    this._cache.clear();
  }

  /**
   * Closest point on the whole mesh (blocked triangles included).
   * @returns {{point:number[], triangle:number, distance:number, blocked:boolean}}
   */
  snapPoint(point) {
    assertPoint(point, 'point');
    let best = null;
    for (let t = 0; t < this.triangles.length; t++) {
      const tri = this.triangles[t];
      const cp = closestPointOnTriangle(
        point,
        this.vertices[tri[0]],
        this.vertices[tri[1]],
        this.vertices[tri[2]],
      );
      const d = distance(point, cp);
      if (!best || d < best.distance - 1e-12) {
        best = { point: cp, triangle: t, distance: d };
      }
    }
    best.blocked = this.blocked.has(best.triangle);
    return best;
  }

  findPath(options) {
    return require('./findpath').findPath(this, options);
  }
}

function assertPoint(p, name) {
  if (!Array.isArray(p) || p.length !== 3 || !p.every((v) => Number.isFinite(v))) {
    throw new TypeError(`${name} must be an array of 3 finite numbers`);
  }
}

function validateVertices(vertices, issues) {
  if (!Array.isArray(vertices) || vertices.length === 0) {
    issues.push('vertices must be a non-empty array');
    return null;
  }
  const seen = new Map();
  for (let i = 0; i < vertices.length; i++) {
    const v = vertices[i];
    if (!Array.isArray(v) || v.length !== 3 || !v.every((c) => Number.isFinite(c))) {
      issues.push(`vertex ${i} must be an array of 3 finite numbers`);
      continue;
    }
    const key = v
      .map((c) => Math.round(c / DUPLICATE_VERTEX_EPS))
      .join(',');
    if (seen.has(key)) {
      issues.push(`duplicate vertex: index ${i} duplicates index ${seen.get(key)} at [${v.join(', ')}]`);
    } else {
      seen.set(key, i);
    }
  }
  return vertices.map((v) => [v[0], v[1], v[2]]);
}

function validateTriangles(triangles, vertexCount, issues) {
  if (!Array.isArray(triangles) || triangles.length === 0) {
    issues.push('triangles must be a non-empty array');
    return null;
  }
  for (let i = 0; i < triangles.length; i++) {
    const tri = triangles[i];
    if (!Array.isArray(tri) || tri.length !== 3) {
      issues.push(`triangle ${i} must be an array of 3 vertex indices`);
      continue;
    }
    for (const idx of tri) {
      if (!Number.isInteger(idx) || idx < 0 || idx >= vertexCount) {
        issues.push(`triangle ${i} references vertex index ${idx} out of range [0, ${vertexCount})`);
        break;
      }
    }
    if (tri[0] === tri[1] || tri[1] === tri[2] || tri[0] === tri[2]) {
      issues.push(`triangle ${i} reuses a vertex index (degenerate)`);
    }
  }
  return triangles.map((t) => [t[0], t[1], t[2]]);
}

function validateCosts(costs, triangleCount, issues) {
  if (costs === undefined || costs === null) {
    return new Array(triangleCount).fill(1);
  }
  if (!Array.isArray(costs) || costs.length !== triangleCount) {
    issues.push(`costs must be an array of length ${triangleCount} (one per triangle)`);
    return new Array(triangleCount).fill(1);
  }
  for (let i = 0; i < costs.length; i++) {
    if (!Number.isFinite(costs[i]) || costs[i] <= 0) {
      issues.push(`cost of triangle ${i} must be a finite number > 0, got ${costs[i]}`);
    }
  }
  return costs.slice();
}

function edgeKey(a, b) {
  return a < b ? `${a}_${b}` : `${b}_${a}`;
}

// Builds adjacency from shared edges and reports non-manifold and
// degenerate-geometry issues.
function buildAdjacency(triangles, issues) {
  const edgeMap = new Map();
  for (let t = 0; t < triangles.length; t++) {
    const tri = triangles[t];
    for (let e = 0; e < 3; e++) {
      const key = edgeKey(tri[e], tri[(e + 1) % 3]);
      if (!edgeMap.has(key)) edgeMap.set(key, []);
      edgeMap.get(key).push({ tri: t, edge: e });
    }
  }
  const adjacency = triangles.map(() => [-1, -1, -1]);
  for (const [key, list] of edgeMap) {
    if (list.length > 2) {
      issues.push(`non-manifold edge ${key.replace('_', '-')} shared by ${list.length} triangles`);
      continue;
    }
    if (list.length === 2) {
      const [a, b] = list;
      adjacency[a.tri][a.edge] = b.tri;
      adjacency[b.tri][b.edge] = a.tri;
    }
  }
  return adjacency;
}

function validateAdjacency(provided, triangles, built, issues) {
  if (!Array.isArray(provided) || provided.length !== triangles.length) {
    issues.push(`adjacency must be an array of length ${triangles.length} (one entry per triangle)`);
    return;
  }
  for (let t = 0; t < triangles.length; t++) {
    const row = provided[t];
    if (!Array.isArray(row) || row.length !== 3) {
      issues.push(`adjacency[${t}] must be an array of 3 entries`);
      continue;
    }
    for (let e = 0; e < 3; e++) {
      const n = row[e];
      if (!Number.isInteger(n) || n < -1 || n >= triangles.length) {
        issues.push(`adjacency[${t}][${e}] has invalid triangle index ${n}`);
        continue;
      }
      if (n === t) {
        issues.push(`adjacency[${t}][${e}] references the triangle itself`);
        continue;
      }
      const expected = built[t][e];
      if (n === -1 && expected !== -1) {
        issues.push(
          `missing adjacency: triangle ${t} edge ${e} shares an edge with triangle ${expected} but is marked -1`,
        );
      } else if (n !== -1 && expected === -1) {
        issues.push(
          `inconsistent shared edge: adjacency[${t}][${e}] = ${n} but triangles ${t} and ${n} do not share that edge`,
        );
      } else if (n !== expected) {
        issues.push(
          `inconsistent shared edge: adjacency[${t}][${e}] = ${n} but the actual neighbor on that edge is ${expected}`,
        );
      }
    }
  }
}

// Degenerate-triangle check (near-zero area).
function checkDegenerate(vertices, triangles, issues) {
  for (let i = 0; i < triangles.length; i++) {
    const tri = triangles[i];
    if (
      tri.some((idx) => !Number.isInteger(idx) || idx < 0 || idx >= vertices.length) ||
      tri[0] === tri[1] ||
      tri[1] === tri[2] ||
      tri[0] === tri[2]
    ) {
      continue; // already reported by structural validation
    }
    const [a, b, c] = tri.map((idx) => vertices[idx]);
    const n = triangleNormal(a, b, c);
    if (length(n) <= DEGENERATE_AREA_EPS) {
      issues.push(`triangle ${i} is degenerate (near-zero area)`);
    }
  }
}

module.exports = { NavMesh, NavMeshError };
