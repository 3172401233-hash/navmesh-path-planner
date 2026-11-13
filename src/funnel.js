'use strict';

// Signed area used by the funnel (same convention as Mikko Mononen's
// "simple stupid funnel"): positive when c is clockwise of a->b in the
// 2D projection plane.
function triarea2(a, b, c) {
  const ax = b[0] - a[0];
  const ay = b[1] - a[1];
  const bx = c[0] - a[0];
  const by = c[1] - a[1];
  return bx * ay - ax * by;
}

/**
 * Funnel algorithm over a portal chain.
 * @param {number[]} start 2D start point.
 * @param {number[]} goal 2D goal point.
 * @param {number[][]} portalLeft Left endpoint of each portal (walking order).
 * @param {number[][]} portalRight Right endpoint of each portal.
 * @returns {number[][]} Smoothed 2D path with minimal turn points.
 */
function funnel(start, goal, portalLeft, portalRight) {
  const lefts = [start, ...portalLeft, goal];
  const rights = [start, ...portalRight, goal];
  const n = lefts.length;

  const path = [start];
  let apex = start;
  let left = start;
  let right = start;
  let apexIndex = 0;
  let leftIndex = 0;
  let rightIndex = 0;

  for (let i = 1; i < n; i++) {
    // Update right vertex.
    const r = rights[i];
    if (triarea2(apex, right, r) <= 0) {
      if (apex === right || triarea2(apex, left, r) > 0) {
        right = r;
        rightIndex = i;
      } else {
        // Right crossed left: left becomes a corner.
        path.push(left);
        apex = left;
        apexIndex = leftIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        i = apexIndex;
        continue;
      }
    }
    // Update left vertex.
    const l = lefts[i];
    if (triarea2(apex, left, l) >= 0) {
      if (apex === left || triarea2(apex, right, l) < 0) {
        left = l;
        leftIndex = i;
      } else {
        // Left crossed right: right becomes a corner.
        path.push(right);
        apex = right;
        apexIndex = rightIndex;
        left = apex;
        right = apex;
        leftIndex = apexIndex;
        rightIndex = apexIndex;
        i = apexIndex;
        continue;
      }
    }
  }
  path.push(goal);
  return path;
}

module.exports = { funnel, triarea2 };
