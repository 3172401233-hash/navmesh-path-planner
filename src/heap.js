'use strict';

// Binary min-heap keyed by (priority, insertionCounter) so that pop order
// is fully deterministic for identical inputs.
class MinHeap {
  constructor() {
    this.items = [];
  }

  get size() {
    return this.items.length;
  }

  push(node, priority, counter) {
    const items = this.items;
    items.push({ node, priority, counter });
    let i = items.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (less(items[i], items[parent])) {
        const tmp = items[i];
        items[i] = items[parent];
        items[parent] = tmp;
        i = parent;
      } else {
        break;
      }
    }
  }

  pop() {
    const items = this.items;
    const top = items[0];
    const last = items.pop();
    if (items.length > 0) {
      items[0] = last;
      let i = 0;
      for (;;) {
        const left = i * 2 + 1;
        const right = left + 1;
        let smallest = i;
        if (left < items.length && less(items[left], items[smallest])) smallest = left;
        if (right < items.length && less(items[right], items[smallest])) smallest = right;
        if (smallest === i) break;
        const tmp = items[i];
        items[i] = items[smallest];
        items[smallest] = tmp;
        i = smallest;
      }
    }
    return top;
  }
}

function less(a, b) {
  if (a.priority !== b.priority) return a.priority < b.priority;
  return a.counter < b.counter;
}

module.exports = { MinHeap };
