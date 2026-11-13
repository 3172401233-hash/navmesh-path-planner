# navmesh-path-planner

零依赖、不依赖任何渲染引擎的 Node.js 三维导航网格（navmesh）寻路库，用于游戏 NPC 在静态场景与临时阻挡下计算路径。基于 CommonJS，要求 Node.js >= 18，仅输出路径、代价与诊断信息，不包含 NPC 逻辑或可视化。

## 特性

- 输入三角形导航网格：顶点、三角形、共享边邻接（可自动生成或严格校验）、逐三角形区域代价、动态阻挡集合
- A* 在可通行三角形区域上搜索，堆结构带插入序号平局裁决，结果完全确定
- 漏斗算法（funnel）在走廊上生成最少转向点的折线路径
- 起点/终点可在网格外，按最大吸附距离投影到最近的可通行三角形
- 动态阻挡通过版本号使查询缓存失效，不会复用过期结果
- 所有非法输入（重复顶点、退化三角形、缺失/不一致邻接、非法代价、非流形边）都抛出带完整问题列表的 `NavMeshError`，不做静默处理

## 快速开始

```js
const { NavMesh } = require('navmesh-path-planner');

const mesh = new NavMesh({
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
  // adjacency 可省略（自动从共享边构建）；若提供则严格校验
  costs: [1, 2], // 每个三角形的通行代价倍率，省略则全部为 1
});

const result = mesh.findPath({
  start: [0.2, 0, 0.2],
  goal: [0.9, 0, 0.8],
  maxSnapDistance: 0.5,
});

// 动态阻挡：传入被阻挡的三角形索引
mesh.setBlocked([0]);
mesh.clearBlocked();
```

## 输入格式

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `vertices` | `number[][]` | 顶点数组，每项 `[x, y, z]`，不允许重复（1e-9 精度） |
| `triangles` | `number[][]` | 三角形数组，每项 3 个顶点索引，不允许退化（近零面积） |
| `adjacency` | `number[][]` | 可选。`adjacency[t][e]` 为与三角形 `t` 的第 `e` 条边（顶点 `e` → `(e+1)%3`）相邻的三角形索引，边界边为 `-1`。提供时会与真实共享边逐一比对：缺失、不一致、自引用、越界都会抛错 |
| `costs` | `number[]` | 可选。每个三角形的区域代价倍率，必须为有限正数 |

## 查询与结果

`mesh.findPath({ start, goal, maxSnapDistance })` 返回：

```js
{
  status: 'success' | 'no-path',
  reason: null | 'start-out-of-range' | 'goal-out-of-range'
        | 'start-blocked' | 'goal-blocked' | 'unreachable',
  path: [[x, y, z], ...],   // 转向点最少的折线，所有点都在网格内
  cost: number | null,      // 按所经过三角形区域代价加权的折线长度
  diagnostics: {
    version,                // 阻挡集合版本号
    cacheHit,               // 本次是否命中查询缓存
    snap: { start, goal },  // 端点吸附距离
    startTriangle, goalTriangle,
    corridor,               // A* 走廊（三角形索引序列）
    visited,                // A* 扩展的三角形数量
    turnPoints,             // 中间转向点数量
  },
}
```

无路径原因：

- `start-out-of-range` / `goal-out-of-range`：端点在 `maxSnapDistance` 内没有任何可通行三角形
- `start-blocked` / `goal-blocked`：距离内最近的三角形被阻挡（且没有可通行的替代）
- `unreachable`：端点吸附成功，但区域不连通（断开岛屿）或被阻挡完全隔断

## 动态阻挡与缓存

`setBlocked` / `addBlocked` / `removeBlocked` / `clearBlocked` 会递增内部版本号并清空查询缓存。相同查询在阻挡不变时命中缓存（`diagnostics.cacheHit === true`），阻挡变化后同一查询必定重新计算，不会返回过期路径。

## 测试

```sh
npm test   # node --test
```

测试场景（`test/`，网格生成器在 `support/helpers.js`）：

- L 形走廊：漏斗在内凹角产生唯一转向点
- 凹形（U 形）区域：路径绕开凹陷，所有路径点与采样线段都在网格内
- 多出口：优先选择更短的出口走廊
- 区域代价：昂贵捷径被绕开，选择便宜的绕行路线
- 断开岛屿与阻挡围墙：返回 `unreachable`
- 动态封路：阻挡变化后缓存失效并改道，恢复后回到原路径
- 网格边缘吸附：距离内投影成功，超距返回明确原因
- 阻挡端点：`start-blocked` / `goal-blocked`
- 错误输入：重复顶点、退化三角形、缺失/不一致邻接、非法代价、非流形边、非法查询参数
- 确定性：全新构建与缓存命中下重复运行结果逐字节一致

## 已知边界

- 漏斗算法在主导投影平面（按面积加权平均法线选取）上进行，适用于基本水平的可行走表面；包含接近竖直墙面的网格不适用
- A* 的区域间代价以三角形质心距离 × 区域代价估算，最终路径是"走廊内最优"；在密集网格中走廊选择可能存在等价平局（结果仍确定，但不一定全局几何最短）
- 阻挡粒度为整个三角形，不支持三角形内部的部分阻挡
- 库是单线程同步实现，面向静态网格 + 动态阻挡集合；网格拓扑变化需要重新构建 `NavMesh`
