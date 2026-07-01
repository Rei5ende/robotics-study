# Lecture 6: Introduction to Motion Planning

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

New module: **Motion Planning**. The previous module (Feedback Control) got the drone to *hover*; this module moves beyond hovering to **autonomous navigation through cluttered environments**.

> Where this sits in the pipeline: Planning decides *what path* to take (this module); Control makes the robot *follow* that path (previous module). Planning comes first.

**Today's focus:** discrete planning — discretize a continuous environment into a grid, then solve it as a **graph search** problem (BFS / DFS).

---

## 1. The Piano Mover's Problem

The canonical motion-planning problem: given a geometric (e.g., CAD) model of a piano and an apartment, find a path to move the piano from point A to point B **without colliding** with anything.

Key insight: **orientation matters**. You can't treat the piano as a sphere — squeezing through tight spaces may require rotating/tilting it. The geometry genuinely matters.

- Long history in robotics (since ~1980s).
- The problem is **PSPACE-complete** — a computational-complexity class meaning it needs *polynomial memory (space)* and is among the hardest in that class. Practically: don't expect exact, efficient algorithms. This motivates the *approximate* methods used throughout (grids, sampling).
- Today, "piano mover's problem" is synonymous with "motion planning."

> **PSPACE-complete vs completeness** — these are different ideas. PSPACE-complete is about *how hard the problem is* (complexity). "Completeness" (Section 6 below) is a property of an *algorithm* (does it find a path if one exists). Don't conflate them.

---

## 2. Two Considerations: Feasibility vs Optimality

As with control (stability vs optimality), planning has two goals:

| Goal | Meaning | Difficulty |
|------|---------|-----------|
| **Feasibility** | find *any* collision-free path from A to B | challenging by itself |
| **Optimality** | find the *best* path (by some criterion: distance, time, energy) | nice to have |

This lecture focuses **only on feasibility**. Optimality is deferred to Lecture 7 (A*).

---

## 3. Assumptions (for now)

1. **Geometry of environment and robot is given** (e.g., a CAD model).
2. **Any path can be executed by the robot** — big assumption; e.g., we allow omnidirectional motion for now (a real car can't move sideways). Relaxed a few lectures later.
3. **Any path can be perfectly followed** — not true in reality (wind, model error), but this is exactly where **feedback control** helps.

Assumption 3 is the concrete link between planning and control: planning produces a path, control tracks it.

---

## 4. Discretization

A continuous environment is hard to search directly, so we **chop it into a grid** and solve the discretized version. This lets us apply powerful **graph search** algorithms.

**From grid to graph (3 steps):**
1. Associate a **vertex** with each grid **cell**.
2. Connect vertices with **edges** based on 4- or 8-connectivity.
3. **Delete** vertices (and their edges) that are occupied by obstacles.

> **Common confusion:** a vertex corresponds to a **cell** (the empty square), NOT to the corner points where grid lines meet.

**Design choices:**

| Choice | Options | Trade-off |
|--------|---------|-----------|
| Grid type | uniform vs non-uniform | uniform is simplest |
| Connectivity | 4-connected (N/S/E/W) vs 8-connected (+ diagonals) | 8 allows diagonal motion |
| Resolution | coarse vs fine | finer = better approximation but harder (more computation); coarse may miss narrow gaps |

---

## 5. Graph Search

A **graph** is a collection of vertices (nodes) connected by edges. Edges define connectivity — the robot can move between two vertices only if an edge connects them. (We use undirected graphs here.)

**Vertex labeling convention:** `(i, j) = (x, y) = (column, row)`, indexed from 1 (not 0).
Example: A = (2,3), B = (5,5).

### Forward Search Algorithm

```
Q.Insert(A) and mark A as visited
while Q not empty do
    x ← Q.GetVertex()
    if x ∈ xG then               // xG is the goal set, e.g., {B}
        Return SUCCESS
    for all neighbors x' of x do
        if x' not visited then
            Parent(x') ← x        // record where we came from
            Mark x' as visited
            Q.Insert(x')
Return FAILURE
```

**Line-by-line:**
- Start by putting A in the queue and marking it visited.
- Loop while the queue has nodes to explore.
- `x ← Q.GetVertex()` pulls a node out — **this single line decides BFS vs DFS**.
- If `x` is the goal → SUCCESS.
- For each **unvisited** neighbor: record its parent, mark visited, add to queue.
- If the queue empties without reaching the goal → FAILURE (no path exists).

**Two/three key data structures:**
- `Q` — the queue of nodes still to explore.
- `visited` list — prevents re-exploring nodes.
- `Parent` map — used to reconstruct the path.

> **Why the `visited` check matters:** without it, already-explored nodes get re-added to the queue forever (A→B→A→B...), causing an **infinite loop**.

---

## 6. BFS vs DFS

The algorithm skeleton is **identical**; only `Q.GetVertex()` differs.

| Method | GetVertex rule | Data structure | Behavior |
|--------|----------------|----------------|----------|
| **BFS** (Breadth First) | FIFO (first-in-first-out) | **queue** | explores all paths of length k before length k+1 (expands outward in layers) |
| **DFS** (Depth First) | LIFO (last-in-first-out) | **stack** | dives deep along one path until a dead-end or goal, then backtracks |

**No general winner** — depends on the environment.
- **DFS** works well when the path is *long* and there are *few* ways to reach the goal.
- **BFS** is better when a shortest path is needed.

### BFS gives shortest paths (in a specific sense)

Even though we didn't explicitly optimize, **BFS returns paths that are optimal with respect to path length (number of steps)**. Because BFS finishes all length-k paths before length-(k+1), the first time it reaches B it has done so in the fewest steps.

> **Caveat:** this is optimal only for *path length / step count*. For other criteria (distance with varying edge costs, time, energy), BFS is *not* optimal — motivating Dijkstra / A* in Lecture 7.

---

## 7. Path Reconstruction (Backtracking)

The algorithm only signals "reached B" — the actual path is recovered from the **Parent** records.

**Idea (breadcrumbs):** each node stores who it came from. From the goal, follow parents backward until reaching the start.

```
B(5,5) → Parent → (4,5) → (3,5) → (2,5) → (1,5) → (1,4) → (1,3) → A(2,3)
```
Reverse it to get the path A → B:
```
A(2,3) → (1,3) → (1,4) → (1,5) → (2,5) → (3,5) → (4,5) → B(5,5)
```

```python
def reconstruct_path(parent, goal, start):
    path = []
    x = goal
    while x != start:
        path.append(x)
        x = parent[x]     # walk backward via parents
    path.append(start)
    path.reverse()        # flip to start → goal order
    return path
```

This is why recording `Parent` *during* search is essential — otherwise we'd know we arrived but not how.

---

## 8. Completeness

**Completeness:** an algorithm is *complete* if it finds a path whenever one exists, and otherwise terminates (in finite time) reporting no path.

- BFS and DFS are **complete for the discretized (graph) problem** — if a path exists in the graph, they will find it.
- They are **not necessarily complete for the continuous problem**. If the grid resolution is too **coarse**, a narrow gap in the real environment can get swallowed into obstacle cells, so the discretized graph shows "no path" even though a continuous path exists.

The limitation comes from **discretization approximating the continuous space imperfectly**, not from the search algorithm itself.

---

## Concept Flow & What's Next

```
Piano mover's problem (hard: PSPACE-complete)
        ↓ approximate it
Discretize continuous space → grid → graph
        ↓ solve via graph search
Forward Search (BFS via queue / DFS via stack)  → feasibility only
        ↓ recover path via Parent backtracking
Next (Lec 7): bias search toward goal → Dijkstra / A* → optimality
```

Next lecture explores smarter `Q.GetVertex()` rules — e.g., prioritizing nodes closest (Euclidean distance) to the goal — leading to **Dijkstra and A***, and finally addressing **optimality**.

---

## Quick Reference

| Term | Meaning |
|------|---------|
| vertex / node | one grid **cell** |
| edge | connection between traversable cells |
| 4- / 8-connectivity | allowed moves (no diagonals / with diagonals) |
| Q | queue of nodes to explore |
| Parent(x') | the node from which x' was discovered (for backtracking) |
| BFS | FIFO / queue → shortest path in # steps |
| DFS | LIFO / stack → deep exploration |
| completeness | finds a path if one exists (holds for discretized problem) |
| PSPACE-complete | complexity class; motion planning is fundamentally hard |
