# Assignment 3 — Problem 1: BFS / DFS Hand-Trace

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Related lecture:** Lecture 6 (Discrete Motion Planning, BFS/DFS)

---

## Problem

Given the grid environment of Figure 1, the robot must get from **A = (5,3)** to **B = (6,2)**. Coordinates are `(x, y)` = `(column, row)`, indexed from 1, with `y` increasing upward.

- (a) Draw the graph (4-connectivity: up/down/left/right only).
- (b) Trace up to 5 iterations of BFS.
- (c) Trace 5 iterations of DFS.
- (d) Update the graph to allow diagonal moves (8-connectivity).
- (e) Repeat BFS for the 8-connected graph.

> The assignment explicitly notes: *"If the algorithm terminates before you complete 5 iterations, you can just stop there."* This foreshadows exactly what happens under 4-connectivity below.

---

## Grid reading (verification)

`■` = obstacle, `·` = free, A = (5,3), B = (6,2):

| y \ x | 1 | 2 | 3 | 4 | 5 | 6 |
|-------|---|---|---|---|---|---|
| **6** | · | · | ■ | · | · | · |
| **5** | · | · | ■ | · | ■ | ■ |
| **4** | ■ | ■ | ■ | ■ | · | · |
| **3** | · | · | ■ | · | **A** | ■ |
| **2** | · | ■ | ■ | ■ | ■ | **B** |
| **1** | · | · | · | · | · | · |

---

## Neighbor ordering convention

> **⚠️ Neighbor order must be fixed and stated explicitly.** The order in which neighbors are pushed onto `Q` changes *which node is popped on which iteration*. Two people using different orders can produce different traces (especially for DFS). To be reproducible, the order is fixed here as:
>
> **Clockwise: Left → NW → Up → NE → Right → SE → Down → SW.**
>
> (For the 4-connectivity parts (b)(c), only Left → Up → Right → Down apply — the same relative order with diagonals removed.)

---

## (a) Graph construction

Following Lecture 6's grid-to-graph procedure: one vertex per **free cell** (there are 22 free cells), edges connecting orthogonally adjacent free cells, and obstacle cells (with their edges) deleted.

The key structural fact for this problem: under 4-connectivity, **A's reachable set is only the four-cell pocket `{A=(5,3), (4,3), (5,4), (6,4)}`**. Every orthogonal exit from this pocket runs into an obstacle. B lies outside it.

---

## (b) BFS (4-connectivity) — FIFO queue

```
iter1: Q = [(5,3)] = [A]
       pop x = (5,3);  free unvisited neighbors: (4,3)[Left], (5,4)[Up]
       Q = [(4,3), (5,4)]

iter2: pop x = (4,3);  neighbors: (5,3)=A visited → nothing new
       Q = [(5,4)]

iter3: pop x = (5,4);  neighbors: (6,4)[Right] new, (5,3)=A visited
       Q = [(6,4)]

iter4: pop x = (6,4);  neighbors: (5,4) visited → nothing new
       Q = []   →   FAILURE (queue empty, goal never reached)
```

**Result:** the queue empties on iteration 4. A is trapped in the four-cell pocket; there is **no 4-connected path to B**.

---

## (c) DFS (4-connectivity) — LIFO stack

Same skeleton, same neighbor order, but `Q.GetVertex()` pops the **most recently pushed** node.

```
iter1: stack (top→bottom) = [A]
       pop A;  push (4,3)[Left], then (5,4)[Up]
       stack = [(5,4), (4,3)]          ← (5,4) on top (pushed last)

iter2: pop (5,4)   ← DIVERGES FROM BFS HERE (LIFO takes newest)
       push (6,4)[Right]; (5,3)=A visited
       stack = [(6,4), (4,3)]

iter3: pop (6,4);  neighbors all visited/obstacle → nothing new
       stack = [(4,3)]

iter4: pop (4,3);  neighbors: (5,3)=A visited → nothing new
       stack = []   →   FAILURE
```

**Result:** DFS also empties on iteration 4 — same pocket, visited in a different order, same failure.

### BFS vs DFS pop order

| iteration | BFS pops | DFS pops |
|-----------|----------|----------|
| 1 | A | A |
| 2 | (4,3) | **(5,4)** ← diverges |
| 3 | (5,4) | (6,4) |
| 4 | (6,4) | (4,3) |

> **💡 Why they diverge at iter 2:** A produces two neighbors (4,3) and (5,4). BFS pops the **first pushed** (FIFO → (4,3)); DFS pops the **last pushed** (LIFO → (5,4)). The final conclusion (failure) is identical, but the visitation order differs — a concrete instance of order-sensitivity in graph search.

---

## (d) 8-connectivity graph update

Add diagonal edges (NW, NE, SW, SE) between diagonally adjacent free cells. The decisive new edge:

$$A = (5,3) \xrightarrow{\text{SE diagonal}} (6,2) = B$$

**A's SE-diagonal neighbor is exactly B.** The single diagonal edge punches directly through the wall that isolated A under 4-connectivity.

---

## (e) BFS (8-connectivity) — FIFO queue

```
iter1: Q = [A=(5,3)]
       pop A;  free unvisited neighbors (clockwise order):
         (4,3)[Left], (5,4)[Up], (6,4)[NE], (6,2)[SE]=B
       Q = [(4,3), (5,4), (6,4), (6,2)]

iter2: pop (4,3);  all neighbors visited/obstacle → nothing new
       Q = [(5,4), (6,4), (6,2)]

iter3: pop (5,4);  new neighbor (4,5)[NW]; rest visited/obstacle
       Q = [(6,4), (6,2), (4,5)]

iter4: pop (6,4);  all neighbors visited/obstacle/out-of-range → nothing new
       Q = [(6,2), (4,5)]

iter5: pop (6,2) = B   →   SUCCESS
```

**Result:** BFS reaches the goal on iteration 5 (pop).

### Path recovery

B was discovered back in iter1 as A's SE-diagonal neighbor, so `parent(B) = A`. Backtracking gives:

$$\text{path: } A = (5,3) \xrightarrow{\text{SE diagonal}} B = (6,2)$$

a single diagonal step. (B is popped only on iter5 because other nodes sit ahead of it in the FIFO queue, but the recorded path is still the one-step diagonal.)

> **💡 Why the clockwise order didn't change the result here:** this graph is a *branch-free chain* — each iteration discovers at most one new free cell (iter1's four are all A's own neighbors, merely reordered among themselves; iter2–4 add 0 or 1 each). With no branch points, "which neighbor is pushed first" has little room to alter the pop order. On a graph with a genuine branch (one node revealing several new free cells at once), the clockwise order and a Left→Up→Right→Down order could produce different iteration-by-iteration results. Here they happen not to.

---

## Summary

| Connectivity | Algorithm | Result | Reason |
|--------------|-----------|--------|--------|
| 4-connected (b) | BFS | **FAILURE** at iter 4 | A trapped in 4-cell pocket `{A,(4,3),(5,4),(6,4)}` |
| 4-connected (c) | DFS | **FAILURE** at iter 4 | same pocket, different visitation order |
| 8-connected (e) | BFS | **SUCCESS** at iter 5 | SE-diagonal edge A→B breaks the isolation; path = 1 diagonal step |

> **💡 The takeaway — connectivity is not just an efficiency knob.** Lecture 6's design-choices table lists 4- vs 8-connectivity as a "trade-off" (diagonals allow more motion). This problem sharpens that: here the choice decides **feasibility itself**. Under 4-connectivity the problem is *infeasible* (A is walled off); adding diagonals makes it feasible with a one-step solution. The discretization/connectivity choice can create or destroy the existence of a path — echoing Lecture 6's deeper point that discretization only approximates the continuous problem.
