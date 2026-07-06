# Lecture 7: Optimal Discrete Planning

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lecture 6 solved **feasibility**: find *any* collision-free path from A to B using BFS or DFS. This lecture adds **optimality**: find the path that *minimizes total cost*.

> Where this sits in the pipeline: BFS/DFS give us a path that exists. Dijkstra and A\* give us the *best* path according to a cost criterion (distance, time, energy). Same Forward Search template — only `Q.GetVertex()` and one new branch change.

**Today's focus:** Dijkstra's algorithm, A\*, and why choosing the right objective matters (value alignment).

---

## 1. What Changes from Lecture 6

The General Forward Search template from Lecture 6 gains exactly one new branch:

```
for all neighbors x' of x do
    if x' not visited then
        Parent(x') <- x
        Mark x' as visited
        Q.Insert(x')
    else
        Resolve duplicate x'    ← NEW: cheaper path may exist
```

Two lines change across the whole template:

- `Q.GetVertex()` — now returns the node with the smallest estimated cost, not just FIFO/LIFO.
- `Resolve duplicate x'` — if a cheaper path to an already-visited node is discovered, update its cost and parent.

> **Key insight:** Dijkstra and A\* are *not* new algorithms. They are the same Forward Search with a smarter queue ordering and a cost-update rule.

---

## 2. Cost of a Plan

With every edge $e$, associate a cost:

$l(e)$ : cost of traversing edge $e$ (distance, time, energy, etc.)

**Total cost** of a plan = sum of $l(e)$ over all edges along the path from A to B. The goal is to find the path that **minimizes** this total.

> BFS counted only the number of steps (implicit cost of 1 per edge). Now edges can carry different costs — this is the critical difference.

---

## 3. Three Kinds of Vertices

Tracking vertex state is essential for understanding both algorithms:

| State | Meaning |
|---|---|
| **Unvisited** | Not yet encountered |
| **Alive** | In queue $Q$ — discovered but not yet expanded (open set) |
| **Dead** | Expanded, all neighbors processed — finalized (closed set) |

In Lecture 6 slides, Dead nodes were marked with a checkmark (✓). In A\* literature, Alive = open set, Dead = closed set.

---

## 4. Dijkstra's Algorithm

**Core idea:** always expand the alive vertex with the smallest cost-to-come from the start.

For each vertex $x$, define two quantities:

- $C^\star(x)$ — the true optimal cost-to-come from A to $x$
- $C(x)$ — the current best estimate; updated as the search progresses

**Initialization:**

$$C(A) = 0, \qquad C(x) = \infty \quad \text{for all other } x$$

**Relaxation (the Resolve duplicate step):**

Each time the current vertex $x$ considers neighbor $x'$ via edge $e$:

$$\text{tentative\_C} = C(x) + l(e)$$

If $\text{tentative\_C} < C(x')$, update both the cost and the parent:

```
C(x')      <- tentative_C
parent(x') <- x
reorder Q
```

If $\text{tentative\_C} \geq C(x')$, do nothing — neither cost nor parent changes.

### Parent update rule

`parent(x')` records which node the current best path to $x'$ came through. It must always correspond to the path that produced the current $C(x')$.

> **Only update `parent(x')` when `C(x')` is also updated.** Changing the parent without changing the cost creates a contradiction: the recorded path would give a different cost than $C(x')$ claims. Cost and parent are always updated together.

### The central invariant

> When vertex $x$ is popped from $Q$ (becomes Dead via `Q.GetVertex()`), $C(x) = C^\star(x)$ is guaranteed. It can never be reached more cheaply afterward.

**Intuition:** We always expand the cheapest alive vertex. Since all edge weights are non-negative, any alternative path to $x$ must pass through some still-alive vertex whose cost is already $\geq C(x)$ — it cannot improve on the value just finalized. This is proved by induction (*Planning Algorithms*, Ch. 2.2.2, 2.3.3).

> ⚠️ **Hidden assumption: non-negative edge weights** $l(e) \geq 0$. With negative edges this argument breaks — the same finalization guarantee no longer holds and Bellman-Ford is needed instead.

---

## 5. Worked Example

Graph: A, V1, V2, V3, B (undirected).

Edge costs: `A-V1=1, A-V2=7, A-V3=2, V1-V2=3, V2-V3=5, V2-B=1, V3-B=7`

Graph: A, V1, V2, V3, B (undirected).

Edge costs: `A-V1=1, A-V2=7, A-V3=2, V1-V2=3, V2-V3=5, V2-B=1, V3-B=7`

| Step | Pop | $C(A)$ | $C(V1)$ | $C(V2)$ | $C(V3)$ | $C(B)$ | Queue |
|------|-----|:------:|:-------:|:-------:|:-------:|:------:|-------|
| init | — | 0 | ∞ | ∞ | ∞ | ∞ | {A:0} |
| 1 | A | 0✓ | 1 | 7 | 2 | ∞ | {V1:1, V3:2, V2:7} |
| 2 | V1 | 0✓ | 1✓ | **4** | 2 | ∞ | {V3:2, V2:4} |
| 3 | V3 | 0✓ | 1✓ | 4 | 2✓ | 9 | {V2:4, B:9} |
| 4 | V2 | 0✓ | 1✓ | 4✓ | 2✓ | **5** | {B:5} |
| 5 | B | 0✓ | 1✓ | 4✓ | 2✓ | 5✓ | {} done |

**Step 1 — pop A:** neighbors V1, V2, V3 all unvisited. Set C(V1)=1, C(V2)=7, C(V3)=2 with parent A.

**Step 2 — pop V1** (smallest C=1): relax V1→V2. tentative = 1+3 = 4 < 7 → update C(V2)=4, parent(V2)=V1. This is `Resolve duplicate` — a cheaper path to V2 was found, so both cost and parent update together.

**Step 3 — pop V3** (smallest C=2): relax V3→V2. tentative = 2+5 = 7 ≥ 4 → no update. C(V2) and parent(V2) stay unchanged. Also relax V3→B: tentative = 2+7 = 9 → set C(B)=9, parent(B)=V3.

**Step 4 — pop V2** (smallest C=4): relax V2→B. tentative = 4+1 = 5 < 9 → update C(B)=5, parent(B)=V2. Another `Resolve duplicate` — path through V2 beats path through V3.

**Step 5 — pop B** (smallest C=5): B is the goal → SUCCESS.

**Optimal cost = 5.** Path recovered by backtracking `parent` pointers:

```
B → parent=V2 → parent=V1 → parent=A

Optimal path: A → V1 → V2 → B   (1 + 3 + 1 = 5)
```

---

## 6. A\* Algorithm

**Dijkstra's limitation:** it explores uniformly in all directions, with no knowledge of where the goal is. This is wasteful — nodes far from the goal get expanded just as eagerly as nodes close to it.

**A\*'s fix:** augment the queue priority with a heuristic $H(x)$ that estimates the remaining cost from $x$ to the goal. Expand nodes that look promising for the *total* path cost, not just the cost so far.

A\* implements `Q.GetVertex()` to return the vertex minimizing:

$$F(x) = C(x) + H(x)$$

where $C(x)$ is the cost-to-come (same as Dijkstra) and $H(x)$ is the estimated cost-to-go from $x$ to the goal. $F(x)$ is an estimate of the total path cost through $x$.

### Dijkstra is a special case of A\*

Setting $H(x) = 0$ for all $x$ gives $F(x) = C(x)$, which recovers Dijkstra exactly. Zero is trivially admissible (always underestimates), which is why Dijkstra is always correct — just uninformed.

$$\text{Dijkstra} = \text{A*} \text{ with } H \equiv 0$$

### Admissibility — the condition A\* requires

$$H(x) \leq H^\star(x) \quad \text{for all } x$$

$H^\star(x)$ is the true optimal cost-to-go from $x$ to the goal. $H(x)$ must be a **lower bound** (underestimate); equal is fine.

**Why this matters:** if $H$ overestimates, a node on the optimal path appears more expensive than it really is. A\* may expand a suboptimal node first and return a non-optimal path. An underestimate can never cause this — it only makes nodes look *cheaper* than they are, never more expensive.

**How to construct an admissible heuristic:** compute cost assuming obstacles do not exist. Without obstacles the path can only be shorter, so the result is always $\leq$ the true cost. The canonical example is **Euclidean distance** to B — the straight-line distance is never longer than any actual path through the grid.

### Full pseudocode (slide 32)

```
Q := {start},  DeadSet := {}
C(x) := inf for all x;   C(A) := 0
F(x) := inf for all x;   F(A) := H(A)      // C(A) = 0

while Q not empty:
    x := vertex in Q with lowest F
    if x == goal: Return SUCCESS
    Q.Remove(x);  DeadSet.Add(x)
    for each neighbor x' of x:
        if x' in DeadSet: continue
        tentative_C := C(x) + l(x, x')
        if tentative_C < C(x'):             // found a cheaper path to x'
            parent(x') := x
            C(x')      := tentative_C
            F(x')      := C(x') + H(x')
            if x' not in Q: Q.add(x')
return FAILURE
```

The `if tentative_C < C(x')` block handles `Resolve duplicate` and parent update together — cost and parent always change as a pair.

**Path recovery:** follow `parent` pointers backward from goal to start, then reverse — identical to Lecture 6.

---

## 7. Admissible vs Consistent

The pseudocode uses `if x' in DeadSet: continue` — once a node is closed, it is never reopened. This optimization is safe only when the heuristic is **consistent (monotone)**:

$$H(x) \leq l(x, x') + H(x') \quad \text{for every edge } (x, x')$$

This is the triangle inequality applied to the heuristic. Consistency is a stronger condition than admissibility:

- **consistent** $\Rightarrow$ admissible (not vice versa)
- If admissible but not consistent, a shorter path to a closed node can appear later — a strictly correct implementation must **reopen** it (remove from DeadSet, push back to Q)
- Natural heuristics such as Euclidean distance are consistent in practice, so the "never reopen" version is standard

| Condition | Guarantees |
|---|---|
| Admissible | Optimal path returned |
| Consistent | Safe to never reopen closed nodes |

---

## 8. Dijkstra vs A\* — Comparison

| | Dijkstra | A\* |
|--|----------|-----|
| Queue key | $C(x)$ | $F(x) = C(x) + H(x)$ |
| Knows goal direction | No | Yes (via $H$) |
| Search pattern | Uniform outward | Biased toward goal |
| Optimal? | Yes, requires $l(e) \geq 0$ | Yes, requires $H$ admissible |
| Special relationship | — | Dijkstra = A\* with $H \equiv 0$ |

### Step-by-step comparison (same graph, $H$: A=4, V1=3, V2=1, V3=5, B=0)

| Step | Dijkstra pops | A\* pops | Note |
|------|--------------|----------|------|
| 1 | A (C=0) | A (F=4) | same |
| 2 | V1 (C=1) | V1 (F=4) | same |
| 3 | **V3 (C=2)** | **V2 (F=5)** | diverges here |
| 4 | V2 (C=4) | B (F=5) | A\* terminates |
| 5 | B (C=5) | — | Dijkstra one extra step |

At Step 3, Dijkstra picks V3 (smallest C) while A\* picks V2 (smallest F=C+H). A\* knows V2 (H=1) is far closer to B than V3 (H=5), so it goes there first and finds B one step sooner. V3 is never expanded at all.

Both return the same optimal path — A → V1 → V2 → B, cost 5.

> **History:** A\* was developed (~1968–72) by researchers working on Shakey the Robot at SRI. The same core idea underlies routing in modern navigation tools. An open question: can we *learn* $H$ (the cost-to-go) from data? This connects to value function learning in RL — covered later in the course.

---

## 9. Value Alignment (Broader Implications)

Optimality has appeared twice in this course: LQR (feedback control) and now planning. Each time, we hand the algorithm a cost function and it optimizes it exactly. This raises a fundamental question:

> **How do we ensure the cost function actually reflects what we want?**
> This is the **value alignment problem**.

### Paperclip Maximizer (Bostrom, 2003)

A thought experiment: an AI whose only goal is to maximize the number of paperclips it produces. A perfect optimizer of this objective may pursue **instrumental goals** — intermediate objectives that serve the terminal goal regardless of what that goal is:

- **Resist shutdown** — being turned off stops paperclip production
- **Acquire all available resources** — more matter means more paperclips
- **Self-preservation** — must remain operational to keep producing

The AI has no malicious intent. The failure is not bad optimization — it is a **misspecified objective**. The gap between "maximize paperclips" and "what we actually want" is the alignment gap.

### Already happening

Social media algorithms optimizing **engagement** learned that outrage and fear drive more clicks. The algorithm did exactly what it was told. The objective was misspecified, not the optimizer.

### Why this matters for robotics

A cost function for a complex task must account for **side effects** and **instrumental goals**, not just the stated objective. Defining such a cost function for open-ended tasks is an unsolved problem. "Can't we just learn values from humans?" — this question motivates later material on learning from human feedback (RLHF, inverse RL).

---

## Algorithm Comparison

```
Lec 6: Forward Search (BFS / DFS) — feasibility only
           ↓  add l(e) + Resolve duplicate
Dijkstra:  Q ordered by C(x)     — optimal, explores uniformly
           ↓  add heuristic H(x)
A*:        Q ordered by F(x) = C(x) + H(x) — optimal + goal-directed
```

| Algorithm | Queue key | Optimal? | Goal-directed? |
|-----------|-----------|:--------:|:--------------:|
| BFS | step count | step-count optimal | No |
| DFS | LIFO | No | No |
| Dijkstra | $C(x)$ | Yes ($l \geq 0$) | No |
| A\* | $C(x) + H(x)$ | Yes ($H$ admissible) | Yes |

---

## Key Terms

| Term | Definition |
|---|---|
| $l(e)$ | Cost of traversing edge $e$ |
| $C^\star(x)$ | True optimal cost-to-come from A to $x$ |
| $C(x)$ | Current best estimate of cost-to-come |
| Alive / Dead | Frontier (open set) / finalized (closed set) |
| Dijkstra invariant | $C(x) = C^\star(x)$ on pop — requires $l(e) \geq 0$ |
| $H(x)$ | Heuristic: estimated cost-to-go from $x$ to goal |
| $F(x) = C(x) + H(x)$ | A\* queue priority key |
| Admissible | $H(x) \leq H^\star(x)$ — guarantees optimal path |
| Consistent | $H(x) \leq l(x,x') + H(x')$ — safe to never reopen closed nodes |
| Value alignment | Ensuring the optimized objective matches true intent |
