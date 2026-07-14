# Week 2 — Motion Planning (Lectures 6–10)

**Course:** Introduction to Robotics (Princeton MAE 345/549), Prof. Anirudha Majumdar
**Scope:** Discrete search → randomized planning → dynamics-aware planning, plus Assignments 3 & 4.

---

## One-line story

> **Lec 6–7 chop the world into a grid and search the graph; Lec 8 throws away the grid and grows a random tree; Lec 9 makes those random edges physically executable for a special class of systems; Lec 10 handles the general case and closes the loop back to control.**

```
Lec 6-7: grid search (BFS/DFS → Dijkstra/A*)   — point robot, feasibility → optimality
      ↓ "the robot has a shape" + curse of dimensionality
Lec 8: RRT — continuous space, Configuration Space; but straight-line Extend ignores dynamics
      ↓ "a car can't move sideways"
Lec 9: Differential Flatness — for flat systems, smooth z-space curves are always executable
      ↓ what about systems that aren't flat?
Lec 10: RRT + dynamics (3 ideas) + TVLQR — plan for general systems, then correct online
```

---

## Contents

### Lecture notes

| Lecture | Topic | Note |
|---------|-------|------|
| Lec 6 | Discrete planning: BFS / DFS | [`lecture6_motion_planning.md`](./lecture6_motion_planning.md) |
| Lec 7 | Optimal discrete planning: Dijkstra / A\* | [`lecture7_optimal_discrete_planning.md`](./lecture7_optimal_discrete_planning.md) |
| Lec 8 | Randomized planning: Configuration Space & RRT | [`lecture8_rapidly-exploring_randomized_tree.md`](./lecture8_rapidly-exploring_randomized_tree.md) |
| Lec 9 | Differential Flatness | [`lecture9_differential_flatness.md`](./lecture9_differential_flatness.md) |
| Lec 10 | Planning with dynamics constraints + TVLQR | [`lecture10_planning_with_dynamics_constraints.md`](./lecture10_planning_with_dynamics_constraints.md) |

### Assignments

**Assignment 3** — [`assignment/assignment3/`](./assignment/assignment3/)

| Item | File |
|------|------|
| Problem 1 — BFS/DFS hand-trace (4- vs 8-connectivity) | [`problem1.md`](./assignment/assignment3/problem1.md) |
| Lab 3 — A\* implementation | [`Lab3.ipynb`](./assignment/assignment3/Lab3.ipynb) |
| Lab 3 — A\* implementation (Korean comments) | [`Lab4_kor.ipynb`](./assignment/assignment3/Lab3_kor.ipynb) |

**Assignment 4** — [`assignment/assignment4/`](./assignment/assignment4/)

| Item | File |
|------|------|
| Problem 1 — RRT probabilistic completeness (bowtie) | [`problem1.md`](./assignment/assignment4/problem1.md) |
| Problem 2(a) — Car differential flatness | [`problem2(a).md`](./assignment/assignment4/problem2(a).md) |
| Problem 2(b) — Fully actuated mechanical system | [`problem2(b)_fully_actuated.md`](./assignment/assignment4/problem2(b)_fully_actuated.md) |
| Problem 2(b) — Hopping robot (alternative) | [`problem2(b)_hopping_robot.md`](./assignment/assignment4/problem2(b)_hopping_robot.md) |
| Lab 4 — RRT implementation | [`Lab4.ipynb`](./assignment/assignment4/Lab4.ipynb) |
| Lab 4 — RRT implementation (Korean comments) | [`Lab4_kor.ipynb`](./assignment/assignment4/Lab4_kor.ipynb) |

---

## Lecture 6 — Discrete Planning (BFS / DFS)

Full note: [`lecture6_motion_planning.md`](./lecture6_motion_planning.md)

- **Grid → Graph (3 steps):** one vertex per free cell, connect with 4-/8-connectivity, delete obstacle cells and their edges.
- **Forward Search:** BFS and DFS differ by a *single line* — how `Q.GetVertex()` pops.
  - **BFS = FIFO (queue)** → explores layer by layer → **optimal in number of steps.**
  - **DFS = LIFO (stack)** → dives deep along one branch → **no optimality guarantee.**
- **Completeness (first pass):** BFS/DFS are complete *on the graph*, but *not necessarily on the continuous problem* — a too-coarse grid can swallow a narrow gap into obstacle cells, so the graph reports "no path" even when one exists. **The failure is in discretization, not in the search.** (This idea returns in Lec 8.)

**Why BFS is optimal but DFS isn't:** when BFS first reaches the goal, every shorter path has already been fully explored, so the path found must be shortest. DFS has no such layer ordering — "found first" does not mean "shortest."

---

## Lecture 7 — Optimal Discrete Planning (Dijkstra / A\*)

Full note: [`lecture7_optimal_discrete_planning.md`](./lecture7_optimal_discrete_planning.md)

- **New ingredient — cost + Resolve Duplicate:** when a cheaper path to an already-seen vertex is found, update its cost **and** parent together (they must always move as a pair).
- **Dijkstra:** `Q.GetVertex()` returns the vertex with minimum **C(x)** (cost-to-come). Each pop finalizes `C(x)=C*(x)` (assuming non-negative edge weights).
- **A\*:** priority is **F(x) = C(x) + H(x)**, where `H(x)` is a heuristic estimate of cost-to-go.
  - **Dijkstra = A\* with H ≡ 0.**
  - **Admissibility:** `H(x) ≤ H*(x)` (never overestimates). Computing `H` while *ignoring obstacles* — e.g. Euclidean distance — is always admissible.
  - A\* biases the search toward the goal, so it typically expands far fewer vertices than Dijkstra.

**Implemented in Lab 3** (`assignment/assignment3/Lab3.ipynb`): `GetBestVertex` (min-F pop), `computeH` (Euclidean), and the Resolve-Duplicate loop. Result: 10×10 maze, A(0,0)→B(7,6), 26 iterations, path length 18.

---

## Lecture 8 — Configuration Space & RRT

Full note: [`lecture8_rapidly-exploring_randomized_tree.md`](./lecture8_rapidly-exploring_randomized_tree.md)

- **Two walls that kill grid search:** (1) curse of dimensionality (k^d vertices), (2) the robot has a *shape* → work in **Configuration Space C** where a whole pose is one point. Note **C ≠ ℝ³** (the θ-axis wraps: C = ℝ² × S¹), and **C_obs** can be very complex.
- **RRT** dodges both walls: no grid, no explicit C_obs — just sample, find nearest, `Extend` by step-size *d*, collision-check.
- **Why it works:** only needs **single-pose collision checks** (very fast, reuses game-engine software) — the primary reason RRTs scale.
- **Completeness revisited:** RRT is **not** complete (can loop forever), only **probabilistically complete** (P→1 as iterations→∞, under a sufficient-separation condition). This is where the failure point *migrates* from discretization (Lec 6) to randomness.

---

## Lecture 9 — Differential Flatness

Full note: [`lecture9_differential_flatness.md`](./lecture9_differential_flatness.md)

- **The broken assumption:** RRT's straight-line `Extend` assumes any trajectory is executable — false ("a car can't move sideways").
- **Differentially flat systems:** there exist flat outputs z̄ such that *all* states and inputs are recoverable from z̄ and finitely many derivatives (no integration). Pick a smooth z-trajectory → everything else follows.
- **Consequence:** any sufficiently smooth z-trajectory is dynamically feasible → run RRT in **flat-output space**, repairing the broken `Extend`.
- **Theorem:** # flat outputs = # control inputs.
- **Worked example (planar quadrotor):** divide (1)/(2) to eliminate thrust → θ = arctan(−ẍ/(ÿ+g)); inputs need z's 4th derivative → "minimum snap." This is the template reused in Assignment 4 P2.

---

## Lecture 10 — Planning with Dynamics Constraints + TVLQR

Full note: [`lecture10_planning_with_dynamics_constraints.md`](./lecture10_planning_with_dynamics_constraints.md)

- **Part 1 — RRT for non-flat systems, three ideas:**
  1. **Random-Propagate** — shoot a random input; no exploration bias → fails in practice.
  2. **Discretize** — keep Lec-8 sampling, approximate `Extend` by trying several candidate inputs.
  3. **Steer** — solve the boundary-value problem exactly (trajectory optimization); heavy but high quality. *(Flatness is a free, closed-form special case of this.)*
- **Part 2 — TVLQR (executing under uncertainty):** feedback ū(t) = ū₀(t) + K(t)[x̄(t) − x̄₀(t)]. Split state into reference + deviation to linearize; solve the (time-varying) Riccati equation for K(t). "Trajectory optimization once, offline + TVLQR for real-time correction" is the standard combo — closing the Planning ↔ Control loop.

---

## Assignments at a glance

- **A3 P1** — Hand-trace BFS/DFS on a maze. Under 4-connectivity, A is trapped in a 4-cell pocket → **FAILURE**; adding diagonals (8-connectivity) makes A→B a single diagonal step → **SUCCESS**. Takeaway: *connectivity choice decides feasibility, not just efficiency.*
- **A4 P1** — Bowtie environment: the free corridor pinches to the single origin point (measure zero), so RRT's success probability is **0 even as K→∞** — a counterexample to probabilistic completeness (sufficient-separation condition violated).
- **A4 P2(a)** — Car is differentially flat with z̄=(x,y): same divide-trick as the quadrotor, terminating at 2nd derivatives (q=2); u₂ encodes path curvature.
- **A4 P2(b)** — Two versions provided:
  - *Fully actuated arm:* flat with z̄=q, proof is a one-line substitution (τ = M(z)z̈ + N(z,ż)) — flatness is nearly a tautology when fully actuated (= computed torque control).
  - *Hopping robot:* flat output = foot position, but θ needs a **quadrature** (geometric phase / holonomy) — sits at the boundary of the strict flatness definition; a deeper alternative.

---

## Recurring threads worth noticing

- **Completeness migrates:** grid search leaks completeness at *discretization* (Lec 6); RRT leaks it at *randomness* (Lec 8) → only probabilistic (A4 P1 is the sharp counterexample).
- **The divide-trick:** eliminating a tangled unknown by taking a ratio recurs in the quadrotor proof (Lec 9) and the car proof (A4 P2a); underactuated systems need it, fully actuated ones don't (A4 P2b).
- **RRT is the backbone throughout:** Lec 8 introduces it, Lec 9 fixes its `Extend` via flatness, Lec 10 generalizes that fix (steering / trajectory optimization) and adds feedback tracking.
