# Lecture 8: Randomized Algorithms for Motion Planning (RRTs)

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lectures 6–7 discretized the world into a grid and ran **graph search** (BFS/DFS for feasibility, Dijkstra/A\* for optimality). Powerful, but discretization has two problems that get worse in high dimensions. This lecture abandons the grid and works **directly in the continuous space** using random sampling — the **Rapidly-Exploring Randomized Tree (RRT)**.

> **One-line summary:** Lec 6–7 chop space into a grid and search the graph; Lec 8 throws away the grid and grows a random tree in continuous space — *Configuration Space* and *RRT*.

---

## 1. Why Abandon the Grid? — Two Walls

Grid + graph search assumed **discretization**. Two walls appear:

**Wall 1 — Curse of dimensionality.** Discretize each of 3 dimensions into *k* partitions → *k³* vertices. Grows exponentially with dimension. The notes warn this is "even worse than it first appears" — and Wall 2 is why.

**Wall 2 — The robot has a shape.** So far we pretended the robot is a point. Real robots are rectangles, arms, etc. Handling shape properly raises the dimension *and* makes obstacles complicated.

---

## 2. Configuration Space — The Core Mental Shift

### Naive idea: wrap the robot in a circle

Approximate a rectangular robot with a radius-*R* circle → treat the robot as a **point** again, but **inflate every obstacle by *R***. Reduces to "point robot + fattened obstacles."

> **⚠️ Confusion #1 — circle approximation can destroy a valid solution.**
> The naive method works *sometimes*, but **fails on the notes' example**. The original rectangular robot *could* get from A→B (feasible), but wrapping it in a circle makes the circle too fat to fit through a narrow gap → the problem becomes **infeasible**. The circle throws away the rectangle's ability to *rotate* and squeeze through. **Inflation can delete a path that actually exists.**

### Real idea: Configuration Space *C*

Treat the robot's entire pose as **a single point** in a new space. A rectangular robot has position (2) + angle (1) = **3 DoF**, fully described by *(x, y, θ)*. The space of all *(x, y, θ)* is the **configuration space** *C*, and:

> **robot pose = one point in *C***

In the real world (**workspace**) the robot is a bulky shape; in *C* it's a single point. The headache of "robot shape" is absorbed by *changing the space* — at the cost of complicated obstacles.

> **⚠️ Confusion #2 — *C* ≠ ℝ³.**
> θ = 0 and θ = 2π are the *same* pose, so the θ-axis is not a straight line — it **wraps into a circle**. The true shape of *C* is **ℝ² × S¹** (plane × circle), not 3D Euclidean space. This matters for sampling and distance (see §3).

### Configuration-space obstacles *C*ₒᵦₛ

$$C_{\text{obs}} = \{(x, y, \theta) \mid \text{robot at pose } (x, y, \theta) \text{ collides with some obstacle}\}$$

For each pose, ask "does this pose collide?"; collect all colliding poses. *C*ₒᵦₛ ⊆ *C*.

> **⚠️ Confusion #3 — simple workspace obstacles become twisted *C*ₒᵦₛ.**
> A plain rectangular wall in the workspace can map to a **very complicated shape** in *C*ₒᵦₛ. The slide's rotation animation shows why: rotating the robot (changing θ) lets it clear a gap at some angles and blocks it at others, so the obstacle's cross-section keeps morphing along the θ-axis.

### The two walls survive

We reduced planning to "find a path for a point" — but:

1. **C can be very high-dimensional.** A robot arm's dimension = number of joints = DoF. 6 joints → 6D → *k⁶* vertices. Discretization is hopeless.
2. **Constructing *C*ₒᵦₛ explicitly is computationally brutal** — even for a rectangle.

---

## 3. RRT — Dodging Both Walls at Once

**Rapidly-Exploring Randomized Tree** (LaValle, ~1999–2000). On arrival it solved problems far beyond previous methods and became the state of the art.

Two key moves:
- No grid → **works directly in continuous space** (dodges Wall 1).
- No *C*ₒᵦₛ → **only checks whether one specific pose collides** (dodges Wall 2).

### Algorithm (vanilla RRT)

Start with only the start pose q̄_A in the tree. Repeat until the goal q̄_B is reached:

```
V = { q_A }                              // tree with just the start
while goal not reached do
    q_rand  ← sample a random configuration     // any point in C
    q_near  ← nearest node in V to q_rand
    q_s     ← Extend(q_near, q_rand)            // step from near toward rand by d
    if q_s in collision then
        continue                                 // discard, resample
    else
        add q_s to V
        add edge q_near — q_s
        if ‖q_s − q_B‖ < ε then                 // close enough to goal
            return SUCCESS
```

### One iteration, in 4 steps

1. **Sample** — drop a random point q̄_rand somewhere in *C*.
2. **Nearest** — find the tree node q̄_near closest to q̄_rand.
3. **Extend** — step from q̄_near toward q̄_rand by **step-size *d***, producing q̄_s. (Not all the way to rand — just one step!)
4. **Collision-check** — if q̄_s is inside an obstacle, discard; else attach it to the tree.

> **⚠️ Confusion #4 — q̄_rand is a *direction signal*, not a destination.**
> The tree never travels *to* q̄_rand. It takes one step of size *d* in that direction, then q̄_rand is **thrown away**. The next iteration draws a brand-new q̄_rand. rand just says "want to grow *this way* this time?"

**Why "Rapidly-Exploring":** uniform random samples land everywhere, so large unexplored regions are more likely to pull samples toward them. The tree naturally develops a bias toward growing into unexplored space.

### Sampling q̄_rand — the details (from discussion)

q̄_rand is a random point in **configuration space**, not the workspace. Sample *every* coordinate that defines the pose:

- Rectangle robot (3 DoF): q̄_rand = (x_rand, y_rand, **θ_rand**) — angle is sampled too.
- Robot arm (6 DoF): q̄_rand = (θ₁, …, θ₆) — all six joint angles.

> **⚠️ Coordinates are sampled over different ranges.**
> - *x, y*: uniform over the environment bounds.
> - *θ*: uniform over **[0, 2π)**, and because the θ-axis wraps (Confusion #2), it's really sampled on a *circle* (wrap-around at 2π→0). Distance computations for q̄_near must also respect this: 359° and 1° are 2° apart, not 358°.

> **Collision of q̄_rand doesn't matter.** We sample q̄_rand without checking whether it's in *C*ₒᵦₛ. Only q̄_s (the new step point) is collision-checked. Since rand is just a direction signal and gets discarded, it's fine if it lands inside an obstacle.

> **💡 Aside — goal biasing (variant, not vanilla):** occasionally (e.g. 5% of the time) force q̄_rand = q̄_B to pull the tree toward the goal. Handy in practice; outside the vanilla algorithm.

### Extend and step-size *d* (Slide 29)

Slide 29 makes two points about `Extend()`:

**(1) *d* is *the* tuning parameter.** It sets how far the tree reaches in one step. *d* too large → skips over obstacles / can't thread fine gaps; *d* too small → tree crawls, needing far more iterations. A genuine trade-off.

**(2) We skip full-segment collision checking.**

> **⚠️ Confusion #5 — only the endpoint is checked, not the whole segment.**
> Points *along* the segment q̄_near → q̄_s can be in collision even if q̄_s is not. Strictly we should check the entire segment. But if *d* is small, checking just q̄_s is a reasonable approximation.

**The two points are one idea:** small *d* → short segment → endpoint-only check is safe. So *d* is not just a stride length — it's also the **safety margin of the collision-check shortcut**.

> **What if ‖q̄_rand − q̄_near‖ < d?** (from discussion) Extend moves by `min(d, ‖q_rand − q_near‖)`. If rand is *closer* than *d*, we stop **exactly at q̄_rand** (q̄_s = q̄_rand) rather than overshooting past it. The `min` guarantees the tree never overshoots q̄_rand — it grows *toward or up to* rand, never beyond.
> ```
> dir  = (q_rand − q_near) / ‖q_rand − q_near‖
> step = min(d, ‖q_rand − q_near‖)
> q_s  = q_near + step · dir
> ```

### Why RRT works so well (Slide 30)

> **💡 The primary reason RRTs work.**
> RRT needs **no explicit construction of *C*ₒᵦₛ**. It only asks: *"is this one specific pose in collision?"* — a yes/no question about a single point, not a map of the whole obstacle set.
>
> Analogy: building *C*ₒᵦₛ is like **mapping an entire minefield**; RRT just asks *"is there a mine under the single step I'm about to take?"* The full map is near-impossible; the single-point check is easy.
>
> Single-pose collision checking is **extremely fast**, and lots of highly optimized software already exists — much of it built for **video-game engines** (games judge collisions thousands of times per second in real time). RRT borrows that machinery directly. Slide 30's exact words: *"This is the primary reason RRTs work so well."*

This single-pose-check property is also **why RRT scales to high-dimensional *C*** (6–7 DoF arms): raising the dimension doesn't blow up the difficulty of "is this pose in collision?" the way a grid explodes.

### Path recovery & quality

- **Recover the path** by **parent backtracking** from goal to start — same idea as the discrete search in Lec 6.
- **Path quality:** RRT paths are **jagged** (random, so not shortest).

---

## 4. RRT Variants

- **RRT\*** (RRT-star): standard RRT finds *any* feasible path; RRT\* returns an **(approximately) optimal** path. In the slide's RRT vs RRT\* comparison, the RRT\* path is visibly cleaner.
- **Bidirectional RRT:** grow two trees — one from the start, one from the goal — and connect them in the middle.

---

## 5. Completeness — RRT's Honest Weakness

**Completeness (shared definition):** an algorithm is *complete* if it finds a path whenever one exists, and otherwise terminates in finite time reporting "no path."

The definition is identical to Lec 6 — what changes across algorithms is **where completeness leaks**. Two layers must be kept apart: completeness on the **graph (discrete)** problem vs on the **continuous (original)** problem.

### Connecting Lec 6 → Lec 8: three layers of failure

**Layer 1 — BFS/DFS are complete on the *graph* (Lec 6).** Given a graph, if an A→B path exists in it, BFS/DFS will find it; otherwise the queue empties and they report "no path." Perfectly complete *at this layer*.

**Layer 2 — but not on the *continuous* problem (Lec 6's subtlety).** If the grid is too coarse, a narrow gap gets swallowed into obstacle cells. The graph then has no path even though the continuous world does.

> **⚠️ The failure is in *discretization*, not in the search.** BFS honestly reports "no path" — the graph really has none. The lie was told by the **discretization step**, not the search algorithm. (Weakly rescued by *resolution completeness*: keep refining the grid and eventually the path is found.)

**Layer 3 — RRT skips the graph entirely, yet fails for a *different* reason (Lec 8).** RRT never discretizes, so it dodges Layer 2's narrow-gap problem. But it's still not complete — this time because of **randomness**:

> **⚠️ Confusion #6 — RRT is not complete.**
> - If **no path exists:** the `while` loop can run **forever** (never reaches the goal, never triggers the termination condition). It can't cleanly declare "failure" and stop.
> - If a path **does** exist: no finite-time guarantee — bad luck can waste samples indefinitely.

### Probabilistic Completeness

RRT is instead **probabilistically complete**:

$$\text{if a path exists, } P(\text{RRT finds a solution}) \to 1 \text{ as iterations} \to \infty$$

i.e. "run it long enough and it almost surely finds a path" — under mild technical conditions (e.g. enough separation between obstacles).

> **💡 Exam/presentation point:** distinguish **strong completeness** (finite-time guarantee) from **probabilistic completeness** (probability → 1 in the limit). The "mild technical condition" breaking down is exactly **Assignment 4 Problem 1** (the bowtie environment): a path exists — through the single origin point — but the separation condition fails, so the probability does *not* go to 1.

### Where completeness leaks — the migration

```
BFS/DFS: search is perfect  → leaks at DISCRETIZATION (narrow gap)
              ↓ drop discretization
RRT:     narrow-gap problem gone → leaks at RANDOMNESS (infinite loop)
              ↓ compromise
         probabilistic completeness (P → 1 in the limit)
```

This *migration of the failure point* is the real storyline linking Lec 6 → Lec 8.

---

## Concept Flow & What's Next

```
Lec 6: Forward Search (BFS/DFS) — point robot, feasibility
Lec 7: Dijkstra / A*            — point robot, optimality
        ↓ "the robot has a shape" + curse of dimensionality
Configuration Space C: robot pose = one point in C; obstacles = C_obs
        ↓ but C is high-dimensional + C_obs is brutal to construct
RRT: no grid, no C_obs — grow a random tree in continuous space
     └ needs only single-pose collision checks (very fast) → dominant in practice
        ↓ paths are jagged / not complete
Variants: RRT* (optimal), Bidirectional RRT
Guarantee: probabilistic completeness (P → 1 in the limit)
        ↓ Next
Lec 9: Differential flatness (trajectories that respect dynamics)
Lec 10: Planning with dynamics constraints
```

**Three approaches compared:**

| Approach | Space handling | Obstacle representation | Guarantee |
|----------|----------------|-------------------------|-----------|
| BFS/DFS | grid discretization | explicit | complete, feasible |
| Dijkstra/A\* | grid discretization | explicit | complete, optimal |
| RRT | continuous + random sampling | single-pose collision check only | probabilistically complete |

---

## Quick Reference

| Term | Meaning |
|------|---------|
| configuration space *C* | space of all robot poses; one pose = one point |
| DoF | degrees of freedom = dimension of *C* |
| *C* ≠ ℝ³ | θ-axis wraps into a circle; *C* = ℝ² × S¹ |
| *C*ₒᵦₛ | set of poses that collide; can be very complex |
| q̄_A / q̄_B | start / goal configuration |
| q̄_rand | random sample in *C* (direction signal, discarded each iteration) |
| q̄_near | tree node nearest to q̄_rand |
| q̄_s | new point, one step of size *d* from q̄_near toward q̄_rand |
| *d* | step-size; stride length **and** safety margin for endpoint-only collision check |
| Extend | step by `min(d, ‖q_rand − q_near‖)` — never overshoots rand |
| ε | goal tolerance (‖q_s − q_B‖ < ε → success) |
| Extend collision check | only q̄_s is checked, not the full segment (OK for small *d*) |
| completeness | finds a path if one exists, else terminates reporting none |
| RRT completeness | **not** complete (can loop forever if no path) |
| probabilistic completeness | P(find path) → 1 as iterations → ∞, if a path exists |
| RRT\* | variant returning approximately optimal paths |
| bidirectional RRT | grow trees from both start and goal, meet in the middle |
