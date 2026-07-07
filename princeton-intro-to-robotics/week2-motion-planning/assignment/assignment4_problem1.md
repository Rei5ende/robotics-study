# Assignment 4 — Problem 1: RRT Probabilistic Completeness (Bowtie Environment)

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Related lecture:** Lecture 8 (Randomized Motion Planning / RRT)

---

## Problem

A motion-planning problem in ℝ² with two obstacles:

- **Obstacle 1:** $\{(x,y) \mid (x,y)\neq(0,0),\; x \le -y^2\}$
- **Obstacle 2:** $\{(x,y) \mid (x,y)\neq(0,0),\; x \ge y^2\}$

The robot is a point. START sits in the upper region, GOAL in the lower region (see figure in the assignment). The problem is **feasible** — a collision-free path from START to GOAL exists. Using vanilla RRT:

1. What is the probability the algorithm finds a feasible path within **K** iterations?
2. As **K → ∞**, what does this probability approach (1, 0, or something else)?

---

## Solution Outline

### Step 1 — Characterize the free space

The free space is everything that is *neither* obstacle:

$$\text{not obstacle 1: } x > -y^2 \qquad \text{and} \qquad \text{not obstacle 2: } x < y^2$$

$$\boxed{-y^2 < x < y^2}$$

This interval is non-empty only when $-y^2 < y^2$, i.e. when $y^2 > 0$, i.e. **$y \neq 0$**.

At **$y = 0$** the condition collapses to $0 < x < 0$, which no $x$ satisfies. The only free point on the line $y=0$ is the **origin itself** (kept free by the explicit "$(x,y)\neq(0,0)$" exclusion in each obstacle).

**Geometry (bowtie):**
- Upper region ($y>0$): free space opens up between the two parabolas — contains START.
- Lower region ($y<0$): opens up symmetrically — contains GOAL.
- Near $y=0$: the corridor pinches to width 0. **Upper and lower regions connect only through the single point $(0,0)$.**

Every path from START to GOAL must pass through the origin.

### Step 2 — What must RRT do to cross?

The tree starts in the upper region (START). To reach GOAL it must eventually cross $y=0$ into the lower region. The **only** free gateway from upper to lower is the origin. So a new tree node must extend across $y=0$ *through the origin* — any other crossing of $y=0$ lands in an obstacle.

### Step 3 — The gateway has measure zero

The gateway is a single point $(0,0)$. In ℝ², a single point has **area (Lebesgue measure) = 0**.

### Step 4 — Probability of hitting a measure-zero set

RRT samples $\bar q_{\text{rand}}$ from a continuous (uniform) distribution. For an Extend step to carry the tree from the upper to the lower region, the extended segment from an upper node $\bar q_{\text{near}}$ must pass exactly through the origin.

Two equivalent ways to see this is a probability-0 event:

- **By direction:** from $\bar q_{\text{near}}$, exactly **one** direction (angle $\theta^\*$) aims at the origin. A continuously-random angle equals a specific value $\theta^\*$ with probability 0 (a single point on $[0,2\pi)$ has length 0).
- **By sample location:** the set of $\bar q_{\text{rand}}$ that produce an origin-crossing segment is a single **line** through $\bar q_{\text{near}}$ and the origin. A line in ℝ² has area 0, so $P(\bar q_{\text{rand}} \text{ lands on it}) = 0$.

Either way, the probability of crossing in one iteration is:

$$p = 0$$

### Step 5 — Probability within K iterations

With per-iteration success probability $p=0$, the probability of at least one success in K iterations is:

$$1 - (1-p)^K = 1 - (1-0)^K = 1 - 1 = 0$$

### Step 6 — Limit as K → ∞

$$\lim_{K\to\infty}\big[\,1-(1-0)^K\,\big] = 0$$

The probability stays at **0** — it does **not** approach 1.

---

## Answers

1. **Probability within K iterations = 0** (for every finite K).
2. **As K → ∞, the probability remains 0** — it does not approach 1.

---

## Why This Matters — Connection to Probabilistic Completeness

Recall from Lecture 8: RRT is **probabilistically complete** — *if a path exists, the probability of finding it → 1 as iterations → ∞* — **but only under mild technical conditions, notably sufficient separation between obstacles.**

> **This bowtie is a counterexample where that condition fails.** A feasible path exists (through the origin), yet RRT finds it with probability 0 even as K → ∞. The failure is geometric: the connecting corridor is not a region of positive area but a single measure-zero point, so continuous random sampling can never land on the gateway. The obstacles touch (zero separation) at the origin, breaking the "sufficient separation" precondition — and with it, probabilistic completeness.

**One-line takeaway:** *feasible ≠ found.* Probabilistic completeness guarantees convergence to a solution **only when the free corridor has positive measure**; pinch it to a point and the guarantee evaporates.

---

## Summary

1. Free space $-y^2 < x < y^2$; upper/lower connect only at the origin (include a sketch).
2. To descend, the tree's extend segment must pass through the origin.
3. That is a measure-zero event → per-iteration probability $p = 0$.
4. K iterations: $1-(1-0)^K = 0$; limit as $K\to\infty$ is 0.
5. Conclusion: feasible yet probability 0 → counterexample to probabilistic completeness (sufficient-separation condition violated).
