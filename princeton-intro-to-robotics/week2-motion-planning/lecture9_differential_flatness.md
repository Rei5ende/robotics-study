# Lecture 9: Differential Flatness

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lecture 8's RRT worked directly in continuous space and was extremely practical — but it silently assumed the robot can **perfectly execute any trajectory** the planner produces. That is false in general: robots have **dynamics**. This lecture introduces a special class of systems — **differentially flat systems** — for which producing *dynamically feasible* trajectories becomes almost trivial: pick a smooth curve for a few "flat outputs," and the entire state and input trajectories fall out automatically by differentiation.

> **One-line summary:** RRT assumed "any trajectory is executable" — a lie (a car can't move sideways). But for differentially flat systems, choosing the flat-output trajectory alone determines everything else (states, inputs) *for free*.

---

## 1. The BIG Assumption Breaks

Lec 8 recap (with praise): RRT works directly in continuous configuration space, is extremely useful in practice, and easy to implement. But:

> **BIG assumption:** the planner's trajectory can be perfectly executed by the robot. **Not true in general.**

Two separate issues — the roadmap for the remaining lectures:

1. **The robot has dynamics (today + Lec 10).** The robot cannot follow arbitrary trajectories; a trajectory must be **dynamically feasible** — it must satisfy the constraints imposed by the robot's equations of motion (e.g., *a car cannot move sideways*).
2. **The dynamics are uncertain (Lec 10).** The model $\dot{\bar x}=f(\bar x,\bar u)$ is imperfect, and disturbances (wind gusts) push the robot off the plan → we need to correct deviations (hint: **feedback control**, from Lec 2–3).

### Where exactly RRT breaks — Extend

RRT's Extend moves from $\bar q_{\text{near}}$ **in a straight line** toward $\bar q_{\text{rand}}$ by distance *d*. Hidden assumption: two nearby configurations can be connected by a straight line *that the robot can actually follow*.

**Notes example — wheeled robot/car (top view):** two cars A and B side by side, both facing forward. In configuration space they are neighbors, but the car **cannot slide sideways**. Getting from A to B requires a potentially complicated maneuver (think parallel parking).

> **💡 "Close in configuration space ≠ close dynamically."** The straight-line edge that Extend draws may be a physical lie. This is the problem the whole lecture answers: *how do we find such maneuvers?*

---

## 2. Differentially Flat Systems — Definition

Today: a special class of systems for which this is easy. The class contains many practical robots: **car, car with trailer, planar quadrotor, full 3D quadrotor**, among others.

(Scope note from the lecture: obstacles are **ignored** today — we focus only on the "extension" operation, i.e., given A and B, find a dynamically feasible path. "It won't quite be possible to do this, but we will come close.")

### Definition

Given dynamics

$$\dot{\bar x} = f(\bar x, \bar u), \qquad \bar x \in \mathbb{R}^n,\; \bar u \in \mathbb{R}^m,$$

the system is **differentially flat** if there exist outputs $\bar z$ (**flat outputs**) such that:

**(i)** the flat outputs are a function of the state and inputs (and input derivatives):

$$\bar z = \alpha(\bar x, \bar u, \dot{\bar u}, \ddot{\bar u}, \ldots, \bar u^{(p)})$$

**(ii)** conversely, the **entire state and input can be recovered from $\bar z$ and finitely many of its time derivatives**:

$$\bar x = \beta(\bar z, \dot{\bar z}, \ddot{\bar z}, \ldots, \bar z^{(q)}), \qquad \bar u = \gamma(\bar z, \dot{\bar z}, \ddot{\bar z}, \ldots, \bar z^{(q)})$$

> **💡 Intuition — the pen tip.** Record only the trace of a pen tip ( $\bar z(t)$ ). If the system is flat, that single trace encodes *everything*: how the hand (state  $\bar x$ ) moved and what forces the muscles (inputs $\bar u$) applied — all recoverable by reading the trace's derivatives. β and γ are the **decoders**. No information was lost.

> **💡 Intuition — physics removes the choices.** Why can a quadrotor's tilt θ be recovered from position alone? Because a quadrotor has exactly **one way** to accelerate sideways: *tilting*. "Accelerating left" physically **forces** "tilted left." Once $\bar z$ is chosen, the remaining states are not free choices — the dynamics determine them, link by link, with no branching. **Flat = no hidden degrees of freedom:** fix $\bar z$, and the dynamics fill in everything else. The usual role of dynamics as a *constraint* ("you can't move sideways") flips into a *decoding rule* ("if z does this, everything else must be that").

> **⚠️ No integration anywhere.** β, γ take $\bar z$ and its **derivatives** only — no ODE solving, no optimization. Differentiation is a local, cheap operation (for polynomial z, just coefficient arithmetic). This is what makes "easily" true, and what makes real-time replanning possible.

### Why this is useful — Main Idea

> For a flat system, **plan the trajectory in flat-output space**. Specify $\bar z(t)$ → γ gives the control inputs to execute it, β gives the resulting state trajectory. Moreover, **any sufficiently smooth $\bar z(t)$ can be executed.**

Pipeline (the notes' three-panel picture):

```
specify z̄(t)          --[ Differential Flatness (β, γ) ]-->   ū(t): required control inputs
(we draw this curve)                                           x̄(t): resulting state trajectory
                                                               (both fall out automatically)
```

> **💡 The question flips.** Normally, checking "is this trajectory dynamically feasible?" means wrestling with a nonlinear ODE constraint (hard). For flat systems the question disappears: **every sufficiently smooth z-trajectory is feasible.** Why: the state/input recovered via β, γ are *constructed from the dynamics themselves*, so they satisfy the dynamics automatically. Feasibility checking is replaced by "draw a smooth curve" (e.g., polynomial fitting).

### 3D quadrotor + a counting theorem

The full 3D quadrotor (12-dimensional nonlinear state!) is differentially flat with

$$\bar z = [x,\; y,\; z,\; \psi]^T \qquad \text{(COM position + yaw)}$$

A 12-state, 4-input beast is completely steered by a trajectory of just **4 numbers**. This realization (Mellinger & Kumar, *Minimum Snap Trajectory Generation and Control for Quadrotors*) enabled real-time aggressive indoor flight — 5–10 body lengths/second through slalom courses.

**Theorem (stated without proof):**

$$\#\text{ of flat outputs} = \#\text{ of control inputs}$$

Sanity checks: quadrotor — 4 rotors = 4 flat outputs ✓. Car (Assignment 4 P2) — 2 inputs ($u_1$ speed, $u_2$ steering) = 2 flat outputs $(x,y)$ ✓. Use this to sanity-check any proposed flat output.

---

## 3. Example: Flatness of the Planar Quadrotor — *the template for Assignment 4 P2*

### System (from Lec 2)

$$\ddot x = -\frac{(F_1+F_2)}{m}\sin\theta \qquad (1)$$

$$\ddot y = \frac{(F_1+F_2)}{m}\cos\theta - g \qquad (2)$$

$$\ddot\theta = \frac{(F_2-F_1)L}{I} \qquad (3)$$

States: `[x, y, θ, ẋ, ẏ, θ̇]ᵀ` (6-dim). Inputs: $F_1, F_2$ (propeller thrusts).

**Claim:** differentially flat with $\bar z = [x, y]^T$. (Counting check: 2 inputs = 2 flat outputs ✓)

### Proof strategy — "express everything in terms of z"

Show all 6 states and 2 inputs are functions of $x, y$ and their derivatives.

**Step 1 — freebies.** $x, y$ are the flat outputs themselves; $\dot x, \dot y$ are one differentiation away. Remaining: $\theta, \dot\theta, F_1, F_2$.

**Step 2 — extract θ (the heart of the proof).** In (1)–(2), the unknown $(F_1+F_2)$ and θ are tangled together. **Divide to eliminate the input:**

$$\tan\theta = \frac{(F_1+F_2)\sin\theta}{(F_1+F_2)\cos\theta} = \frac{-m\ddot x}{m\ddot y + mg} = \frac{-\ddot x}{\ddot y + g}$$

$$\Rightarrow\; \theta = \arctan\!\left(\frac{-\ddot x}{\ddot y + g}\right) \qquad (4)$$

> **💡 Why the division works:** from (1), $(F_1+F_2)\sin\theta = -m\ddot x$; from (2), $(F_1+F_2)\cos\theta = m(\ddot y+g)$. The unknown lump $(F_1+F_2)$ is common to numerator and denominator, so **taking the ratio kills it**, leaving a relation between θ and the second derivatives of z alone. *"When two unknowns are tangled, take a ratio to eliminate one"* — the same move reappears in the car problem (Assignment 4 P2).

Physical meaning of (4): **attitude is determined by acceleration.** To accelerate left, the quadrotor *must* tilt left — that necessity *is* equation (4).

**Step 3 — the dominoes fall.**
- $\dot\theta$: differentiate (4) once → third derivatives $x^{(3)}, y^{(3)}$ appear.
- $\ddot\theta$: differentiate again → **fourth** derivatives $x^{(4)}, y^{(4)}$ appear.
- $F_1, F_2$: plug $\ddot\theta$ into (3), combine with (1) or (2) → **two linear equations** in $F_1, F_2$, easily solved.

All states (β) and inputs (γ) are now expressed via derivatives of z. **∎**

> **⚠️ The derivative order q matters — this is where "sufficiently smooth" comes from.** Recovering the inputs required z's **4th derivatives** (q = 4 for this system). So the z-trajectory must be 4-times differentiable for the inputs to be well-defined. This dictates the polynomial degree when designing trajectories — and it is exactly why quadrotor trajectory generation is called **"minimum snap"** (snap = the 4th derivative): the 4th derivative feeds directly into the inputs, so minimizing it smooths the inputs.

### ⚠️ Why z = (x, y, θ) is NOT a valid flat output

A tempting mistake: why not include θ in the flat outputs?

**Layer 1 — the count is wrong.** The theorem says #flat outputs = #inputs = 2. $(x,y,\theta)$ has 3. Violation.

**Layer 2 — θ is not independent.** Equation (4) says θ is *determined* by $\ddot x, \ddot y$. It is a **dependent** quantity, like claiming to independently choose a rectangle's width, height, *and area* — the third is fixed by the first two. Concretely, $(x,y,\theta)$ fails in one of two ways:
- **If you treat all three as free:** pick $x(t), y(t)$ and an unrelated $\theta(t)$ — physics (4) demands a *specific* θ for that $x,y$; your θ almost surely contradicts it → **physically unrealizable**. The key flatness property ("any smooth trajectory is executable") collapses.
- **If you admit θ depends on x, y:** then θ is **redundant** — it adds no information beyond $[x,y]$. Not a minimal representation.

**Requirements for a valid flat output** distilled from this example: (1) count = #inputs, (2) mutually **independent** components, (3) **complete** — everything recoverable from them. $(x,y)$ passes all three; $(x,y,\theta)$ fails (1) and (2).

### Notes (three closing remarks from the lecture)

- **You only specify z; the rest follows.** The flat outputs behave exactly as designed, but the full state trajectory can be a bit **wacky** (θ may swing wildly).
  > **⚠️ Practical consequence:** an overly aggressive z-curve can demand states/inputs beyond physical limits (thrust going negative, θ flipping past 90°). Real systems add **constraints on velocities, accelerations, and inputs** when designing z — exactly what the Mellinger–Kumar abstract lists. Flatness is not a license for arbitrary aggression; there's a ceiling on how violently you may draw z.
- **For a flat system, run RRT in the flat output space.** The extension operation becomes straightforward. (See §4.)
- **No general technique exists** to determine whether a new system is flat — it takes cleverness. The single realization that the 3D quadrotor is flat unlocked an era of aggressive flight.

> **💡 Why α (direction (i)) is also part of the definition:** it guarantees z genuinely *belongs to* the system — z must be constructible from the system's own state/inputs, not an arbitrary external signal. For the planar quadrotor α is trivial (z is just part of the state), but in general z can be a nontrivial combination. Flatness = a **two-way dictionary** (α one way; β, γ the other).

> **💡 Non-uniqueness:** the choice of flat outputs need not be unique — any output set satisfying the three requirements works. $(x,y)$ is *a* valid choice, not necessarily the only one.

---

## 4. How RRT and Flatness Connect

**What broke (Lec 8):** Extend draws straight edges in configuration space; with dynamics, those edges can be unexecutable (the car can't slide sideways). A naive RRT tree fills up with edges the robot cannot follow.

**Two repair options:**

- **Option A — integrate real dynamics in Extend (heavy).** From $\bar q_{\text{near}}$, apply some input and integrate $\dot{\bar x}=f(\bar x,\bar u)$ forward. Honest edges, but every Extend now solves a steering / two-point boundary-value problem — expensive (the kinodynamic-RRT world).
- **Option B — run RRT in flat-output space (flatness's gift).**

**Mechanism of Option B:**

1. Sample $\bar z_{\text{rand}}$ in flat-output space.
2. Find nearest tree node $\bar z_{\text{near}}$.
3. **Extend:** connect toward $\bar z_{\text{rand}}$ with a short **smooth curve** (e.g., polynomial segment).
4. Map the curve through β, γ to the actual state/input trajectory; collision-check along it.

In z-space, **every smooth connecting curve is automatically dynamically feasible** — the broken assumption of Lec 8 is restored. Straight-line-style thinking becomes legal again, just in a different space.

> **💡 One-line summary: flatness manufactures a substitute space (z-space) where dynamics constraints vanish; RRT then explores freely in that easy space.** A hard problem in the original coordinates becomes an unconstrained one after a change of coordinates.

**Division of labor (both halves stay essential):**

| Component | Responsibility |
|-----------|----------------|
| RRT (sampling + collision check) | **obstacle avoidance** — Lec 8's "single-pose collision check" machinery unchanged |
| Flatness (β, γ) | **dynamic feasibility** — every edge is executable |

**RRT\* + flatness** (the slides' video): run RRT\* (Lec 8's asymptotically-optimal variant) in z-space → trajectories that are both dynamically feasible *and* approximately optimal. The indoor aggressive-flight videos are exactly this combination: RRT\* grows the tree in $z=(x,y,z,\psi)$ space; flatness back-computes thrusts in real time.

> **Presentation one-liner:** *"RRT explores **where** to go; flatness translates the path into something the robot can **actually fly**."*

---

## Concept Flow & What's Next

```
Lec 8 RRT: assumed "any trajectory is executable"
        ↓ false! a car can't move sideways (dynamics constraints)
Problem: how to make Extend dynamically feasible?
        ↓ easy for a special class of systems
Differential Flatness: pick z̄(t) → x̄ = β(z, ż, ...), ū = γ(z, ż, ...) — everything recovered
        ↓ consequence
every smooth z-trajectory is executable → plan in z-space
        ↓ worked example
Planar quadrotor: z = (x, y); division trick → θ = arctan(−ẍ/(ÿ+g)); inputs need 4th derivatives (→ "minimum snap")
        ↓ Assignment 4 P2
Car: z = (x, y), same proof skeleton (divide → extract θ → differentiate → solve inputs)
        ↓ Lec 10
Planning with dynamics constraints (full integration; obstacles return) + uncertainty → feedback
```

---

## Quick Reference

| Term | Meaning |
|------|---------|
| dynamically feasible | trajectory satisfies the robot's equations of motion |
| differential flatness | ∃ outputs z̄ s.t. all states & inputs are functions of z̄ and finitely many derivatives |
| flat outputs z̄ | the "pen tip": the few signals whose trace encodes the whole system |
| α | maps (state, inputs, input derivatives) → z̄; makes z̄ belong to the system |
| β | maps (z̄, ż̄, …, z̄⁽q⁾) → full state x̄ |
| γ | maps (z̄, ż̄, …, z̄⁽q⁾) → inputs ū |
| q (derivative order) | highest z-derivative needed; sets the smoothness requirement on z̄(t) |
| counting theorem | # flat outputs = # control inputs |
| planar quadrotor | flat with z = (x, y); θ = arctan(−ẍ/(ÿ+g)); q = 4 |
| why not (x, y, θ) | wrong count (3 ≠ 2) and θ is dependent on x, y via eq. (4) |
| minimum snap | minimize 4th derivative of z — because q = 4 feeds inputs directly |
| wacky states | only z is specified; remaining states may swing (add input constraints in practice) |
| RRT + flatness | run RRT (or RRT*) in z-space: smooth edges are automatically feasible |
| kinodynamic alternative | integrate dynamics in Extend — honest but expensive (steering problem) |
| no general test | determining flatness of a new system requires cleverness |
