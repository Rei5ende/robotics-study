# Lecture 10: Planning with Dynamics Constraints

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lecture 9 covered only **differentially flat** systems — a broad but still special class. Today fills in the rest: **(Part 1)** how to plan for systems that are *not* necessarily flat, by extending RRT to handle general dynamics, and **(Part 2)** what to do when the planned trajectory can't be followed perfectly due to uncertainty.

> **One-line summary:** Lec 9 solved the easy case (flat systems); Lec 10 covers the general case (RRT + dynamics, three approaches) and closes the loop back to Control by adding feedback (TVLQR) to correct for deviations.

> **Where this sits:** the lecture explicitly frames Lec 9 + Lec 10-Part-1 as one topic — "motion planning with differential constraints" — with flatness as the easy special case and today's RRT extensions as the general-purpose fallback.

---

## Part 1 — RRT with Dynamics: Three Ideas

Recall the broken piece from Lec 8/9: Extend drew a **straight line** toward $\bar q_{\text{rand}}$, which assumed the robot could follow it — false for systems with dynamics. Lec 9's fix (flatness) only works for flat systems. Today: what if the system isn't flat?

### Idea 1 — Random-Propagate

```
V = {x̄_A}
each iteration:
    x̄_rand  ← pick an existing tree node at random          (NOT a fresh sample!)
    ū_rand  ← sample a random control input
    x̄_s = x̄_rand + ∫₀^ΔT f(x̄(t), ū_rand) dt                  (forward-simulate)
    collision-check x̄_s; if free, add to tree; check goal
```

> **⚠️ x̄_rand here means something different from Lec 8's q̄_rand!** In Lec 8, $\bar q_{\text{rand}}$ was a *fresh sample from the whole space*, acting as a direction signal pulling the tree toward unexplored regions. Here, $\bar x_{\text{rand}}$ is just **an existing tree node chosen at random** — there is no "aim for unexplored space" signal at all. This is close to a pure random walk from a random existing point.

**Result:** provably probabilistically complete, but **doesn't work well in practice**. The slide's figure (Tedrake, *Underactuated Robotics*) shows the tree tangling into a knot near the start while the goal sits far away, essentially unreached.

> **💡 Why it fails — connect to Lec 8.** RRT was "Rapidly-Exploring" because $\bar q_{\text{rand}}$ was drawn from the *entire* space, so unexplored regions were naturally more likely to be targeted. Idea 1 throws that mechanism away — there's no direction being aimed at, so the tree drifts locally instead of reaching outward. This failure is exactly what motivates Idea 2.

### Idea 2 — Discretize (sample-and-pick-best Extend)

Keep Lec 8's sampling exactly (fresh $\bar x_{\text{rand}}$ from the whole space, find nearest $\bar x_{\text{near}}$). Only change **how Extend is approximated**: instead of solving for the input that reaches $\bar x_{\text{rand}}$ exactly, **try several candidate inputs and keep whichever gets closest**.

```
x̄_rand ~ sample whole space (Lec 8 style)      x̄_near ← nearest tree node
for several sampled control input candidates ū_i:
    simulate x̄_near forward by ΔT under ū_i
keep the candidate whose result is closest to x̄_rand  →  x̄_s
```

> **Both x̄_rand *and* the control-input candidates are randomly sampled** — a "mini Monte Carlo search": can't solve for the exact steering input, so throw several darts and keep the best.

**Two things determine how well this works:**

1. **Works best with few control inputs.** Few input dimensions → candidate set is cheap to sample densely. Many inputs → combinatorial blow-up.
2. **How x̄_near is found (the distance metric) matters a lot.**

> **⚠️ Euclidean distance in state space can be misleading.** Example systems: a **pendulum** (359° and 1° are 2° apart physically, but 358° apart in naive Euclidean angle-distance — the same wraparound issue as Lec 8's θ ∈ S¹) or a **car** (mixing position units and velocity units in one Euclidean norm treats physically incomparable quantities as equal). Better, dynamics-informed distance metrics (e.g., reachability-based) improve search efficiency — the lecture doesn't derive one, but flags that "Euclidean is not automatically correct."

### Idea 3 — Steer (solve the boundary value problem exactly)

Keep the RRT skeleton; change Extend once more — this time, **solve exactly** for the control input(s) that drive the system from $\bar x_{\text{near}}$ to $\bar x_{\text{s}}$:

> **Boundary Value Problem (BVP):** find $\bar u(t)$ that drives the system from $\bar x_{\text{near}}$ to $\bar x_{\text{s}}$. Also called **trajectory optimization**. Tackled with nonlinear optimization techniques (not covered in depth — "good to know these exist," active research area).

The resulting edge in the slide figure is a genuine curve (not a straight dotted line) — it respects the dynamics exactly.

> **⚠️ Trade-off:** solving the BVP is usually **computationally heavy** and works best over **short time horizons**. But if it can be solved efficiently, the resulting RRT works well.

### The three ideas, one word each

| Idea | One word | What it does |
|------|-----------|---------------|
| 1 | **Random-Propagate** | shoot a random input from a random tree node, no aiming |
| 2 | **Discretize** | sample several candidate inputs, keep the closest-to-target one |
| 3 | **Steer** | solve exactly for the input connecting near → target (standard term in kinodynamic-RRT literature: "steering function/problem") |

> **💡 Connection to Lec 9:** flatness is essentially a **free, closed-form special case of Idea 3**. A flat system's BVP is solved instantly by differentiation (no numerical optimization needed); a non-flat system has no such shortcut, so it must fall back to Idea 3's real optimization or Idea 2's approximation.

### Worked examples (slides)

- **Acrobot** — underactuated double pendulum (no actuator at the "shoulder," only at the "elbow"). Trajectory found via **trajectory optimization** (Idea 3), then tracked with **time-varying LQR** (→ Part 2). This exact pairing — plan once with optimization, track with TVLQR — is the bridge to Part 2.
- **Airplane** — "Fast and Accurate Knife-Edge Maneuvers" and "Funnel Libraries for Realtime Obstacle Avoidance": trajectory optimization plus feedback control, deployed to real-time obstacle avoidance via precomputed trajectory libraries with safety margins ("funnels").

---

## Part 2 — Dealing with Uncertainty: Time-Varying LQR

### Why feedback is needed

Every planning method so far assumed the planned trajectory is followed *perfectly*. Not true in reality:
- exact initial conditions may be unknown
- the dynamics model $f$ is imperfect
- external disturbances exist (e.g., wind)

> **💡 This is the second half of Lec 9's forward-looking promise** — recall Lec 9 flagged "today and next lecture: dynamics" and "next lecture: uncertainty → feedback." This is that promise being kept.

### The fix — feedback proportional to deviation

Given a reference trajectory $\bar x_0(t), \bar u_0(t)$ (e.g., from RRT) and the actual state $\bar x(t)$ at time $t$:

$$\bar u(t) = \bar u_0(t) + K(t)\big[\bar x(t) - \bar x_0(t)\big]$$

> **💡 Intuition:** add a correction proportional to "how far off track we are" on top of the nominal input — spring-like, pulling the system back toward the reference. $K(t)$ is the strength of that pull, and it's allowed to vary in time.

### Why the reference is split as $\bar x = \bar x_0 + \tilde x$ (and $\bar u = \bar u_0 + \tilde u$)

This split exists **specifically to set up linearization**. Linearizing $f$ requires a well-defined point to expand around; writing $\bar x(t) = \bar x_0(t) + \tilde x(t)$ makes that point explicit, so a Taylor expansion of $f(\bar x_0+\tilde x,\, \bar u_0+\tilde u)$ around $(\bar x_0,\bar u_0)$ becomes:

$$f(\bar x_0+\tilde x,\, \bar u_0+\tilde u) \approx f(\bar x_0,\bar u_0) + A(t)\tilde x + B(t)\tilde u$$

Because the reference trajectory itself satisfies the dynamics ($\dot{\bar x}_0 = f(\bar x_0,\bar u_0)$ by definition), this term **exactly cancels** the $-\dot{\bar x}_0(t)$ left over from differentiating $\tilde x = \bar x - \bar x_0$, leaving a clean **linear deviation dynamics**:

$$\dot{\tilde x}(t) \approx A(t)\tilde x(t) + B(t)\tilde u(t)$$

> **This is the whole point of the split: it turns a nonlinear tracking problem into a linear one purely about the deviation**, so the standard LQR machinery (cost function, Riccati equation) can be applied directly to $(\tilde x, \tilde u)$.

### What "Riccati equation" means (plain-language)

The cost functional to minimize:

$$\int_0^T \Big[\tilde x^T Q(t)\tilde x(t) + \tilde u^T R(t)\tilde u(t)\Big] dt$$

The key fact that makes LQR tractable: the optimal cost-to-go from any state $\tilde x(t)$ turns out to have the clean quadratic form $\tilde x(t)^T S(t)\tilde x(t)$, for some matrix $S(t)$. **The Riccati equation is the differential equation $S(t)$ must satisfy** for this to hold:

$$-\dot S(t) = Q(t) - S(t)B(t)R^{-1}(t)B^T(t)S(t) + S(t)A(t) + A^T(t)S(t)$$

Once $S(t)$ is known, the optimal gain drops out algebraically: $K^*(t) = R^{-1}(t)B^T(t)S(t)$.

> **💡 Intuition — "how hard to correct depends on how much time is left."** Picture a ship off course: how sharply to turn now should depend on the remaining voyage and what lies ahead. $S(t)$ encodes exactly that "value of correcting, given time remaining." This is also why the boundary condition $S(T)=S_f$ is given at the **end** and the equation is integrated **backward** in time — the future must be known before deciding what to do now.

> **⚠️ Contrast with ordinary (non-time-varying) LQR.** Standard LQR linearizes around a single **fixed** equilibrium, giving constant $A, B$ and an algebraic (not differential) Riccati equation. TVLQR linearizes around a **moving** reference trajectory, so $A(t), B(t)$ change with time and the Riccati equation becomes a genuine matrix ODE, solved numerically (e.g., `ode45`, `scipy.integrate`) backward from $t=T$ to $t=0$.

### Final controller

$$\bar u(t) = \bar u_0(t) + K^*(t)\big[\bar x(t)-\bar x_0(t)\big]$$

> **💡 Why "trajectory optimization + TVLQR" is the standard real-world combo:** solve the (expensive) BVP/trajectory optimization **once**, offline, to get a good nominal trajectory; then use TVLQR only for **cheap, real-time correction** during execution. No need to re-optimize every instant. The Acrobot example is exactly this pairing.

---

## Concept Flow & What's Next

```
Lec 6-7: grid search (point robot, discrete)
Lec 8: RRT — continuous space, but straight-line Extend ignores dynamics (the gap)
Lec 9: Flatness — for flat systems, the gap closes for free in z-space (easy special case)
Lec 10 Part 1: general (non-flat) systems → RRT + dynamics, three approaches
   (Idea 1 random walk fails → Idea 2 approximate Extend → Idea 3 exact BVP/trajectory optimization)
Lec 10 Part 2: executing the planned trajectory under uncertainty → feedback (TVLQR)
        ↓
Full pipeline closes: Planning (6-10) → Tracking (TVLQR) reconnects to the Control module
```

> **💡 Presentation one-liner:** *"Lectures 6-8 decided where to go; Lectures 9-10's first half made the path physically executable; Lecture 10's second half closes the loop by correcting for what goes wrong during execution."* This completes the Lec 6 promise that "Planning decides the path, Control makes the robot follow it."

---

## Quick Reference

| Term | Meaning |
|------|---------|
| RRT Idea 1 (Random-Propagate) | random input from a random existing tree node; no exploration bias; fails in practice |
| RRT Idea 2 (Discretize) | Lec-8-style sampling kept; Extend approximated by trying several random input candidates |
| RRT Idea 3 (Steer) | Extend solved exactly via BVP / trajectory optimization; heavy but high quality |
| distance metric caveat | Euclidean distance in state space can misrepresent "closeness" (pendulum angle wraparound, mixed units) |
| BVP / trajectory optimization | find control inputs connecting two states exactly; flatness is a free special case of this |
| feedback correction | $\bar u = \bar u_0 + K(t)[\bar x-\bar x_0]$ — proportional pull back toward the reference |
| deviation variables $\tilde x,\tilde u$ | $\bar x=\bar x_0+\tilde x$, split specifically to enable linearizing $f$ around the moving reference |
| $A(t), B(t)$ | Jacobians of $f$ at the reference trajectory point at time $t$ — time-varying, unlike fixed-point LQR |
| cost functional (Q,R integral) | the quantity minimized by choice of input |
| $S(t)$ | matrix encoding optimal cost-to-go as $\tilde x^TS(t)\tilde x$ |
| Riccati equation | the ODE $S(t)$ must satisfy; solved backward in time from $S(T)=S_f$ |
| $K^*(t)=R^{-1}B^TS(t)$ | optimal time-varying feedback gain, derived from $S(t)$ |
| TVLQR | Riccati-based, time-varying feedback control for tracking a reference trajectory under model/disturbance uncertainty |
