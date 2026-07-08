# Assignment 4 — Problem 2(b): Flatness of a Hopping Robot (Flight Phase)

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Related lecture:** Lecture 9 (Differential Flatness)
**Source:** Table 2 of the Murray flatness catalog ("Hopping robot — flat output: position of end of leg"); underlying system from R. M. Murray and S. S. Sastry, *Nonholonomic motion planning: Steering using sinusoids*, IEEE TAC 38(5), 1993 [20].

---

## System

A one-legged hopping robot **in flight** (airborne between hops). With no external torque, **angular momentum is conserved** — this single conservation law *is* the dynamics of the orientation.

**Variables:**
- $\psi$ — hip angle (leg angle relative to the body)
- $l$ — leg extension (length beyond the fixed offset)
- $\theta$ — body attitude in the inertial frame
- $d$ — fixed leg-attachment offset; $I$ — body inertia; $m$ — foot/leg mass

**Angular momentum conservation:**

$$I\dot\theta + m(l+d)^2(\dot\theta + \dot\psi) = 0$$

> **💡 Physical picture:** the leg's absolute angle is $\theta+\psi$, so its inertial angular rate is $\dot\theta+\dot\psi$. The body's spin angular momentum plus the leg's must stay zero. This is the **falling-cat / astronaut mechanism**: with zero external torque, changing the robot's *shape* $(\psi, l)$ still reorients its *attitude* θ. The same geometric structure governs satellite attitude control with reaction arms.

**State and inputs (kinematic model):**

$$\bar{x} = (\psi, l, \theta), \qquad \bar{u} = (u_1, u_2) = (\dot\psi, \dot l)$$

Solving the conservation law for $\dot\theta$:

$$\dot\theta\[I + m(l+d)^2] = -m(l+d)^2\\dot\psi \\Rightarrow\ \dot\theta = -\frac{m(l+d)^2}{I+m(l+d)^2}u_1$$

So the dynamics are first-order (like the car in 2(a)):

$$\dot\psi = u_1, \qquad \dot l = u_2, \qquad \dot\theta = -\frac{m(l+d)^2}{I+m(l+d)^2}u_1$$

## Claimed flat output — position of the end of the leg

$$\bar{z} = (x_p, y_p), \qquad x_p = (d+l)\cos(\theta+\psi), \quad y_p = (d+l)\sin(\theta+\psi)$$

**Counting check** (Lec 9 theorem): 2 inputs, 2 flat outputs. OK.

The foot position is a polar-coordinate pair in disguise: radius $r = d+l$, angle $\alpha = \theta+\psi$ (the leg's absolute angle).

---

## Recovery derivation

### Step 1 — leg extension l (radius)

$$r = \sqrt{x_p^2 + y_p^2} = d + l \\Rightarrow\ \boxed{l = \sqrt{x_p^2+y_p^2} - d}$$

*(Zeroth derivatives of z suffice.)*

### Step 2 — input u₂ (radial rate)

$$u_2 = \dot l = \dot r = \boxed{\dfrac{x_p\dot x_p + y_p\dot y_p}{\sqrt{x_p^2+y_p^2}}}$$

*(First derivatives.)*

### Step 3 — input u₁ (via the absolute leg angle α)

The leg's absolute angle is recovered from the foot position:

$$\alpha = \theta + \psi = \arctan\\left(\frac{y_p}{x_p}\right) \\Rightarrow\\dot\alpha = \frac{x_p\dot y_p - y_p\dot x_p}{x_p^2 + y_p^2}$$

Independently, from the dynamics:

$$\dot\alpha = \dot\theta + \dot\psi = u_1\left(1 - \frac{m(l+d)^2}{I+m(l+d)^2}\right) = \frac{I}{I+m(l+d)^2}\,u_1$$

Equating the two expressions and using $(l+d)^2 = x_p^2+y_p^2$ from Step 1:

$$\boxed{u_1 = \frac{I + m(x_p^2+y_p^2)}{I}\cdot\frac{x_p\dot y_p - y_p\dot x_p}{x_p^2+y_p^2}}$$

*(First derivatives. Note the same divide-style move as 2(a): the arctan of a ratio recovers an angle, and its derivative brings in the familiar $x/*/y - y/*/x$ numerator.)*

### Step 4 — attitude θ: **an integral appears**

$\dot\theta$ is available instantaneously — most cleanly straight from the conservation law. Rewriting it with $\dot\alpha = \dot\theta+\dot\psi$ and $r^2 = (l+d)^2 = x_p^2+y_p^2$:

$$I\dot\theta + m r^2 \dot\alpha = 0 \\Rightarrow\ \dot\theta = -\frac{m r^2}{I}\,\dot\alpha = -\frac{m}{I}\big(x_p\dot y_p - y_p\dot x_p\big)$$

(the $r^2$ cancels against the denominator of $\dot\alpha$ — a pleasantly clean expression: the body's spin rate is proportional to the foot's areal velocity, a direct restatement of momentum conservation).

But θ itself requires integrating this from the initial condition:

$$\theta(t) = \theta(0) + \int_0^t \dot\theta(\tau)\, d\tau = \theta(0) - \frac{m}{I}\int_0^t \big(x_p\dot y_p - y_p\dot x_p\big)(\tau)\, d\tau$$

### Step 5 — hip angle ψ

Once θ(t) is known, ψ follows algebraically:

$$\psi(t) = \alpha(t) - \theta(t)$$

---

## ⚠️ The honest caveat: θ needs an *integral*, not just derivatives

Recall the Lec 9 definition of flatness:

$$\bar x = \beta(\bar z, \dot{\bar z}, \ldots, \bar z^{(q)})$$

β must be an **instantaneous function of z and finitely many derivatives — no integrals.** For the quadrotor and the car, every state and input came out of pure differentiation. Here, Steps 1–3 do too ($l$, $u_1$, $u_2$, $\dot\theta$, α) — but **θ itself is only recoverable by a quadrature over the whole past trajectory** (Step 4), and ψ inherits that through Step 5.

> **What this means:** the state θ is a *geometric phase* — the attitude accumulated by shape changes over time, like the net rotation a falling cat achieves by cycling its body shape. Such holonomy-type states are generically **not** expressible as instantaneous functions of any output's derivatives. In the strict Lec 9 sense, this recovery involves one integration, so the system sits at the boundary of the definition — the flat-output structure recovers *everything except one integration constant's worth of information*. This is precisely the class of systems for which Murray & Sastry [20] developed **sinusoidal steering** as an alternative planning technique: cyclic shape inputs produce controlled net drift in θ.

**Why the catalog still lists it as flat:** all *inputs* ($u_1, u_2$) and the shape states ($l$, and ψ up to the θ integral) come from derivatives of the foot position, and the θ quadrature is a known, explicit functional of $z(\cdot)$. In practice one plans the foot trajectory and evaluates the integral along it — the *planning* benefit of flatness survives intact. But a rigorous write-up should state the integral explicitly rather than hide it.

> **Presentation angle:** flagging this distinction ("recovery is by differentiation *plus one quadrature*, unlike the quadrotor/car") demonstrates critical engagement with the definition rather than mechanical reproduction — a strong point to make if this system is chosen for the talk.

---

## Summary

| Step | Quantity | How recovered | Derivatives of z needed | Pure function of derivatives? |
|------|----------|---------------|--------------------------|-------------------------------|
| 1 | $l$ | radius: $\sqrt{x_p^2+y_p^2}-d$ | 0th | yes |
| 2 | $u_2$ | radial rate | 1st | yes |
| 3 | $u_1$ | angle rate ratio (α̇ two ways) | 1st | yes |
| 4 | $\theta$ | **integrate** $\dot\theta$ from $\theta(0)$ | 1st, under an integral | **no — quadrature required** |
| 5 | $\psi$ | $\alpha - \theta$ | 1st + Step 4's integral | inherits Step 4's caveat |

---

## Comparison with 2(a) and the quadrotor

| | Planar quadrotor (Lec 9) | Car — 2(a) | Hopping robot — 2(b) |
|---|---|---|---|
| Dynamics order | 2nd | 1st (kinematic) | 1st (kinematic, from conservation) |
| Flat outputs | COM position $(x,y)$ | position $(x,y)$ | **foot position** $(x_p,y_p)$ |
| Angle recovery | $\arctan(-\ddot x/(\ddot y+g))$ — instantaneous | $\arctan(\dot y/\dot x)$ — instantaneous | $\alpha$ instantaneous, but **θ needs an integral** |
| Key structure | attitude forced by acceleration | heading forced by velocity | attitude = geometric phase of shape changes |
| Highest order | 4 | 2 | 1 (+ one quadrature) |

The recurring pattern: an angle is recovered as the arctan of a ratio of flat-output derivatives. The hopping robot adds a twist — one state (θ) lives *behind* the conservation law and can only be reached by integrating along the trajectory.
