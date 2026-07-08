# Assignment 4 — Problem 2(b): Flatness of a Fully Actuated Mechanical System

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Related lecture:** Lecture 9 (Differential Flatness)
**Source:** Table 2 of the Murray flatness catalog; underlying result from R. M. Murray, *Nonlinear control of mechanical systems: A Lagrangian perspective*, NOLCOS 1995 [18].

---

## System

A general Lagrangian mechanical system in Euler-Lagrange form:

$$M(q)\ddot{q} + N(q, \dot{q}) = \tau$$

- $q \in \mathbb{R}^n$ — **generalized coordinates** (joint angles, positions, ...)
- $M(q)$ — mass/inertia matrix ($n \times n$, symmetric positive definite)
- $N(q,\dot{q})$ — Coriolis, centrifugal, gravity, damping terms lumped together
- $\tau \in \mathbb{R}^n$ — generalized forces/torques (**inputs**)

> **"Fully actuated"** means the input has the same dimension as the configuration: $\tau \in \mathbb{R}^n$ for $q \in \mathbb{R}^n$. Every degree of freedom has its own independent actuator — the canonical example is a robot arm with a motor at every joint. (Contrast: the quadrotor and the car are *underactuated* — fewer inputs than degrees of freedom.)

## Claim

The system is differentially flat with flat outputs:

$$\bar{z} = q$$

**Counting check** (Lec 9 theorem): # inputs = n, # flat outputs = n. OK.

---

## Proof

### Step 1 — β is trivial

The flat output *is* the configuration, so state recovery is the identity map. The state of this second-order system is $(q, \dot q)$:

$$q = z, \qquad \dot{q} = \dot{z}$$

### Step 2 — γ by direct substitution

Solve the equation of motion for τ — it is already solved:

$$\tau = M(q)\ddot{q} + N(q,\dot{q})$$

Substituting $q = z$, $\dot q = \dot z$, $\ddot q = \ddot z$:

$$\boxed{\tau = M(z)\ddot{z} + N(z, \dot{z})}$$

$M$ and $N$ are known functions of the system, so given any $z(t)$ (twice differentiable), the required torque history $\tau(t)$ follows by **one algebraic substitution** — no elimination tricks, no solving of equations.

**Conclusion:** states $(q,\dot q)$ are trivially $z, \dot z$; inputs τ are an algebraic function of $z, \dot z, \ddot z$. Highest derivative order q = 2. **Differentially flat.** ∎

> **⚠️ Why is this so much easier than 2(a)?** For the quadrotor and the car, elimination tricks (divide, square-and-add) were needed because those systems are *underactuated*: with fewer inputs than degrees of freedom, information about unactuated coordinates must be squeezed out indirectly (acceleration → tilt). Full actuation means each degree of freedom is directly commanded, so the equation of motion is *already* solved for the input. **Flatness of a fully actuated system is essentially trivial; the cleverness is only ever needed for underactuated ones.**

---

## Concrete instance — 2-link planar robot arm

To make the general theorem tangible: a planar arm with two joints $q = (q_1, q_2)$ and two motor torques $\tau = (\tau_1, \tau_2)$ (one per joint → fully actuated). The dynamics take exactly the form above with

$$M(q) = \begin{bmatrix} m_{11}(q_2) & m_{12}(q_2) \\ m_{12}(q_2) & m_{22} \end{bmatrix}$$

and $N(q,\dot q)$ collecting Coriolis and gravity terms. Flat outputs: the joint angles themselves. Draw any smooth joint trajectory $z(t) = (q_1(t), q_2(t))$; the torques to execute it are $\tau = M(z)\ddot z + N(z,\dot z)$.

> **💡 This formula has a famous name in robotics: computed torque control.** The flatness statement "inputs are an algebraic function of the desired output trajectory" *is* the computed-torque idea — flatness gives it a trajectory-planning interpretation. This is why Table 2 comments that this row "covers robot manipulators."

---

## Summary

| Step | Move | Result | Derivative order |
|------|------|--------|------------------|
| 0 | Counting check | n inputs, n flat outputs — plausible | — |
| 1 | Identity | $q = z$, $\dot q = \dot z$ | 1st |
| 2 | Substitute into EOM | $\tau = M(z)\ddot z + N(z,\dot z)$ | 2nd |
| — | Conclusion | **differentially flat**, q = 2, zero tricks required | — |

---

## Comparison across all Assignment-4 flatness proofs

| | Planar quadrotor (Lec 9) | Car — 2(a) | Fully actuated arm — 2(b) |
|---|---|---|---|
| Actuation | underactuated (2 inputs, 3 DoF) | underactuated (2 inputs, 3 states) | **fully actuated** (n = n) |
| Flat outputs | $(x,y)$ — required insight | $(x,y)$ — required insight | $q$ itself — **obvious** |
| Key technique | divide to kill $(F_1{+}F_2)$ | divide to kill $u_1$ | none — EOM already solved for τ |
| Highest order q | 4 ("minimum snap") | 2 (curvature) | 2 |

> **Presentation one-liner:** *"Flatness does not always demand cleverness — for fully actuated systems it is nearly a tautology; the genuine insight is only needed for underactuated systems like the quadrotor and the car."* This contrast ties 2(a) and 2(b) into one story.
