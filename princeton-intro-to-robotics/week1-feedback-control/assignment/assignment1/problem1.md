# Assignment 1 — Problem 1: State-Space Conversion and Linearization

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Topic:** Converting second-order EOMs to first-order state-space form, then linearizing about an equilibrium.

---

## General Method: Counting States and Inputs

Before working the two systems below, the counting rule used throughout:

> **State count = (number of independent generalized coordinates) × (highest derivative order).** For a standard mechanical system where every coordinate obeys a 2nd-order ODE, this is simply **2 × (degrees of freedom)** — position and velocity for each coordinate. Intuitively: an *n*th-order ODE needs *n* initial conditions to pin down a unique solution, and those *n* numbers are exactly the state.
>
> **Input count** is unrelated to derivative order — it is just the number of independent control channels (actuators) acting on the system.

This is the same pattern behind the general Lagrangian form $M(q)\ddot q + N(q,\dot q) = \tau$: states are always $(q,\dot q) \in \mathbb{R}^{2n}$ for $n$ generalized coordinates, regardless of how many inputs $\tau$ there are.

---

## Problem 1 (MAE 345) — Pendulum

### System

$$\ddot\theta = -\frac{g}{l}\sin\theta + u$$

One generalized coordinate ($\theta$), one control input (torque $u$).

### (a) States, inputs, first-order form

**State count:** 1 coordinate × 2nd order = **2 states**. **Input count:** **1** ($u$).

$$\bar x = \begin{bmatrix}x_1\\x_2\end{bmatrix} = \begin{bmatrix}\theta\\\dot\theta\end{bmatrix}, \qquad \bar u = u$$

Converting to first order ($\dot x_1=\dot\theta=x_2$; $\dot x_2 = \ddot\theta$):

$$\dot{\bar x} = f(\bar x,\bar u) = \begin{bmatrix} x_2 \\ -\dfrac{g}{l}\sin x_1 + u \end{bmatrix}$$

### (b) Linearization about $\theta_0=0,\dot\theta_0=0,u_0=0$

Check the reference is an equilibrium: $f(\bar x_0,u_0) = \begin{bmatrix}0\\-\frac{g}{l}\sin0+0\end{bmatrix} = \begin{bmatrix}0\\0\end{bmatrix}$ ✓ (needed so the linear deviation dynamics has no constant term).

$$A = \frac{\partial f}{\partial \bar x}\bigg|_{\bar x_0,u_0} = \begin{bmatrix} \dfrac{\partial x_2}{\partial x_1} & \dfrac{\partial x_2}{\partial x_2} \\[4pt] \dfrac{\partial}{\partial x_1}\!\left(-\frac{g}{l}\sin x_1+u\right) & \dfrac{\partial}{\partial x_2}\!\left(-\frac{g}{l}\sin x_1+u\right) \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -\dfrac{g}{l}\cos x_1 & 0 \end{bmatrix}$$

At $x_1=\theta_0=0$: $\cos 0 = 1$, so:

$$\boxed{A = \begin{bmatrix} 0 & 1 \\ -g/l & 0 \end{bmatrix}}$$

$$B = \frac{\partial f}{\partial u}\bigg|_{\bar x_0,u_0} = \begin{bmatrix} \partial x_2/\partial u \\ \partial(-\frac{g}{l}\sin x_1+u)/\partial u \end{bmatrix} = \boxed{\begin{bmatrix}0\\1\end{bmatrix}}$$

**Linearized dynamics:**

$$\dot{\tilde x} = \begin{bmatrix}0&1\\-g/l&0\end{bmatrix}\tilde x + \begin{bmatrix}0\\1\end{bmatrix}\tilde u, \qquad \tilde x = \bar x-\bar x_0,\ \tilde u=\bar u-u_0$$

> **Sign check:** $A_{21}=-g/l<0$ — a small positive $\theta$ deviation produces negative angular acceleration, pulling the pendulum back toward $\theta=0$. This is the **hanging-down equilibrium**, stable without any control (restoring, not amplifying).

---

## Problem 1 (MAE 549) — Cart-Pole

### System

$$2\ddot x + \ddot\theta\cos\theta - \dot\theta^2\sin\theta = u \qquad (2)$$
$$\ddot x\cos\theta + \ddot\theta - \sin\theta = 0 \qquad (3)$$

Two generalized coordinates ($x$, $\theta$), one control input (force $u$ on the cart). Note $u$ is **underactuated** relative to the two coordinates — this is exactly why (2),(3) must be solved *together* rather than one at a time.

### (a) States, inputs, first-order form

**State count:** 2 coordinates × 2nd order = **4 states**. **Input count:** **1** ($u$).

$$\bar x = \begin{bmatrix}x_1\\x_2\\x_3\\x_4\end{bmatrix} = \begin{bmatrix}x\\\dot x\\\theta\\\dot\theta\end{bmatrix}, \qquad \bar u = u$$

**Step 1 — isolate $\ddot x,\ddot\theta$.** Equations (2),(3) are linear in $(\ddot x,\ddot\theta)$ but coupled; write them as $M\begin{bmatrix}\ddot x\\\ddot\theta\end{bmatrix}=\bar b$:

$$\begin{bmatrix}2 & \cos\theta\\ \cos\theta & 1\end{bmatrix}\begin{bmatrix}\ddot x\\\ddot\theta\end{bmatrix} = \begin{bmatrix}u+\dot\theta^2\sin\theta\\ \sin\theta\end{bmatrix}$$

**Step 2 — solve via Cramer's rule.** $\det M = 2-\cos^2\theta \ge 1 > 0$ for all $\theta$ (never singular). Replacing column 1 or 2 of $M$ with $\bar b$ and dividing by $\det M$:

$$\ddot x = \frac{u+\dot\theta^2\sin\theta-\cos\theta\sin\theta}{2-\cos^2\theta}, \qquad \ddot\theta = \frac{2\sin\theta-\cos\theta\,(u+\dot\theta^2\sin\theta)}{2-\cos^2\theta}$$

**Step 3 — assemble first-order system** ($\dot x_1=x_2$, $\dot x_3=x_4$):

$$\dot{\bar x} = f(\bar x,\bar u) = \begin{bmatrix} x_2 \\[4pt] \dfrac{u+x_4^2\sin x_3-\cos x_3\sin x_3}{2-\cos^2 x_3} \\[8pt] x_4 \\[4pt] \dfrac{2\sin x_3-\cos x_3\,(u+x_4^2\sin x_3)}{2-\cos^2 x_3} \end{bmatrix}$$

### (b) Linearization about $x_0=0,\dot x_0=0,\theta_0=0,\dot\theta_0=0,u_0=0$

**Shortcut:** small-angle approximate the *original* equations directly ($\sin\theta\approx\theta$, $\cos\theta\approx1$, drop the quadratic term $\dot\theta^2\sin\theta\approx0$ since it's already 2nd order in deviations):

$$2\ddot x+\ddot\theta = u \quad (2') \qquad \ddot x+\ddot\theta = \theta \quad (3')$$

$(2')-(3')$: $\ddot x = u-\theta$. Substituting into $(3')$: $\ddot\theta = \theta-\ddot x = \theta-(u-\theta) = 2\theta-u$.

$$\boxed{\ddot x = -\theta+u, \qquad \ddot\theta = 2\theta-u}$$

> **Consistency check:** differentiating the exact $\ddot x,\ddot\theta$ from part (a) (Jacobian at the origin) gives the same coefficients: $\partial\ddot x/\partial\theta=-1$, $\partial\ddot x/\partial u=1$, $\partial\ddot\theta/\partial\theta=2$, $\partial\ddot\theta/\partial u=-1$, all other partials zero at $\theta_0=\dot\theta_0=u_0=0$. Both routes agree.

**First-order deviation equations** ($\dot x_1=x_2,\ \dot x_2=-x_3+u,\ \dot x_3=x_4,\ \dot x_4=2x_3-u$):

$$A = \begin{bmatrix} 0&1&0&0\\ 0&0&-1&0\\ 0&0&0&1\\ 0&0&2&0 \end{bmatrix}, \qquad B = \begin{bmatrix}0\\1\\0\\-1\end{bmatrix}$$

$$\dot{\tilde x} = A\tilde x + B\tilde u$$

> **Sign check:** $A_{43}=+2>0$ — a small $\theta$ deviation produces angular acceleration in the *same* direction, growing the deviation further. This is the **inverted-pendulum equilibrium** (pole balanced upright): open-loop unstable, requiring feedback (e.g. LQR) to stabilize. Contrast with the plain pendulum above, where the analogous entry was negative (stable, self-restoring).

---

## Summary

| | Pendulum (MAE 345) | Cart-Pole (MAE 549) |
|---|---|---|
| Generalized coordinates | 1 ($\theta$) | 2 ($x,\theta$) |
| States | 2 | 4 |
| Inputs | 1 | 1 |
| Coupling in original EOM | none (single 2nd-order ODE) | coupled (solved via Cramer's rule) |
| $A$ at linearization point | $\begin{bmatrix}0&1\\-g/l&0\end{bmatrix}$ | $\begin{bmatrix}0&1&0&0\\0&0&-1&0\\0&0&0&1\\0&0&2&0\end{bmatrix}$ |
| $B$ | $\begin{bmatrix}0\\1\end{bmatrix}$ | $\begin{bmatrix}0\\1\\0\\-1\end{bmatrix}$ |
| Equilibrium type | stable (hanging down) | unstable (inverted) |
