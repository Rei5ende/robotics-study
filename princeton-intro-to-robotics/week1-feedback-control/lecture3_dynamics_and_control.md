# Lecture 3: 3D Dynamics, Linearization & Feedback Control

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Recap & Today's Plan

**Goal (ongoing):** Make a quadrotor hover in place, via two steps:
1. **Dynamics** — how the drone behaves under propeller commands *(Lec 2 + first half of today)*
2. **Feedback Control** — corrective actions when it drifts from hover *(starts today)*

**Last lecture:** Planar quadrotor dynamics + started 3D dynamics.
**Today:** Finish 3D dynamics → Linearization → Begin feedback control.

---

## 1. 3D Quadrotor Dynamics

### State and Control Input

$$\bar{x} = [x, y, z, \phi, \theta, \psi, \dot{x}, \dot{y}, \dot{z}, p, q, r]^T \quad \text{(12 states)}$$

$$\bar{u} = [F_{tot}, M_1, M_2, M_3]^T \quad \text{(4 inputs)}$$

- `[φ, θ, ψ]` — Euler angles (space 1-2-3 convention) = Roll, Pitch, Yaw
- `[p, q, r]` — angular velocity vector in the **body frame**. Direction = instantaneous rotation axis, magnitude = rotation rate. Related to (but not equal to) `φ̇, θ̇, ψ̇`.
- `Ftot` — total thrust; `M₁, M₂, M₃` — moments about body axes `b̄x, b̄y, b̄z`.

> **Key distinction — State vs Dynamics:**
> - **State (Lec 2 §4):** just a *list* of what we track (12 numbers). No derivatives, no laws. Like introducing the cast of characters.
> - **Dynamics (Lec 3 §3):** the *law* describing how those numbers change over time, `ẋ = f(x, u)`. Like the rules of how characters move.

### Motor Model (3D)

In 3D, a spinning rotor produces **both thrust and a moment**:

$$F_i = k_f \omega_i^2 \quad \text{(thrust)}$$
$$M_i^{aero} = k_m \omega_i^2 \quad \text{(aerodynamic drag moment)}$$

- `ωᵢ` here = **the i-th rotor's own spin speed** (not the body's angular velocity p, q, r).
- The drag moment acts about `b̄z` — this is what enables **yaw** control.
- **Adjacent rotors spin in opposite directions** so their drag moments cancel; otherwise the drone would spin uncontrollably.
- New parameter `km` (drag/moment coefficient). Full parameter set is now: `m, I, kf, km`.

### Full 3D Dynamics (sketch)

All dynamics take the form $\dot{\bar{x}} = f(\bar{x}, \bar{u})$. Sketch:

**Position / velocity:**
$$
\ddot{\bar{r}} =
\begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix}
+ \bar{R}
\begin{bmatrix} 0 \\ 0 \\ F_{tot}/m \end{bmatrix}
$$

- `R̄` = **rotation matrix** that takes a vector from body frame → inertial frame.
- Thrust always points along the drone's own up-axis (`b̄z`), so in the body frame it's `[0, 0, Ftot/m]`. `R̄` rotates it into world coordinates based on how the drone is tilted.
- This is the 3D generalization of the planar `ẍ = -(u₁/m)sinθ`, `ÿ = (u₁/m)cosθ - g`. Instead of writing sin/cos by hand, `R̄` packages the tilt.
- **We don't compute `R̄` ourselves** — it's determined automatically once the Euler angles `(φ, θ, ψ)` are known.

**Euler angle rates (kinematic relation):**
$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi\sec\theta & \cos\phi\sec\theta
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

- Converts body-frame angular velocity `[p,q,r]` into Euler angle rates `[φ̇, θ̇, ψ̇]`.
- *(The lecture notes only wrote out the first row and left the rest as "...". The full matrix above is the standard space 1-2-3 form, shown here for completeness — no need to memorize it.)*
- They differ because Euler angles are applied *sequentially* (Roll→Pitch→Yaw), so each rotation's axis is offset by the previous ones — hence the sin/cos/tan correction terms.
- **This matrix is NOT `R̄`.** `R̄` rotates a *vector*; this matrix converts *angular velocity units*. Different mathematical objects.
- This conversion is **convention-dependent** — using a different rotation order changes the correction terms.

**Angular velocity dynamics:**
$$
\dot{\bar{\omega}}_{BW} = \mathbf{I}^{-1}
\left[
-\bar{\omega}_{BW} \times (\mathbf{I}\bar{\omega}_{BW})
+ \begin{bmatrix} M_1 \\ M_2 \\ M_3 \end{bmatrix}
\right]
$$

- The 3D version of planar `θ̈ = u₂/I`. Here `I` is a matrix, so we use its inverse.
- The `-ω̄ × (Iω̄)` term is the **gyroscopic effect** — rotation about one axis influences the others (like a figure skater spinning faster when pulling arms in). Doesn't exist in planar because there's only one rotation axis.

**Inertia matrix:**
$$
\mathbf{I} =
\begin{bmatrix}
I_{xx} & 0 & 0 \\
0 & I_{yy} & 0 \\
0 & 0 & I_{zz}
\end{bmatrix}
$$

- Off-diagonal terms (products of inertia) are zero **not just because the drone is symmetric, but because the body axes `b̄x, b̄y, b̄z` are chosen to align with the principal axes.** Axis choice — not just the object — determines whether off-diagonals vanish. Every rigid body has principal axes that diagonalize `I`; the designer deliberately defines the body frame along them.

> *Note: The course doesn't expect us to derive these. The point is to recognize they all fit $\dot{\bar{x}} = f(\bar{x}, \bar{u})$.*

---

## 2. Linearization

### Why linearize?

The dynamics are **nonlinear** (sin, cos, cross products), which makes control hard. We approximate them with a **linear model valid near hover**.

> Analogy: the Earth is round, but a local map drawn flat is good enough over a small area. Linearization is accurate only in a small region around hover.

### Nominal (reference) point — hover

$$
\bar{x}_0 =
\begin{bmatrix} x_0 \\ y_0 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix},
\quad
\bar{u}_0 =
\begin{bmatrix} mg \\ 0 \end{bmatrix}
$$

- Same as the hover condition: `u₁ = mg`, `u₂ = 0`, all velocities and angles zero.
- Key property: `f(x̄₀, ū₀) = 0̄` — start at hover, apply hover input, nothing changes.
- `x₀, y₀` can be set to **0** for convenience by shifting the coordinate origin. This is allowed because the dynamics **don't depend on absolute position** (the x and y columns of `A` are all zero). In contrast, `θ₀ = 0` is **required** (dynamics genuinely depend on θ via sin/cos).

### Taylor expansion (1st-order)

A function can be approximated near a point by its tangent:
$$f(x) \approx f(x_0) + f'(x_0)(x - x_0)$$

Applied to the dynamics:

$$\dot{\bar{x}} = f(\bar{x}, \bar{u}) \approx \underbrace{f(\bar{x}_0, \bar{u}_0)}_{=\,0} + \underbrace{\frac{\partial f}{\partial \bar{x}}\bigg|_{0}}_{:=\,A}(\bar{x} - \bar{x}_0) + \underbrace{\frac{\partial f}{\partial \bar{u}}\bigg|_{0}}_{:=\,B}(\bar{u} - \bar{u}_0)$$

We keep only the 1st-order (linear) terms; near hover the deviations are small, so higher-order terms are negligible.

### A and B are Jacobians

With multiple variables, the "slope" becomes a matrix of partial derivatives (Jacobian):

$$A_{ij} = \frac{\partial(\text{i-th equation})}{\partial(\text{j-th state})}, \quad B_{ij} = \frac{\partial(\text{i-th equation})}{\partial(\text{j-th input})}$$

> **On differentiating w.r.t. `u₁`:** In the control view, `u₁` is treated as an **independent input variable** (a dial we set), not as the function `kf(ω₁²+ω₂²)`. That's why `∂f/∂u₁` is straightforward — we momentarily set aside how the motors realize `u₁`.

### Planar A and B (after substituting hover)

$$
A =
\begin{bmatrix}
0&0&0&1&0&0 \\
0&0&0&0&1&0 \\
0&0&0&0&0&1 \\
0&0&-g&0&0&0 \\
0&0&0&0&0&0 \\
0&0&0&0&0&0
\end{bmatrix},
\quad
B =
\begin{bmatrix}
0&0 \\
0&0 \\
0&0 \\
0&0 \\
\frac{1}{m}&0 \\
0&\frac{1}{I}
\end{bmatrix}
$$

**Physical meaning of the nonzero entries:**

| Entry | Location | Meaning |
|-------|----------|---------|
| `1` (×3) | top block of A | position's rate of change = velocity (`ẋ = vₓ`, kinematic identity) |
| `-g` | A, row 4 col 3 | **a small tilt θ produces horizontal acceleration** — derived from `∂(-u₁/m·sinθ)/∂θ = -u₁/m·cosθ`, which at hover (`u₁=mg, θ=0`) gives `-g`. This is *why a drone must tilt to move sideways* — the bridge between tilt and translation. |
| `1/m` | B, row 5 col 1 | thrust → vertical accel (`ÿ = u₁/m`); lighter drone = more responsive |
| `1/I` | B, row 6 col 2 | torque → angular accel (`θ̈ = u₂/I`); smaller inertia = more responsive |

Both `1/m` and `1/I` are "inverse of inertia" — larger inertia means more sluggish response to input.

### Planar vs 3D — same recipe, different scale

| | Planar | 3D |
|---|---|---|
| States | 6 | 12 |
| Inputs | 2 | 4 |
| A size | 6×6 | 12×12 |
| B size | 6×2 | 12×4 |
| Nonlinearity | sin θ, cos θ only | many trig products + gyroscopic |
| By hand? | ✅ feasible (for understanding) | ❌ use Python (Assignment 1) |
| Axis coupling | none (1 rotation axis) | yes (gyroscopic) |

The **method is identical** (differentiate `f`, evaluate at hover); only size and complexity grow.

### Benefits of linearization

1. **Unlocks the entire linear control toolbox** — PD (Lec 4), LQR (Lec 5), pole placement.
2. **Stability is decidable** — just inspect eigenvalues of `A` (real part < 0 ⇒ stable).
3. **Fast computation** — matrix ops run the 500 Hz hover loop cheaply, vs. repeated sin/cos.
4. **Simplicity & intuition** — superposition holds; each matrix entry has clear physical meaning.

Trade-off: accurate only **near hover**. Fine for this course's goal (hovering), not for aggressive maneuvers.

---

## 3. Feedback Control

### What it is (§5)

Feedback control is the **sense–think–act** cycle:

```
Sense state x̄(t)  →  Compare to desired x̄₀  →  Choose input ū  →  Repeat
                  (~500 Hz for quadrotor hover)
```

**Control law:**
$$\bar{u}(\bar{x}) = \bar{u}_0 + K(\bar{x} - \bar{x}_0)$$

| Term | Meaning |
|------|---------|
| `ū₀` | baseline input to hold hover (`mg`, `0`) |
| `x̄ - x̄₀` | deviation from desired state (error) |
| `K` | gain matrix — how strongly to correct the error |

- If already at target (`x̄ = x̄₀`) → apply only `ū₀`.
- If off-target → add a correction proportional to the deviation.
- The whole game of control theory is choosing `K` in a principled way.

### Why we need it (§6)

To deal with **uncertainty**:
- Uncertainty in **initial conditions** (most direct case)
- **External disturbances** (e.g., wind gusts)
- **Model parameters** (`kf, km, I` not known exactly)
- **State estimation** error (sensors imperfect — covered later in the course)

In a perfect world we could open-loop our way to hover; feedback exists because reality is uncertain, and it cancels these disturbances in real time.

---

## Concept Flow — How Lec 3 Hangs Together

```
Goal: hover
  ↓ need to know how it moves first
3D Dynamics:  ẋ = f(x,u)   ← nonlinear (sin/cos, gyroscopic)
  ↓ nonlinear is hard to control → simplify
Linearization:  ẋ = A(x-x₀) + B(u-u₀)   ← via Taylor expansion at hover
  ↓ now linear → control theory applies
Feedback Control:  u(x) = u₀ + K(x-x₀)   ← corrects for uncertainty
  ↓ remaining question: how to choose K?
Lec 4–5:  PD Control, LQR   ← uses A, B to design K
```

Each stage is a prerequisite for the next: no dynamics ⇒ nothing to linearize; no linearization ⇒ no linear control tools; no control ⇒ no hover. The `A, B` matrices built here become the raw material for designing `K` next lecture.

---

## Notation Reference

| Symbol | Meaning |
|--------|---------|
| `x̄` (bar) | a **vector** (e.g., full state); bar distinguishes it from scalar `x` (horizontal position) |
| `[...]ᵀ` | transpose — written as a row but actually a column vector |
| `≜` | "is defined as" (a naming declaration, not an equation) |
| `x̄₀, ū₀` | nominal (hover) state and input |
| `A, B` | linearized system matrices (Jacobians of `f`) |
| `K` | feedback gain matrix |
| `R̄` | rotation matrix (body → inertial) |
| `[p, q, r]` | body-frame angular velocity |
| `I` (matrix) | inertia matrix |
| `ωᵢ` (motor model) | i-th rotor spin speed |
