# Assignment 4 — Problem 2(a): Differential Flatness of a Wheeled Robot

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Related lecture:** Lecture 9 (Differential Flatness)

---

## Problem

The dynamics of a wheeled robot (car) are approximated by:

$$\dot{x} = u_1 \cos\theta \qquad (1)$$

$$\dot{y} = u_1 \sin\theta \qquad (2)$$

$$\dot{\theta} = u_1 \tan(u_2) \qquad (3)$$

Here $(x, y)$ is the car's position, θ its orientation, $u_1$ the speed input, $u_2$ the steering input.

**Show** that this system is differentially flat with flat outputs:

$$\bar{z} = \begin{bmatrix} x \\ y \end{bmatrix}$$

**Sanity check before starting** (counting theorem from Lec 9): # flat outputs = # control inputs. Inputs: $u_1, u_2$ → 2. Proposed flat outputs: $(x, y)$ → 2. ✓

**What must be shown:** every state ($x, y, \theta$) and every input ($u_1, u_2$) can be written as a function of $x, y$ and finitely many of their time derivatives.

---

## Proof

### Step 0 — Freebies

$x, y$ are the flat outputs themselves. Remaining: $\theta$, $u_1$, $u_2$.

### Step 1 — Extract θ (division trick)

Equations (1) and (2) tangle the unknown $u_1$ with θ — exactly like $(F_1+F_2)$ and θ in the planar quadrotor. **Divide to eliminate the input:**

$$\frac{\dot{y}}{\dot{x}} = \frac{u_1 \sin\theta}{u_1 \cos\theta} = \tan\theta$$

$$\Rightarrow \quad \boxed{\theta = \arctan\left(\frac{\dot{y}}{\dot{x}}\right)} \qquad (4)$$

θ is recovered from **first** derivatives of the flat outputs. (Compare: the quadrotor needed *second* derivatives, since its dynamics are second-order.)

### Step 2 — Extract u₁ (square and add)

The same pair (1), (2), attacked the complementary way — square both and add:

$$\dot{x}^2 + \dot{y}^2 = u_1^2\cos^2\theta + u_1^2\sin^2\theta = u_1^2$$

$$\Rightarrow \quad \boxed{u_1 = \sqrt{\dot{x}^2 + \dot{y}^2}} \qquad (5)$$

Physical meaning: $u_1$ is the speed, and speed is the magnitude of the velocity vector $(\dot x, \dot y)$ — of course it is recoverable from the path.

> **Two complementary moves on the same equation pair:** dividing (1),(2) kills $u_1$ and isolates θ; squaring-and-adding kills θ and isolates $u_1$. Together they crack both unknowns of the pair.

### Step 3 — Differentiate (4) to get θ̇

Equation (3) involves $\dot\theta$, so differentiate (4) in time. With $\frac{d}{dt}\arctan(w) = \frac{\dot w}{1+w^2}$ and $w = \dot y/\dot x$:

$$\frac{d}{dt}\left(\frac{\dot{y}}{\dot{x}}\right) = \frac{\ddot{y}\dot{x} - \dot{y}\ddot{x}}{\dot{x}^2}, \qquad 1 + \frac{\dot{y}^2}{\dot{x}^2} = \frac{\dot{x}^2 + \dot{y}^2}{\dot{x}^2}$$

$$\Rightarrow \quad \dot{\theta} = \frac{\ddot{y}\dot{x} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2} \qquad (6)$$

Second derivatives of the flat outputs have now entered.

### Step 4 — Solve (3) for u₂

From (3): $\tan u_2 = \dot\theta / u_1$. Substituting (5) and (6):

$$\tan u_2 = \frac{\ddot{y}\dot{x} - \dot{y}\ddot{x}}{\dot{x}^2 + \dot{y}^2} \cdot \frac{1}{\sqrt{\dot{x}^2 + \dot{y}^2}}$$

$$\Rightarrow \quad \boxed{u_2 = \arctan\left(\frac{\ddot{y}\dot{x} - \dot{y}\ddot{x}}{(\dot{x}^2 + \dot{y}^2)^{3/2}}\right)} \qquad (7)$$

> **💡 Physical meaning:** the quantity inside the arctan is exactly the **signed curvature** κ of the planar path traced by $(x(t), y(t))$. So (7) says: *the steering angle encodes the curvature of the path* — steer harder, curve tighter. The flatness dictionary is just this geometric fact written as a formula.

### Conclusion

$$\theta = \arctan\left(\frac{\dot{y}}{\dot{x}}\right), \qquad u_1 = \sqrt{\dot{x}^2+\dot{y}^2}, \qquad u_2 = \arctan\left(\frac{\ddot{y}\dot{x}-\dot{y}\ddot{x}}{(\dot{x}^2+\dot{y}^2)^{3/2}}\right)$$

Every state and input is a function of $\bar z = (x,y)$ and its derivatives up to **second order** (q = 2). Therefore the wheeled robot is **differentially flat** with flat outputs $(x, y)$. ∎

---

## Remarks

### ⚠️ Singularity at zero speed

The recovery formulas break when $\dot{x} = \dot{y} = 0$ (i.e., $u_1 = 0$): θ = arctan(0/0) is undefined. Physically obvious — **a parked car's path is a single point, and a point does not reveal which way the car faces.** Flatness recovery needs the car to be moving. (Also, arctan alone gives θ only up to π; in practice use atan2(ẏ, ẋ) to resolve forward vs reverse.)

### Comparison with the planar quadrotor (Lec 9 example)

| Item | Planar quadrotor | Wheeled robot (car) |
|------|------------------|---------------------|
| Inputs | 2 ($F_1, F_2$) | 2 ($u_1, u_2$) |
| States | 6 — `[x, y, θ, ẋ, ẏ, θ̇]` (2nd-order dynamics) | 3 — `[x, y, θ]` (1st-order kinematics) |
| Flat outputs | $(x, y)$ | $(x, y)$ |
| θ recovery | $\arctan(-\ddot{x}/(\ddot{y}+g))$ — from **acceleration** | $\arctan(\dot{y}/\dot{x})$ — from **velocity** |
| Key trick | divide (1)/(2) to kill $(F_1+F_2)$ | divide (2)/(1) to kill $u_1$ |
| Highest derivative q | 4 (inputs need 4th derivatives → "minimum snap") | 2 (inputs need 2nd derivatives = curvature) |

Same proof skeleton in both: **divide a tangled equation pair to extract θ → differentiate → back out the inputs.** The car is the gentler sibling: first-order dynamics push everything down two derivative orders.

### Why q matters (Lec 9 callback)

q = 2 means the flat-output trajectory $z(t)$ must be **twice differentiable** for the inputs to be well-defined — the "sufficiently smooth" requirement, made concrete. Planning for the car in flat-output space therefore only requires C² curves, versus C⁴ for the quadrotor.

---

## Summary

| Step | Move | Equations used | Result | New derivative order reached |
|------|------|-----------------|--------|-------------------------------|
| 0 | Counting check | # inputs = 2, proposed # flat outputs = 2 | plausible before starting | — |
| 1 | **Divide** (2) by (1) → cancels $u_1$ | (1), (2) | $\theta = \arctan(\dot y/\dot x)$ | 1st (ẋ, ẏ) |
| 2 | **Square & add** (1), (2) → cancels θ | (1), (2) | $u_1 = \sqrt{\dot x^2+\dot y^2}$ | 1st (ẋ, ẏ) |
| 3 | **Differentiate** θ (Step 1) in time | quotient rule on arctan | $\dot\theta = \dfrac{\ddot y\dot x-\dot y\ddot x}{\dot x^2+\dot y^2}$ | 2nd (ẍ, ÿ) |
| 4 | **Solve** (3) for $u_2$ using $u_1$ (Step 2) and θ̇ (Step 3) | (3) | $u_2 = \arctan\!\left(\dfrac{\ddot y\dot x-\dot y\ddot x}{(\dot x^2+\dot y^2)^{3/2}}\right)$ | 2nd (ẍ, ÿ) |
| — | **Conclusion** | all of θ, $u_1$, $u_2$ expressed via $x,y$ and derivatives up to order 2 | **system is differentially flat**, $q=2$ | — |

**Two complementary moves on one equation pair:** dividing (1),(2) kills $u_1$ and isolates θ; squaring-and-adding kills θ and isolates $u_1$ — together they crack both unknowns. This is the same *divide → extract θ → differentiate → solve-for-inputs* skeleton used for the planar quadrotor, just terminating two derivative orders earlier (q=2 vs q=4) because the car's dynamics are first-order.

**Two side notes worth keeping:** the recovery is singular at $u_1=0$ (a parked car's path is a single point and cannot reveal heading), and $u_2$'s expression is exactly the **signed curvature** of the path — steering angle literally encodes how sharply the path bends.
