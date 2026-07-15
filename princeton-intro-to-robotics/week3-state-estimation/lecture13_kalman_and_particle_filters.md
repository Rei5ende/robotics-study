# Lecture 13: Kalman Filters and Particle Filters

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lec 12 gave us the Bayes filter — fully general, but with a fatal practical flaw: for continuous state spaces the sums become **integrals**, and the belief is an arbitrary function over an infinite state space. No computer can carry that around. This lecture gives **two ways to actually run the filter**, and both are really answers to a single question: *what do we represent the belief with?*

**Kalman filter** restricts the belief to a **Gaussian**, which is pinned down by just $(\mu, \Sigma)$ — exact and fast, but only under strict assumptions. **Particle filter** approximates the belief with a **cloud of samples** — works almost anywhere, including on beliefs a Gaussian could never express.

> **One-line summary:** Keep the Bayes filter's predict–correct skeleton exactly as it is, and swap the belief's representation — a Gaussian $(\mu, \Sigma)$ updated by matrix algebra (Kalman), or a set of $M$ particles reweighted and resampled (particle filter).

> **Where this sits:** the practical payoff of the estimation module. Lec 11 gave sets, Lec 12 gave distributions, Lec 13 gives the two representations people actually implement. Both inherit the same two-step rhythm; neither is a new algorithm.

---

## Part 1 — The Kalman Filter

### The three assumptions

The Kalman filter is not a new filter — it is the **Bayes filter instantiated for linear-Gaussian systems**. Three assumptions are required together.

**1. Linear dynamics with Gaussian noise**

$$\bar{x}_t = A_t \bar{x}_{t-1} + B_t \bar{u}_{t-1} + \bar{\varepsilon}_t, \qquad \bar{\varepsilon}_t \sim \mathcal{N}(0, R_t)$$

This is nearly the LQR setup, but in discrete time and with uncertainty injected through $\bar{\varepsilon}_t$. That noise term is exactly what turns a deterministic model into the probabilistic dynamics model $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1})$ the Bayes filter needs.

**2. Linear measurement with Gaussian noise**

$$\bar{z}_t = C_t \bar{x}_t + \bar{\delta}_t, \qquad \bar{\delta}_t \sim \mathcal{N}(0, Q_t)$$

**3. Gaussian initial belief:** $\text{bel}(\bar{x}_0) = \mathcal{N}(\mu_0, \Sigma_0)$.

Together these guarantee that $\text{bel}(\bar{x}_t)$ **stays Gaussian at every time step**. That is the whole reason linear-Gaussian systems are so nice: a Gaussian is fully determined by a mean and a covariance, so instead of propagating a function over an infinite state space we only update a few numbers.

> **💡 Why the belief never escapes the Gaussian family.** Gaussians are **closed** under the two operations the filter performs: pushing a Gaussian through a linear map yields a Gaussian (the dynamics update), and multiplying two Gaussians yields a Gaussian (the Bayes correction). So under linear dynamics, linear measurements, and a Gaussian start, the belief has nowhere else to go. The formal proof is heavy (Ch. 3.2.4) and the lecture explicitly does not require it.

> **⚠️ What the lecture does require.** Three things only: (1) the assumptions on dynamics, sensor, and initial belief; (2) that this is just the Bayes filter under those assumptions; (3) that it is easy to implement in Python — it is only matrix multiplications.

### What $C_t$ means

$C_t$ is the **linear version of the sensor function $h$ from Lec 11**: it translates a state into the measurement that state would produce. The product $C_t \bar{x}_t$ is the *expected reading* at state $\bar{x}_t$.

It matters because the robot's state and its sensor readings live in **different spaces**. If the state is (position, velocity) but the sensor reads position only, then

```
x̄ = [ position ]        C = [ 1  0 ]        C x̄ = position
    [ velocity ]
```

so $C$ is the instruction "pull out the position component and ignore velocity." Without $C$ we could not even compare a predicted state to an actual reading — they are not the same kind of object.

> **💡 $A$ versus $C$.** $A$ connects state to *next state* and does its work in the dynamics update. $C$ connects state to *measurement* and does its work in the measurement update. One moves through time; the other translates into sensor units.

### Dimensions

Let the state be $n$-dimensional and the measurement $m$-dimensional. Every term's shape follows from making $\bar{z}_t = C_t \bar{x}_t + \bar{\delta}_t$ dimensionally consistent: $\bar{\delta}_t$ is added to $\bar{z}_t$, so it must also be $m \times 1$, and a covariance matrix is always "vector dimension × vector dimension."

| symbol | meaning | shape |
|---|---|---|
| $\bar{x}_t$ | state | $n \times 1$ |
| $\bar{z}_t$ | measurement | $m \times 1$ |
| $A_t$ | state transition | $n \times n$ |
| $B_t$ | input matrix | $n \times \ell$ ($\ell$ inputs) |
| $C_t$ | measurement matrix | $m \times n$ |
| $\bar{\varepsilon}_t$ | dynamics noise | $n \times 1$ |
| $R_t$ | dynamics noise covariance | $n \times n$ |
| $\bar{\delta}_t$ | measurement noise | $m \times 1$ |
| $Q_t$ | measurement noise covariance | $m \times m$ |

> **⚠️ $C_t$ is generally not square.** The number of measurements $m$ need not equal the number of states $n$ — a position-only sensor on a (position, velocity) state gives $m = 1$, $n = 2$, so $C_t$ is $1 \times 2$, $Q_t$ is $1 \times 1$, and $R_t$ is $2 \times 2$. Each noise covariance follows *its own* vector's dimension, not the other's.

### Covariance — what $\Sigma$ actually encodes

Variance measures how far one quantity spreads around its mean; covariance $\text{Cov}(X, Y) = \mathbb{E}[(X - \mu_X)(Y - \mu_Y)]$ measures how two quantities move **together** (positive = same direction, negative = opposite, zero = unrelated). For a vector, all of this packs into one square matrix whose $(i,j)$ entry is $\text{Cov}(x_i, x_j)$ — diagonal entries are the per-component variances, off-diagonals the couplings.

Geometrically, a 2D Gaussian belief seen from above is an **ellipse**: its *size* is the overall uncertainty and its *tilt* comes from the off-diagonal covariance. A large covariance produces a long thin ellipse — "I know I'm somewhere along this diagonal, but not where along it."

> **💡 The filter in one picture.** Predict inflates the ellipse (motion adds uncertainty); correct flattens it along the direction the sensor observed. The Kalman gain decides how the predicted ellipse and the measurement ellipse combine into a smaller one.

> **⚠️ $\Sigma$ is not sensor noise.** $Q_t$ and $R_t$ are fixed properties of the sensor and the dynamics — they are *inputs*. $\Sigma_t$ is the robot's current confidence in its own estimate, produced by running those inputs through the filter — it is an *output*, and it changes every step.

### The algorithm

Primes mark the **prediction**, i.e. the parameters of $\overline{\text{bel}}$ from Lec 12; unprimed symbols are the corrected belief $\text{bel}$.

**1. Dynamics update** — computes $\overline{\text{bel}}(\bar{x}_t) = \mathcal{N}(\mu'_t, \Sigma'_t)$:

$$\mu'_t = A_t \mu_{t-1} + B_t \bar{u}_{t-1}$$

$$\Sigma'_t = A_t \Sigma_{t-1} A_t^T + R_t$$

The mean is simply pushed through the linear dynamics. The covariance **grows**: $A_t \Sigma_{t-1} A_t^T$ propagates the existing uncertainty and $R_t$ adds the fresh process noise.

**2. Measurement update** — computes $\text{bel}(\bar{x}_t) = \mathcal{N}(\mu_t, \Sigma_t)$:

$$K_t = \Sigma'_t C_t^T \left(C_t \Sigma'_t C_t^T + Q_t\right)^{-1}$$

$$\mu_t = \mu'_t + K_t\left(\bar{z}_t - C_t \mu'_t\right)$$

$$\Sigma_t = (I - K_t C_t)\,\Sigma'_t$$

The covariance **shrinks**. $K_t$ is the **Kalman gain**.

> **⚠️ Index typo in the lecture notes.** Step (1) of the printed algorithm writes $B_t \bar{u}_t$, but the dynamics model consumes $\bar{u}_{t-1}$ — the input that *drove* the transition into time $t$. Read it as $B_t \bar{u}_{t-1}$, consistent with this course's convention (the same $U_t$ versus $U_{t-1}$ mismatch flagged in Lec 12 against *Probabilistic Robotics*).

### The Kalman gain — the heart of it

Drop to one dimension with $C = 1$ and the gain's identity is exposed:

$$K = \frac{\sigma'^2}{\sigma'^2 + q} = \frac{\text{prediction variance}}{\text{prediction variance} + \text{measurement variance}}$$

This always lies between 0 and 1: it is the fraction of the total uncertainty owned by the prediction, i.e. **how much to trust the measurement over the prediction**.

- Measurement is precise ($q$ small) → $K \to 1$ → $\mu_t \approx \bar{z}_t$: believe the sensor.
- Measurement is noisy ($q$ large) → $K \to 0$ → $\mu_t \approx \mu'_t$: ignore it, keep the prediction.
- Prediction is uncertain ($\sigma'^2$ large) → $K \to 1$: my own guess is poor, so lean on the sensor.

The updated mean $\mu_t = \mu'_t + K_t(\bar{z}_t - C_t\mu'_t)$ is therefore a **weighted average of prediction and measurement**, with $K_t$ setting the weights.

> **💡 The gain is Lec 12's prior-versus-likelihood tug-of-war, quantified.** In Assignment 6 a strong prior kept the evidence from moving the belief; here a large $Q$ makes $K$ small so the measurement moves the belief less. Identical principle — the Gaussian case just gives an explicit formula for the balance.

> **⚠️ Innovation: $\bar{z}_t - C_t \mu'_t$ is the "surprise."** $C_t \mu'_t$ is the reading we *expected* given our predicted state; subtracting it from the actual reading measures how much reality disagreed. The filter moves the mean by the gain **times the surprise** — if the sensor reports exactly what was predicted, the mean does not move at all.

### $(I - K_tC_t)$ — the shrink factor

In one dimension the covariance update is just $\Sigma_t = (1 - K)\Sigma'_t$. Since $K \in [0, 1]$, the factor $(1-K)$ is a **fraction between 0 and 1**: it says what share of the predicted uncertainty *survives* the measurement.

- $K \to 1$ (trust the sensor) → $1 - K \to 0$ → uncertainty almost entirely removed.
- $K \to 0$ (ignore the sensor) → $1 - K \to 1$ → nothing is removed.

Because $(1-K) < 1$ always, **a measurement can never increase uncertainty** — at worst a useless reading leaves it unchanged. That is exactly right: receiving information cannot make you more confused.

In matrix form, $K_tC_t$ describes *which directions of the state space the measurement actually informs*. So $(I - K_tC_t)$ reads as: start from "change nothing" ($I$) and subtract the uncertainty the measurement genuinely resolved ($K_tC_t$). Directions the sensor cannot see are preserved by the $I$.

### The picture

The lecture's sketch is three frames of one Gaussian:

1. **$\text{bel}(x_t)$** — the belief from last step: center $\mu$, width $\Sigma$.
2. **Dynamics update** — the curve **shifts** (mean moves with the dynamics) and **widens** ($R_t$ piles on). This is $\overline{\text{bel}}(x_t)$.
3. **Measurement update** — the prediction and the measurement fuse into a belief that sits **between them** and is **narrower than either**.

> **💡 Two things to read off frame 3.** First, the corrected belief lands *between* prediction and measurement, pulled toward whichever is sharper — that pull is $K$. Second, it is **narrower than both**: fusing two independent pieces of evidence makes you more confident than either alone. Overlapping two blurry clues yields a sharper one.

> **⚠️ The corrected belief does not sit on top of the measurement.** The filter never simply believes the sensor; it blends prediction and measurement in proportion to their confidences. Predict inflates, correct deflates, and that breathing rhythm — identical to Lec 11 and Lec 12 — repeats forever.

### When the dynamics are not linear → EKF

If the dynamics or the measurement are nonlinear, the Kalman filter does not apply directly. The fix is to **approximate them locally with linear ones** (Jacobians) and then run the Kalman filter as usual. This is the **Extended Kalman Filter (EKF)**, and it works well in many settings.

---

## Part 2 — Particle Filters

### Why the EKF is not enough — the two-door problem

The EKF still represents the belief with a **Gaussian**, and a Gaussian has exactly **one peak**. Some beliefs simply are not shaped like that.

Suppose a robot in a corridor has a (probabilistic) door detector, and there are **two identical doors**. The sensor detects a door. What should the new belief look like? It must have a **peak at each door** — the robot cannot tell which one it is at. That belief is **bi-modal**, and no Gaussian can capture it. This is the motivating failure that particle filters solve.

### The main idea — represent the belief with samples

Instead of parameters like $(\mu, \Sigma)$, represent $\text{bel}(\bar{x}_t)$ by $M$ **samples ("particles")** drawn from it, where $M$ is typically large (1000, $10^4$, …):

$$S_t \triangleq \{\bar{x}_t^{[1]}, \bar{x}_t^{[2]}, \bar{x}_t^{[3]}, \ldots, \bar{x}_t^{[M]}\}$$

**Where particles are dense, probability is high; where they are sparse, it is low.** No functional form is imposed, so *any* shape — bimodal included — can be represented.

> **⚠️ $\bar{x}_t^{[m]}$ is a value, not a distribution.** It is one concrete state, a single point such as $(1.7, 0.3)$. In $\bar{x}_t^{[m]} \sim p(\cdot)$ the distribution is on the **right** of the tilde; the particle is what gets *drawn out of it*. The die is $p(\cdot)$; the particle is the number that came up.

> **💡 The belief is never stored explicitly.** Kalman carries the distribution directly as $(\mu, \Sigma)$. A particle filter carries only points, and the distribution lives **implicitly in their density** — "two hundred particles landed here, so probability is high here."

### The algorithm

At each time step, for $m = 1 \ldots M$:

**1. Sample (dynamics update)** — push each particle one step through the dynamics:

$$\bar{x}_t^{[m]} \sim p(\bar{x}_t \mid \bar{x}_{t-1}^{[m]}, \bar{u}_{t-1})$$

**2. Compute the importance weight** — how well this particle explains the reading:

$$w_t^{[m]} = p(\bar{z}_t \mid \bar{x}_t^{[m]})$$

**3. Resample** — initialize $S_t$ as empty, then $M$ times: draw index $i$ with probability proportional to $w_t^{[i]}$ and add $\bar{x}_t^{[i]}$ to $S_t$. Sampling is **with replacement**, so duplicates are expected and fine; particles with low importance tend to be eliminated.

> **⚠️ The tilde and the equals sign are different operations.** The lecture slides call this out explicitly. $\bar{x}_t^{[m]} \sim p(\cdots)$ means **drawing a sample from** a distribution; $w_t^{[m]} = p(\bar{z}_t \mid \bar{x}_t^{[m]})$ means **evaluating the probability density** at a value. In code: `np.random.choice` and sampling routines versus `.pdf(...)`. Writing `.rvs()` where `.pdf()` belongs silently destroys the filter.

> **💡 Why resampling *is* the Bayes correction.** After the dynamics step the particles approximate $\overline{\text{bel}}(x_t)$. Weighting them by $p(z_t \mid x_t)$ and resampling in proportion makes the surviving density proportional to $\overline{\text{bel}}(x_t) \cdot p(z_t \mid x_t)$ — which is the posterior. "Multiply by the likelihood, then normalize" is executed **with samples**, and drawing exactly $M$ of them handles the normalization automatically.

> **⚠️ $S_t$ is a multiset, not a set.** It is written with set braces, yet duplicates are not only allowed but essential — **the multiplicity encodes the density**. A particle copied five times means high probability there. Contrast Lec 11's $\hat{X}_t$, a genuine set where duplication would have been meaningless because it only recorded possible-versus-impossible. Duplicates are the trace of moving from sets to distributions.

### Properties

Particle filters are **one of the most popular state estimation algorithms in practice**, particularly for **localization**. They are very simple to implement and work very well. The limitation: if the state space is very high-dimensional or the belief needs to be very complicated, **many particles** may be required for a good approximation.

### Seen in Lab 6

Lab 6 makes the bimodality concrete. The state is $\bar{x} = (x, y)$, the dynamics are known exactly, and the only sensor reports the *distance to the origin*, $p(z_t \mid \bar{x}_t) = \mathcal{N}(\|\bar{x}_t\|, \sigma_\text{meas}^2)$. Running the filter, the particles cluster in **two** locations.

The cause is visible right in the weight computation: the weight depends on the particle only through $\|\bar{x}^{[m]}\|$, so a particle at $(r, 0)$ and one at $(-r, 0)$ receive **identical weights** — a distance-only sensor has thrown the sign away. The dynamics sharpen this: $y_{t+1} = 0.8y_t$ collapses particles toward the x-axis while $x_{t+1} = 1.1x_t$ expands, leaving exactly the two symmetric candidates. The remedy is to break the symmetry — at least **three** non-collinear range measurements (two circles still intersect in two points), or a single sensor that supplies sign, position, or heading.

> **💡 This is Lec 11's opening example returning.** One distance leaves a circle; two leave two points; three pin it down. The same geometry that motivated the whole estimation module reappears as the failure mode of a working particle filter.

---

## Concluding Notes

Both algorithms leave the Bayes filter's logic untouched and change only how the belief is written down. The Kalman filter buys exactness and speed by **restricting the belief** to a Gaussian, at the cost of assumptions that many real systems violate — and of being fundamentally unimodal, even in EKF form. The particle filter buys generality by **approximating the belief** with samples, at the cost of needing enough particles.

| | Kalman filter | Particle filter |
|---|---|---|
| belief representation | Gaussian $(\mu, \Sigma)$ | $M$ samples $\{\bar{x}^{[m]}\}$ |
| exact or approximate | exact (under its assumptions) | approximate |
| assumptions | linear dynamics, linear sensor, Gaussian noise and prior | essentially none |
| multimodal beliefs | impossible (one peak) | natural |
| predict step | $\mu' = A\mu + B\bar{u}$, $\Sigma' = A\Sigma A^T + R$ | push each particle through the dynamics |
| correct step | gain $K$, innovation, $(I - KC)$ | weight by $p(z \mid x)$, resample |
| cost driver | matrix dimensions | number of particles $M$ |
| fails when | dynamics or sensor nonlinear; belief multimodal | state space high-dimensional |

> **💡 The module is a history of belief representations.** Lec 11 carried a **set** and fused by intersection. Lec 12 carried a **distribution** and fused by Bayes' rule. Lec 13 carries either a **Gaussian** (matrix updates) or a **sample cloud** (weight and resample). The predict–correct skeleton never changed. There is even a small symmetry in the ending: the particle filter's $S_t$ is a collection of states again — Lec 11's "set of possible states" returns as "a set of samples approximating a distribution."

**Next lectures:** localization, mapping, and finally SLAM — estimating the state and the map at the same time.

---

## Concept Flow & What's Next

```
Lec 11: Non-deterministic filter — belief = a SET, fuse by ∩
   ↓   (worst-case, no ranking of possibilities)
Lec 12: Bayes filter — belief = a DISTRIBUTION, fuse by Bayes' rule
   ↓   (continuous state ⇒ sums become integrals ⇒ intractable)
Lec 13: two ways to actually run it
   ├── Kalman filter — belief = GAUSSIAN (μ, Σ)
   │      linear dynamics + linear sensor + Gaussian noise ⇒ belief stays Gaussian
   │      predict: μ' = Aμ + Bū,  Σ' = AΣAᵀ + R      (Σ grows)
   │      correct: K, μ = μ' + K(z̄ − Cμ'),  Σ = (I − KC)Σ'   (Σ shrinks)
   │      nonlinear? → EKF (linearize, then run Kalman)
   │      but: always ONE peak — cannot express "either door"
   └── Particle filter — belief = M SAMPLES  S_t = {x̄⁽¹⁾, ..., x̄⁽ᴹ⁾}
          predict: sample each particle through the dynamics
          weight:  w⁽ᵐ⁾ = p(z̄ₜ | x̄⁽ᵐ⁾)          ← evaluate density (=)
          resample: draw ∝ w, with replacement   ← draw samples (~)
          any shape, bimodal included; needs many particles in high dimensions
        ↓
Lec 14-16: Localization → Mapping → SLAM
```

> **💡 Presentation one-liner:** *"Lec 13 does not invent a new filter — it answers 'what do we store the belief in?' twice: a Gaussian you update with matrices, or a cloud of particles you reweight and resample."*

---

## Quick Reference

| Term | Meaning |
|------|---------|
| linear-Gaussian system | linear dynamics + linear sensor + Gaussian noise + Gaussian prior; the setting where the belief stays Gaussian |
| Kalman filter | the Bayes filter instantiated for linear-Gaussian systems; updates only $(\mu, \Sigma)$ |
| closure of Gaussians | linear maps and products of Gaussians are Gaussian — why the belief never leaves the family |
| $A_t$ | state transition matrix ($n \times n$); acts in the dynamics update |
| $C_t$ | measurement matrix ($m \times n$); linear version of Lec 11's sensor function $h$; $C\bar{x}$ = expected reading |
| $R_t$ | dynamics noise covariance ($n \times n$); added every predict step, grows uncertainty |
| $Q_t$ | measurement noise covariance ($m \times m$); large $Q$ means a sensor worth less trust |
| $\Sigma_t$ | belief covariance — the robot's confidence; an *output* of the filter, unlike $Q$ and $R$ |
| covariance matrix | $(i,j)$ entry $= \text{Cov}(x_i, x_j)$; diagonal = variances, off-diagonal = coupling; geometrically the belief ellipse's size and tilt |
| $\mu'_t, \Sigma'_t$ | the prediction, i.e. $\overline{\text{bel}}$ from Lec 12 (primes = before the measurement) |
| Kalman gain $K_t$ | how much to trust the measurement over the prediction; in 1D, $\sigma'^2/(\sigma'^2 + q) \in [0,1]$ |
| innovation | $\bar{z}_t - C_t\mu'_t$, the surprise: actual reading minus expected reading; no surprise, no correction |
| $(I - K_tC_t)$ | the shrink factor; in 1D just $(1-K) \in [0,1]$, so a measurement never increases uncertainty |
| predict / correct | predict widens the Gaussian ($+R_t$); correct narrows it and pulls the mean toward the sharper source |
| EKF | linearize nonlinear dynamics or sensor locally, then run the Kalman filter |
| unimodality limit | Gaussians have one peak; the two-door problem needs two — the motivation for particle filters |
| particle filter | represent the belief with $M$ samples; density of particles *is* the distribution |
| $S_t$ | the particle set (really a **multiset** — duplicates encode density) |
| $\bar{x}_t^{[m]}$ | one particle: a concrete state value, **not** a distribution |
| importance weight $w_t^{[m]}$ | $p(\bar{z}_t \mid \bar{x}_t^{[m]})$, a density **value**; how well the particle explains the reading |
| $\sim$ versus $=$ | $\sim$ draws a sample from a distribution; $=$ evaluates a density at a point |
| resampling | draw $M$ indices with replacement $\propto w$; realizes "multiply by likelihood, then normalize" |
| particle filter limits | high-dimensional states or very complicated beliefs demand many particles |
| Lab 6 bimodality | a distance-only sensor gives equal weight to $(r,0)$ and $(-r,0)$; fix by breaking the symmetry |
