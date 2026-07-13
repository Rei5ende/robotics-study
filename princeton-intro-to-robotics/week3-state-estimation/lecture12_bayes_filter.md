# Lecture 12: Bayes Filter

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

This lecture turns the set-based **non-deterministic filter** of Lec 11 into its probabilistic cousin, the **Bayes filter**. Instead of tracking a *set* of possible states, the robot now tracks a **belief** $\text{bel}(\bar x_t)$ — a full probability distribution over the state at time $t$. The three-step predict–correct–fuse skeleton is identical; only the representation (set → distribution) and the fusion rule (intersection → Bayes' rule) change.

> **One-line summary:** Keep the robot's belief as a probability distribution, and each step **predict** it forward with the dynamics (a weighted sum over previous states) and **correct** it with the measurement via Bayes' rule (multiply by the sensor likelihood, then normalize).

> **Where this sits:** the probabilistic upgrade of the Lec 11 filter. It answers Lec 11's loose end — "a *set* estimate can't drive a controller" — by producing a distribution whose mean can feed $\bar u = K\bar x$. Its own loose end (sums become intractable integrals for continuous state) motivates the approximations of Lec 13 (Kalman / particle filters).

---

## Motivation — Why Move From Sets to Beliefs

The non-deterministic filter returned a **set** $\hat X_t$ of possible states. That raises a practical problem: suppose we want an LQR controller $\bar u = K\bar x$. It needs a single state $\bar x$, but the filter hands us a whole set. Which element do we plug in? The set treats every state in it as equally possible and gives no way to choose.

The fix is to carry a **belief** instead — a probability distribution $\text{bel}(\bar x_t)$ over the state. Rather than "somewhere in this set," the robot now says "this state with probability 0.7, that one with 0.3." A distribution has a mean (or a mode), so it plugs straight into the controller, and it encodes *which* states are more likely — information the set never had.

> **💡 Set → distribution is the whole idea.** Everywhere Lec 11 kept a set, Lec 12 keeps a probability mass over states. The filter's logic is unchanged; it is re-expressed in the language of probability.

---

## Probability Toolkit

A compact review of the pieces the filter needs (see Ch. 2.2 of *Probabilistic Robotics*).

- **Normalization:** $\sum_x p(x) = 1$ (discrete), $\int p(x)\,dx = 1$ (continuous).
- **Joint:** $p(x, y) = p(X = x \text{ and } Y = y)$.
- **Independence:** $p(x, y) = p(x)\,p(y)$ iff $X, Y$ independent.
- **Conditional:** $p(x \mid y) = \dfrac{p(x, y)}{p(y)}$ for $p(y) > 0$.
- **Total probability:** $p(x) = \sum_y p(x \mid y)\,p(y)$ — the **marginal**; "sum the joint over the other variable."

These lead directly to the one result the whole module runs on.

### Bayes' Rule — the engine

$$p(x \mid y) = \frac{p(y \mid x)\*p(x)}{p(y)} = \frac{p(y \mid x)p(x)}{\sum_{x'} p(y \mid x')\*p(x')}$$

It follows in one line from splitting the joint two ways — $p(x \mid y)\,p(y) = p(x, y) = p(y \mid x)\,p(x)$ — and dividing by $p(y)$. The four pieces:

$$\underbrace{p(x \mid y)}_{\text{posterior}} \\propto\ \underbrace{p(y \mid x)}_{\text{likelihood}}\\underbrace{p(x)}_{\text{prior}}, \qquad \underbrace{p(y)}_{\text{normalizer}} = \sum_{x'} p(y \mid x')\*p(x')$$

The reading is **posterior ∝ likelihood × prior**: start from the prior belief, multiply by how well each state explains the evidence, and renormalize. The denominator $p(y)$ does not depend on $x$ — it is just the number that makes the posterior sum to 1.

> **💡 Bayes is the probabilistic $h^{-1}$.** Our *model* runs forward, state → observation: $p(z \mid x)$ (like the sensor function $h$). What we *want* is the reverse, observation → state: $p(x \mid z)$ (like the preimage $h^{-1}$). Bayes' rule performs exactly that inversion — the probabilistic version of "h-reverse."

---

## The Two-Step Bayes Filter

Each time step runs the same two updates as Lec 11, now on distributions. The standard notation marks the halfway point with a **bar**:

- $\overline{\text{bel}}(x_t)$ — belief **after the dynamics prediction but before the measurement** (the *prior* for this step).
- $\text{bel}(x_t)$ — belief **after folding in the measurement** (the *posterior*).

$$\text{bel}(x_{t-1}) \\xrightarrow{\\text{1. dynamics}\}\ \overline{\text{bel}}(x_t) \\xrightarrow{\\text{2. measurement}\}\ \text{bel}(x_t)$$

**1. Dynamics update (prediction)** — push the previous belief through the transition model. This is the theorem of total probability with $x_{t-1}$ marginalized out:

$$\overline{\text{bel}}(x_t) = \sum_{x_{t-1}} p(x_t \mid x_{t-1}, u_{t-1})\,\text{bel}(x_{t-1})$$

**2. Measurement update (correction)** — apply Bayes' rule with $\overline{\text{bel}}(x_t)$ as the prior:

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\,\overline{\text{bel}}(x_t), \qquad \eta = \frac{1}{p(z_t)}$$

> **⚠️ The bar is not decoration — it separates two beliefs in the same step.** $\overline{\text{bel}}(x_t)$ (prediction) is what goes into the *prior slot* of the measurement update; $\text{bel}(x_t)$ (correction) is what comes out. Using one symbol for both is what makes the raw lecture notes read as if the belief "changes twice." The dynamics update always outputs $\overline{\text{bel}}$; the measurement update always outputs $\text{bel}$ — regardless of which state you are looking at.

> **💡 Predict broadens, correct narrows.** The dynamics update spreads the belief out (motion adds uncertainty); the measurement update sharpens it (an observation rules states out). Same broaden-then-narrow rhythm as the set filter — intersection has simply become "multiply by the likelihood and renormalize."

---

## Worked Example — The Door

A robot's sensor reports whether a door is open or closed, but is imperfect:

| condition | $p(\text{sense-open}\mid\cdot)$ | $p(\text{sense-closed}\mid\cdot)$ |
|---|---|---|
| door $\text{open}$ | 0.6 | 0.4 |
| door $\text{closed}$ | 0.2 | 0.8 |

So the sensor is more trustworthy when the door is closed (0.2 chance of error) than when open (0.4 chance of error).

### t = 1 — measurement only

At the very first step there is no previous belief to propagate, so the dynamics update is skipped and only the measurement update runs. The initial "prior belief" *is* the prediction here: $\overline{\text{bel}}(X_1 = \text{open}) = 0.5$ and $\overline{\text{bel}}(X_1 = \text{closed}) = 0.5$. The robot senses **open**:

$$\text{bel}(X_1 = \text{open}) = \frac{0.6 \times 0.5}{0.6 \times 0.5 + 0.2 \times 0.5} = \frac{0.3}{0.4} = 0.75$$

so $\text{bel}(X_1 = \text{closed}) = 0.25$. The reading pushed the belief from 0.5 up to 0.75 — but not to 1.0, because the sensor is imperfect.

> **💡 $t=1$ is a half step, and the "prior" is $\overline{\text{bel}}$.** With no past to propagate, the given initial distribution plays the role of $\overline{\text{bel}}(X_1)$ directly. Defining $\overline{\text{bel}}$ as "the belief just before the measurement" unifies $t=1$ (prediction = initial guess) with $t \ge 2$ (prediction = dynamics output) into one structure.

### t = 2 — full step (dynamics, then measurement)

Now the robot **pushes** the door. The transition model:

| from → to | $p(\cdot \to \text{open})$ | $p(\cdot \to \text{closed})$ |
|---|---|---|
| $\text{open}$, $\text{push}$ | 1 | 0 |
| $\text{closed}$, $\text{push}$ | 0.8 | 0.2 |

**Dynamics update** (propagate $t=1$ belief through the push):

$$\overline{\text{bel}}(X_2 = \text{open}) = 1 \times 0.75 + 0.8 \times 0.25 = 0.95$$

so $\overline{\text{bel}}(X_2 = \text{closed}) = 0.05$. Pushing raised the "open" belief from 0.75 to 0.95, *before any new measurement*.

**Measurement update** (robot again senses **open**, using $\overline{\text{bel}}$ as the prior):

$$\text{bel}(X_2 = \text{open}) = \frac{0.6 \times 0.95}{0.6 \times 0.95 + 0.2 \times 0.05} = \frac{0.57}{0.58} \approx 0.983$$

so $\text{bel}(X_2 = \text{closed}) \approx 0.017$. Combining the action and the second observation drives the confidence to about 98.3%.

### The four values, sorted by stage

| value | quantity | stage |
|---|---|---|
| 0.95 | $\overline{\text{bel}}(X_2 = \text{open})$ | dynamics (before measurement) |
| 0.05 | $\overline{\text{bel}}(X_2 = \text{closed})$ | dynamics (before measurement) |
| 0.983 | $\text{bel}(X_2 = \text{open})$ | measurement (final) |
| 0.017 | $\text{bel}(X_2 = \text{closed})$ | measurement (final) |

> **⚠️ Both dynamics-stage values carry a bar.** In the raw lecture notes the $t=2$ prediction values (0.95, 0.05) are written as plain $\text{bel}$, but they are $\overline{\text{bel}}$: the accompanying sentence says "belief at $t=2$ *before making a sensor measurement*," and the 0.95 is reused as the *prior* in the very next Bayes step — both are decisive signs it is $\overline{\text{bel}}$. (The notes also contain a typo, "from $t=1$ ti the text time-step," which should read "to the next time-step.")

---

## Key Assumptions & Practical Notes

- **Markov assumption (the filter rests on it).** Writing the models as $p(x_t \mid x_{t-1}, u_{t-1})$ and $p(z_t \mid x_t)$ assumes the next state depends only on the *immediately previous* state and input, and the measurement only on the *current* state — history beyond that is irrelevant. This is why a single belief suffices and the filter can run **recursively** rather than storing the whole past.

- **The likelihood ratio sets the update strength.** In the measurement update, what moves the belief is the *ratio* of likelihoods across states. At $t=1$ the ratio was $0.6 : 0.2 = 3 : 1$, turning a $1:1$ prior into a $3:1$ posterior (0.75 : 0.25). If a sensor gave equal likelihoods for both states, the measurement would not change the belief at all — it cannot distinguish them.

- **Normalization trick — you never compute $p(z_t)$ directly.** Since $p(z_t)$ does not depend on the state, in practice: compute the numerator (likelihood × $\overline{\text{bel}}$) for every state, then divide by their sum. That is exactly the $0.57 / (0.57 + 0.01)$ step.

- **Continuous state → integrals.** For continuous state spaces the sums become integrals, which is what makes the exact Bayes filter hard to implement and motivates the approximations in Lec 13.

- **Two models are required.** A dynamics model $p(\bar x_t \mid \bar x_{t-1}, \bar u_{t-1})$ and a measurement model $p(\bar z_t \mid \bar x_t)$. A worthwhile exercise: think about how you would actually obtain each in practice.

- **Book convention differs by one index.** *Probabilistic Robotics* writes the dynamics as $p(X_t \mid X_{t-1}, U_t)$; this course uses $U_{t-1}$. Read the book's $U_t$ as our $U_{t-1}$.

---

## Concluding Notes

The Bayes filter is the probabilistic version of the non-deterministic filter, and it resolves Lec 11's practical complaints: it is less conservative (it ranks states by probability instead of only bounding them), and its distribution output feeds a controller cleanly. What remains hard is computation — the exact filter requires sums or integrals over the entire state space every step.

> **💡 Bayes' rule is the correction step, relabeled.** The measurement update is literally $\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\,\overline{\text{bel}}(x_t)$: sensor model = likelihood, predicted belief = prior, corrected belief = posterior. The intersection that fused information in the set filter has become "multiply and normalize."

**Next lecture:** making the Bayes filter tractable — Kalman filters (exact, for linear-Gaussian systems) and particle filters (sampling-based approximation).

---

## Lec 11 ↔ Lec 12 Correspondence

| | Lec 11 — non-deterministic (set) | Lec 12 — Bayes (probability) |
|---|---|---|
| belief | set $\hat X_t$ | distribution $\text{bel}(\bar x_t)$ |
| prediction | $f(\hat X_{t-1})$ (push the set forward) | $\sum_{x_{t-1}} p(x_t \mid x_{t-1}, u_{t-1})\,\text{bel}(x_{t-1})$ |
| measurement info | preimage $h^{-1}(\bar z_t)$ | likelihood $p(z_t \mid x_t)$ |
| fusion | intersection $\cap$ | multiply by likelihood, then normalize |
| "both must hold" | AND → intersection | joint → product |
| output for control | a set (which element?) | a distribution (take the mean) |
| limitation | worst-case, no ranking | exact sums/integrals are costly |

---

## Concept Flow & What's Next

```
Lec 11: Non-deterministic filter — belief = a SET
   predict  f(X̂ₜ₋₁)   →   correct  ∩ h⁻¹(z̄ₜ)          (worst-case, no probabilities)
        ↓   (a set estimate can't feed a controller — which element to pick?)
Lec 12: Bayes filter — belief = a probability DISTRIBUTION  bel(xₜ)
   predict  Σ p(xₜ|xₜ₋₁,uₜ₋₁) bel(xₜ₋₁)   →   correct  η · p(zₜ|xₜ) · bel‾(xₜ)
        ↓   (sums become integrals for continuous state — hard in general)
Lec 13: Kalman / particle filters — tractable implementations & approximations
Lec 14-16: Localization → Mapping → SLAM  (estimate state AND map at once)
```

> **💡 Presentation one-liner:** *"The Bayes filter is the non-deterministic filter with probabilities bolted on: sets become distributions, and set intersection becomes Bayes' rule — predict with the dynamics, correct with the measurement, repeat."*

---

## Quick Reference

| Term | Meaning |
|------|---------|
| belief $\text{bel}(\bar x_t)$ | probability distribution over the state at time $t$ (replaces the set $\hat X_t$) |
| $\overline{\text{bel}}(x_t)$ | belief after the dynamics prediction, **before** the measurement (the prior for the step) |
| $\text{bel}(x_t)$ | belief after the measurement correction (the posterior) |
| Bayes' rule | $p(x\mid y) = p(y\mid x)\,p(x)/p(y)$; inverts a conditional (forward model → reverse inference) |
| posterior ∝ likelihood × prior | the core shape of every measurement update |
| likelihood $p(z_t\mid x_t)$ | sensor model; how well state $x_t$ explains the reading $z_t$ |
| prior | the belief before the current evidence ($\overline{\text{bel}}$ in the filter) |
| normalizer $p(z_t)$ | $\sum_{x'} p(z_t\mid x')\,\overline{\text{bel}}(x')$; makes the posterior sum to 1 (never computed on its own) |
| dynamics update | $\overline{\text{bel}}(x_t) = \sum_{x_{t-1}} p(x_t\mid x_{t-1},u_{t-1})\,\text{bel}(x_{t-1})$; total probability (prediction) |
| measurement update | $\text{bel}(x_t) = \eta\, p(z_t\mid x_t)\,\overline{\text{bel}}(x_t)$; Bayes' rule (correction) |
| Markov assumption | next state depends only on previous state+input; measurement only on current state — enables recursion |
| likelihood ratio | ratio of likelihoods across states sets how far a measurement moves the belief |
| $t=1$ special case | no dynamics step; the initial prior acts as $\overline{\text{bel}}(X_1)$, then one measurement update |
| continuous state | sums become integrals → exact filter costly → motivates Lec 13 |
| book convention | *Probabilistic Robotics* uses $U_t$ where this course uses $U_{t-1}$ |
