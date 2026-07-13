# Assignment 6: State Estimation — Theory Solutions

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Topics:** non-deterministic filter (Lec 11), Bayes filter (Lec 12)

---

## Overview

Three theory problems, each exercising the predict–correct filter on a discrete state space. Problem 1 is the **set-based** non-deterministic filter; Problems 2 and 3 are the **probabilistic** Bayes filter. Two of the three are really lessons about *failure modes and subtleties* rather than mechanical computation.

> **One-line each:** **P1** — a useless sensor still localizes the robot, because information comes from the *dynamics* (a wall), not the sensor. **P2** — a prior of exactly 0 or 1 permanently breaks the Bayes filter. **P3** — a 3-state HMM where the $1 - \text{other}$ shortcut fails and full normalization is required.

---

## Problem 1 — Non-deterministic Filter on a Corridor

**Setup.** Five cells, state space $\mathcal{X} = \{1,2,3,4,5\}$. Action **Left** moves $k \to k-1$ for interior cells, and cell 1 stays at 1 (left wall). The sensor **always reports "cell 3"** regardless of the true cell. The robot starts with no information, $\hat{X}_0 = \{1,2,3,4,5\}$, and applies **Left** every step.

**The sensor carries no information.** Since $h(k) = 3$ for every cell $k$, the preimage of the only possible reading is the *entire* state space:

$$h^{-1}(3) = \{\, \bar{x} \in \mathcal{X} \mid h(\bar{x}) = 3 \,\} = \{1,2,3,4,5\}$$

So the measurement update never removes anything: $\hat{X}_t \cap h^{-1}(3) = \hat{X}_t$. Only the **dynamics update** (Left) does any work.

> **⚠️ Common trap: $h^{-1}(3) \neq \{3\}$.** Reading "sensor says 3, so the state is 3" assumes a *working* sensor. Here every cell outputs 3, so *every* cell is consistent with the reading — the preimage is the whole set, and the measurement is inert.

**Applying Left to a set** sends $2\to1,\ 3\to2,\ 4\to3,\ 5\to4$, and $1\to1$. Iterating (measurement leaves the set unchanged each step):

| $t$ | $\hat{X}_t$ | note |
|---|---|---|
| 0 | $\{1,2,3,4,5\}$ | initial, no information |
| 1 | $\{1,2,3,4\}$ | cell 5 unreachable under Left |
| 2 | $\{1,2,3\}$ | |
| 3 | $\{1,2\}$ | |
| 4 | $\{1\}$ | **state uniquely determined** |
| 5 | $\{1\}$ | $1 \to 1$ at the wall, stays put |

$$\boxed{\hat{X}_4 = \hat{X}_5 = \{1\}}$$

**Why a useless sensor still localizes.** The set shrank entirely from the dynamics. The key is that Left is **non-injective at the wall**: both cell 1 and cell 2 map to cell 1, collapsing two possibilities into one. Repeated collapses funnel every start state to the left wall.

> **💡 Information came from the environment, not the sensor.** Walking left with eyes closed, you eventually *know* you are against the left wall — the wall erases where you started and pins your location. The information source is **action + boundary structure**, not observation.

> **💡 Contrast: a circular track would never converge.** If cell 1 wrapped to cell 5, Left would be a bijection (nothing ever merges), so $\{1,2,3,4,5\}$ would just rotate forever at size 5. The wall's *irreversibility* is exactly what shrinks the belief.

---

## Problem 2 — Bayes Filter With a Degenerate Prior

**Sensor model** (imperfect door sensor):

| actual state | $p(\text{sense-open}\mid\cdot)$ | $p(\text{sense-closed}\mid\cdot)$ |
|---|---|---|
| $\text{open}$ | 0.6 | 0.4 |
| $\text{closed}$ | 0.2 | 0.8 |

Unlike class, the prior is **degenerate**: $\overline{\text{bel}}(X_1 = \text{open}) = 1$, $\overline{\text{bel}}(X_1 = \text{closed}) = 0$. There are no control inputs, so each step is measurement-update only.

### (a) One measurement of "closed"

Bayes measurement update with the "closed" reading:

$$\text{bel}(X_1 = \text{closed}) = \frac{p(\text{sense-closed}\mid\text{closed})\,\overline{\text{bel}}(\text{closed})}{p(\text{sense-closed}\mid\text{closed})\,\overline{\text{bel}}(\text{closed}) + p(\text{sense-closed}\mid\text{open})\,\overline{\text{bel}}(\text{open})}$$

$$= \frac{0.8 \times 0}{0.8 \times 0 + 0.4 \times 1} = \frac{0}{0.4} = 0$$

$$\boxed{\text{bel}(X_1 = \text{closed}) = 0, \qquad \text{bel}(X_1 = \text{open}) = 1}$$

The robot sensed "closed" from a fairly reliable sensor, yet its belief did not move at all.

> **⚠️ A prior of 0 is a death sentence for a hypothesis.** Since $\text{posterior} \propto \text{likelihood} \times \text{prior}$, any state with prior 0 stays 0 no matter how strongly the evidence supports it: $\text{something} \times 0 = 0$.

### (b) Repeated measurements when the door is truly closed

With no control input, the next step's prior is just the previous posterior: $\overline{\text{bel}}(X_2 = \text{closed}) = \text{bel}(X_1 = \text{closed}) = 0$. The 0 is inherited every step, so the measurement update keeps returning 0 forever. This holds **for either reading** — sensing "open" or "closed" both give 0, because the prior 0 zeroes the numerator regardless of the likelihood value:

$$\text{bel}(X_t = \text{closed}) = \frac{p(z_t \mid \text{closed}) \times 0}{p(z_t)} = 0 \quad \text{for any } z_t$$

$$\boxed{\text{bel}(X_t = \text{closed}) \to 0, \qquad \text{bel}(X_t = \text{open}) \to 1}$$

The belief converges to the **wrong** answer — full confidence in "open" while the door is actually closed.

> **⚠️ The two sensing outcomes must be checked separately, and both give 0.** That the belief is *unmoved* by either reading is itself the symptom: a healthy filter would push "open" down after a "closed" reading, but with the competing hypothesis dead at 0, that force has nothing to act on. The measurement is effectively disabled.

### (c) Intuition, practical risk, and the fix

- **Intuition:** the prior declared "closed" *impossible* (probability 0). Bayes' rule can never resurrect a hypothesis it was told is impossible, so the robot locks onto the wrong state with total certainty.
- **Practical risk — yes, serious:** the robot is confidently wrong, and the belief looks stably converged, so the failure is easy to miss. A perfectly good sensor is rendered useless by the prior.
- **Fix:** never assign a prior of exactly 0 or 1 to a state you are trying to infer. Leave every state a small positive probability (e.g. 0.01) so evidence can revive it.

$$\boxed{\text{Keep every prior in }(0,1)\text{: never assign exactly 0 or 1.}}$$

> **💡 Cromwell's rule.** Probability 0 and 1 mean *impossible* and *certain*, not *very unlikely* and *very likely*. For anything to be learned from data, admit a nonzero chance you might be wrong; otherwise no evidence can update you.

---

## Problem 3 — Three-State Weather HMM

**Dynamics** $p(\text{tomorrow}\mid\text{today})$:

| today \\ tomorrow | sunny | cloudy | rainy |
|---|---|---|---|
| sunny | 0.8 | 0.2 | 0 |
| cloudy | 0.4 | 0.4 | 0.2 |
| rainy | 0.2 | 0.6 | 0.2 |

**Sensor** $p(\text{reading}\mid\text{actual})$:

| actual \\ reading | sunny | cloudy | rainy |
|---|---|---|---|
| sunny | 0.6 | 0.4 | 0 |
| cloudy | 0.3 | 0.7 | 0 |
| rainy | 0 | 0 | 1 |

Day 1 is known **sunny**, so bel_1 = (1, 0, 0) over (sunny, cloudy, rainy). The sensor then observes **cloudy, cloudy** on Days 2 and 3. There are no control inputs, so the dynamics update uses $p(x_t \mid x_{t-1})$ (no $u$).

### Day 2

**Predict** (push $\text{bel}_1$; sunny row of the dynamics table):

$$\overline{\text{bel}}_2 = (0.8,\ 0.2,\ 0)$$

**Correct** (reading = cloudy → likelihoods p(\text{cloudy}\mid\cdot) = (0.4,\ 0.7,\ 0)):

$$\text{numerators} = (0.4 \times 0.8,\ 0.7 \times 0.2,\ 0) = (0.32,\ 0.14,\ 0), \qquad p(Z_2 = \text{cloudy}) = 0.46$$

$$\text{bel}_2 = \left(\tfrac{16}{23},\ \tfrac{7}{23},\ 0\right) \approx (0.696,\ 0.304,\ 0)$$

### Day 3

**Predict** (push $\text{bel}_2$ through all three rows):

$$\overline{\text{bel}}_3(\text{sunny}) = 0.8\cdot\tfrac{16}{23} + 0.4\cdot\tfrac{7}{23} + 0.2\cdot 0 = \tfrac{78}{115}$$

$$\overline{\text{bel}}_3(\text{cloudy}) = 0.2\cdot\tfrac{16}{23} + 0.4\cdot\tfrac{7}{23} + 0.6\cdot 0 = \tfrac{6}{23} = \tfrac{30}{115}$$

$$\overline{\text{bel}}_3(\text{rainy}) = 0\cdot\tfrac{16}{23} + 0.2\cdot\tfrac{7}{23} + 0.2\cdot 0 = \tfrac{7}{115}$$

Check: $\tfrac{78 + 30 + 7}{115} = 1$. **Rainy revives to $\tfrac{7}{115}$** even though it was 0 on Day 2, because cloudy can transition to rainy.

**Correct** (reading = cloudy again → likelihoods $(0.4, 0.7, 0)$):

$$p(Z_3 = \text{cloudy}) = 0.4\cdot\tfrac{78}{115} + 0.7\cdot\tfrac{30}{115} + 0\cdot\tfrac{7}{115} = \tfrac{261}{575}$$

$$\text{bel}_3(\text{cloudy}) = \frac{0.7\cdot\tfrac{30}{115}}{\tfrac{261}{575}} = \frac{105}{261} = \frac{35}{87} \approx 0.402$$

$$\boxed{p(\text{Day 3 is cloudy}\mid \text{observations}) = \tfrac{35}{87} \approx 0.40}$$

> **⚠️ The $1 - \text{other}$ shortcut fails with 3 states.** In the Day-3 normalizer, $\overline{\text{bel}}_3(\text{sunny})$ is $\tfrac{78}{115}$, **not** $1 - \tfrac{6}{23} = \tfrac{85}{115}$. The difference $\tfrac{7}{115}$ is exactly the rainy mass. Folding rainy into sunny inflates the normalizer and drags the answer down (to $\approx 0.38$). With three or more states, $1 - \text{bel(cloudy)} = \text{bel(sunny)} + \text{bel(rainy)}$ — you must compute all components and sum them, never subtract a single one.

---

## Key Lessons

| Problem | Mechanism | Lesson |
|---|---|---|
| P1 | useless sensor, converging set | $h^{-1}$ of a constant sensor is the **whole state space**; measurement is inert |
| P1 | wall collapses states | information can come from **non-injective dynamics + boundaries**, not the sensor |
| P1 | circular track counterexample | irreversibility is what shrinks a belief; a bijection never localizes |
| P2a | prior 0, "closed" reading | $\text{posterior} \propto \text{likelihood} \times \text{prior}$; prior 0 stays 0 |
| P2b | 0 inherited each step | with no control, posterior becomes next prior; wrong belief locks in |
| P2b | both readings give 0 | check each sensing outcome separately; unmoved belief is the symptom |
| P2c | fix | never assign a prior of exactly 0 or 1 (Cromwell's rule) |
| P3 | 3-state normalization | normalizer is the **sum of all** numerators; compute every component |
| P3 | rainy revives | a dynamics step can resurrect a state that was 0 |
| P3 | $1-\text{other}$ trap | the "1 minus one" shortcut holds only for **2 states** |
| all | notation | prior in a measurement update is $\overline{\text{bel}}$; posterior is $\text{bel}$ |
