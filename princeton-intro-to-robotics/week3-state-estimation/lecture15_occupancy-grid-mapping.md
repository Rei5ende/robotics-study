# Lecture 15: Mapping (Occupancy Grid Mapping)

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Localization (Lec 14) asked "where am I?" *given a map*. Mapping flips the question: **assume the robot knows its own state, and estimate the map instead.** Formally we want $p(\bar{m} \mid \bar{x}_{1:t}, \bar{z}_{1:t})$ — the map given the full history of states and measurements. The filter machinery is the same Bayes update; what changes is that the unknown is now the map, and a naive treatment of that unknown blows up combinatorially, so most of the lecture is about the tricks that keep it tractable.

> **One-line summary:** With the state assumed known, estimate each grid cell's occupancy independently (Naive Bayes), start every cell at prior 0.5, and update it with the sensor model using an **odds ratio** so the intractable normalizer cancels — implemented as additions in **log-odds**. There is no dynamics update, because the map is static.

> **Where this sits:** the second half of relaxing Lec 11's two assumptions. Lec 14 dropped "the robot knows its state" (map given); Lec 15 drops "the robot knows the map" (state given). This is deliberately artificial — a **chicken-and-egg** problem, since good localization needs a map and mapping needs localization — and Lec 16 (SLAM) finally does both at once.

---

## The Estimation Target Has Flipped

| Lecture | Estimates | Given |
|---|---|---|
| Lec 12–13 | **state** $\bar{x}$ | (no map) |
| Lec 14 | **state** $\bar{x}$ | map $\bar{m}$ |
| **Lec 15** | **map** $\bar{m}$ | **state** $\bar{x}$ |

The lecture is explicit that assuming known state is a simplification: *good localization requires a map, but mapping requires localization.* Today we grant perfect localization and estimate only the map; the next lecture removes the crutch.

> **💡 The module isolates one unknown at a time.** Lec 11 exposed two hidden assumptions — that the robot knows its state and knows the map. Lec 14 lifts the first, Lec 15 lifts the second, and Lec 16 (SLAM) lifts both together. Learn each in isolation, then combine.

---

## What Kind of Map?

Under the two broad classes from Lec 14 (location-based / feature-based), three concrete options:

| Map | What it is | Pros | Cons |
|---|---|---|---|
| **Geometric primitive** | obstacles as polygons / spheres (like the RRT assignment) | memory efficient | complicated geometry needs many pieces |
| **Occupancy grid** | each cell marked occupied or free | represents arbitrary geometry at a chosen precision | heavy memory/compute; an $n \times n$ grid has $n^2$ cells ($n^3$ in 3D) |
| **Semantic / feature** | object locations plus meaning (kind, color) | good for high-level planning ("go to the kitchen") | poor for obstacle avoidance (may not capture every object) |

This lecture uses the **occupancy grid map** — common in practice, with tricks to improve efficiency.

> **💡 The geometric primitive map is Lab 4's RRT world.** Those obstacles were `[(center, radius), ...]` — a handful of circles stored as a few numbers. That tiny footprint is exactly what "memory efficient" means, and the trade-off (hard to draw complicated shapes) is why occupancy grids exist.

---

## Goal and the $2^N$ Explosion

Denote the map by a random variable $\bar{m}$. The goal is to use state and measurement data to estimate it:

$$p(\bar{m} \mid \bar{x}_{1:t}, \bar{z}_{1:t})$$

For an occupancy grid, $\bar{m} \in \{0, 1\}^N$: each of the $N$ cells is either **0 (free)** or **1 (occupied)**, so the map is a length-$N$ binary vector.

Here the trouble starts. With each cell binary, the number of possible maps is $2^N$. A $100 \times 100$ grid has $N = 10{,}000$, giving $2^{10000}$ candidate maps — far more than the number of atoms in the universe. Maintaining a distribution over that space is **impossible in principle**.

---

## Escape 1 — Independence (Naive Bayes)

Assume each cell is **independent** of the others. The joint factors into a product:

$$p(\bar{m} \mid \bar{x}_{1:t}, \bar{z}_{1:t}) = \prod_i p(m_i \mid \bar{x}_{1:t}, \bar{z}_{1:t})$$

A single distribution over $2^N$ maps becomes $N$ separate binary distributions — for a $100 \times 100$ grid, $2^{10000}$ collapses to **10,000 numbers**, one occupancy probability per cell (the free probability is $1 - p$, so it is free).

Is the assumption true? The lecture is candid: *not really.* If cell $i$ is occupied, its neighbor is likely occupied too (think of a wall); real environments are full of such correlations, and independence discards all of them. The computational payoff is worth it anyway. This assumption is called **Naive Bayes**.

> **💡 Why "naive."** The $\prod$ *is* the independence assumption written as a symbol — recall from Lec 12 that $p(x, y) = p(x)p(y)$ is the definition of independence. Factoring the map this way knowingly throws away all cell-to-cell correlation, trading accuracy for tractability. It is "naive" precisely because the independence is obviously false, yet it works remarkably well in practice.

> **⚠️ The product is not free.** The moment the joint is written as $\prod_i p(m_i \mid \cdots)$, every correlation between cells is gone. The lecture's "not really valid" caveat lives entirely inside that one symbol.

---

## Escape 2 — Prior of 0.5

Each cell needs a prior $p(m_i)$. There is no real prior over maps to appeal to, so in practice every cell starts at $p(m_i) = 0.5$.

> **⚠️ This is Cromwell's rule from Assignment 6(c).** If a cell were initialized at $p_0 = 0$ (can never be occupied) or $1$ (must be occupied), no sensor measurement could ever change it — the robot could observe a wall in that cell a hundred times and the belief would not move, because $\text{posterior} \propto \text{likelihood} \times \text{prior}$ and a prior of 0 zeroes everything. So 0.5 is the safe choice: it means "I know nothing," the state of maximum uncertainty, and it leaves room to move in either direction. The lesson learned on the door problem is now a concrete design decision.

---

## Bayes on a Single Cell

Estimate one cell $m_i$. With the state $\bar{x}$ as a background condition, apply **conditional Bayes** — the ordinary rule with $\bar{x}$ carried through every term:

$$p(m_i \mid z_i, \bar{x}) = \frac{p(z_i \mid m_i, \bar{x})\,p(m_i \mid \bar{x})}{p(z_i \mid \bar{x})}$$

| term | role | what it is |
|---|---|---|
| $p(m_i \mid z_i, \bar{x})$ | posterior | what we want — is this cell occupied, given the reading? |
| $p(z_i \mid m_i, \bar{x})$ | likelihood | **the Lec 14 sensor model** |
| $p(m_i \mid \bar{x})$ | prior | 0.5 (after the approximation below) |
| $p(z_i \mid \bar{x})$ | normalizer | intractable — cancelled by the odds trick below |

> **💡 Conditioning on $\bar{x}$ means doing Bayes "inside a world where $\bar{x}$ is known."** $\bar{x}$ is not the thing being estimated; it rides along as a condition on all four terms. Since a conditional distribution is itself a perfectly good distribution, every rule of probability — including Bayes' rule — holds inside it. Nothing new is needed.

> **💡 The likelihood is the Lec 14 sensor model, reused verbatim.** Localization used $p(\bar{z}_t \mid \bar{x}_t, \bar{m})$ — conditioned on **both** state and map. That same object slots straight into the likelihood slot here. The two lectures share one forward model $p(z \mid x, m)$; Bayes just inverts it in different directions — unknown $\bar{x}$ gives localization, unknown $\bar{m}$ gives mapping.

> **⚠️ The prior is approximated as $p(m_i \mid \bar{x}) = p(m_i)$.** This says "the state tells us nothing about the map," which the lecture flags as *only an approximation — think about this point*. It is false: if the robot is standing in cell $i$, that cell is free (a robot cannot be inside a wall). This is exactly the entanglement Lec 14's dynamics model already declared with $p(\bar{x}_t \mid \cdots, \bar{m}) = 0$ for occupied cells. Strictly $p(m_i \mid \bar{x}) \neq p(m_i)$; the approximation is for convenience.

---

## The Key Trick — Cancel the Normalizer With Odds

The only unavailable term is the normalizer $p(z_i \mid \bar{x})$. Expanded, it requires summing over **every possible map** — $2^N$ terms — so it cannot be computed. The escape is to **avoid computing it rather than compute it**: divide the $m_i = 1$ equation by the $m_i = 0$ equation.

$$\frac{p(m_i = 1 \mid z_i, \bar{x})}{p(m_i = 0 \mid z_i, \bar{x})} = \frac{p(z_i \mid m_i = 1, \bar{x})\,p(m_i = 1)}{p(z_i \mid m_i = 0, \bar{x})\,p(m_i = 0)}$$

The normalizer $p(z_i \mid \bar{x})$ appears in both numerators and **cancels** — the two equations share the same denominator. Since $m_i$ is binary, $p(m_i = 0 \mid z_i, \bar{x}) = 1 - p(m_i = 1 \mid z_i, \bar{x})$, so the left side is the **odds** of occupancy, and the right side contains only known quantities (sensor model and prior). Solve for $p(m_i = 1 \mid z_i, \bar{x})$ and the cell is updated.

> **💡 This is the decisive use of "the denominator is a state-independent constant."** In Lab 6 the normalizer was computed directly (`weights / np.sum(weights)`) because summing 1000 particles is easy; in the weather HMM (Assignment 6.3) it was a sum over 3 states. Here the sum has $2^N$ terms and computing it is hopeless — so the strategy shifts from *normalize* to *cancel*. The same "shared denominator" fact that once let us normalize now lets us make the denominator disappear.

> **⚠️ The odds trick is clean only because the cell is binary.** It relies on $p(m_i = 0) = 1 - p(m_i = 1)$ — the same two-state fact that made the "1 minus the other" shortcut work for the door but fail for the 3-state weather problem (Assignment 6.3). An occupancy grid making each cell binary is what makes this whole approach tidy.

---

## No Dynamics Update — the Map Is Static

After the first measurement, further readings update recursively: the previous step's probability becomes the new prior. But the lecture makes a crucial remark: *there is no dynamics update here (assuming obstacles are static).* Walls do not walk around, so the map does not evolve between steps.

> **💡 This is the same half-filter as Assignment 6, Problem 2(b).** There the dynamics update vanished because there were **no control inputs** (the robot never pushed the door), leaving measurement-update-only, with $\overline{\text{bel}}(x_t) = \text{bel}(x_{t-1})$ — the previous posterior becoming the next prior directly. Here the dynamics update vanishes for a different reason — the **estimation target is static** — but the structure is identical: a filter with only the correct step. Of the predict–correct rhythm built since Lec 11, only correct remains.

---

## Log-Odds — Turning Products Into Sums

Dividing probabilities is numerically dangerous: small probabilities lose precision in floating point. So the recursion is carried in **log-odds**, where the multiplicative update becomes additive:

$$\log \frac{p(z_i \mid m_i = 1, \bar{x})\,p(m_i = 1)}{p(z_i \mid m_i = 0, \bar{x})\,p(m_i = 0)} = \log p(z_i \mid m_i = 1, \bar{x}) + \log p(m_i = 1) - \log p(z_i \mid m_i = 0, \bar{x}) - \log p(m_i = 0)$$

Each new measurement just **adds** its contribution instead of multiplying.

> **💡 What log-odds encodes.** Zero is neutral (50:50); positive leans occupied, negative leans free; larger magnitude means stronger confidence. Each measurement adds its own score, so evidence **accumulates** — "saw a wall three times → +3." The 0.5 prior maps to log-odds **exactly 0**, so "no evidence yet" lands cleanly on the number zero.

> **💡 Why the product form is dangerous.** Multiplying $0.001$ ten times gives $10^{-30}$, which floating point handles poorly (**underflow**). In log space that is adding $-3$ ten times to get $-30$ — no problem at all. This is the general reason logs appear throughout probabilistic computation.

---

## Concluding Notes

Occupancy grid mapping is a Bayes update with the estimation target flipped to the map. Every hard part is a way of coping with the size of the map space: independence shrinks $2^N$ to $N$, a 0.5 prior keeps every cell revisable, the odds ratio removes an uncomputable normalizer, and log-odds keeps the arithmetic stable. Because the map is static, only the measurement update survives.

> **💡 One forward model, inverted two ways.** Bayes' rule inverts a conditional. We possess only the forward sensor model $p(z \mid x, m)$. Not knowing $\bar{x}$ inverts toward the state → **localization** (Lec 14); not knowing $\bar{m}$ inverts toward the map → **mapping** (Lec 15). Lec 16 (SLAM) asks what to do when **both** are unknown.

**Next lecture:** SLAM — simultaneous localization and mapping.

---

## Concept Flow & What's Next

```
Lec 11: two hidden assumptions — robot knows its STATE, and knows the MAP
        ↓
Lec 14: Localization — drop "knows state"; estimate x̄ given map m̄
        ↓   (flip the unknown)
Lec 15: Mapping — drop "knows map"; estimate m̄ given state x̄
   goal:  p(m̄ | x̄₁:ₜ, z̄₁:ₜ)   with m̄ ∈ {0,1}ᴺ  ⇒  2ᴺ possible maps (intractable)
   ├── Escape 1: independence (Naive Bayes)   ∏ᵢ p(mᵢ | ...)   ⇒  2ᴺ → N numbers
   ├── Escape 2: prior 0.5 on every cell      (Cromwell's rule → never 0 or 1)
   ├── per-cell Bayes:  p(mᵢ | zᵢ, x̄) ∝ p(zᵢ | mᵢ, x̄) · p(mᵢ)   ← Lec 14 sensor model
   ├── normalizer p(zᵢ | x̄) = sum over 2ᴺ maps ⇒ intractable
   │        ⇒ ODDS TRICK: divide mᵢ=1 by mᵢ=0, denominator CANCELS (binary only)
   ├── no dynamics update — the map is STATIC (half-filter, like A6 P2b)
   └── LOG-ODDS: products → sums; add a score per measurement; 0.5 ↔ 0; avoids underflow
        ↓   (but here the state was GIVEN — and localization needed a map...)
Lec 16: SLAM — estimate the state AND the map simultaneously (chicken and egg)
```

> **💡 Presentation one-liner:** *"Mapping is localization run backwards — same sensor model, but now the map is the unknown. The whole lecture is four tricks for surviving the fact that there are 2ᴺ possible maps: independence, a 0.5 prior, cancelling the normalizer with odds, and log-odds for stability."*

---

## Quick Reference

| Term | Meaning |
|------|---------|
| mapping | estimate the **map** $\bar{m}$, assuming the robot's state $\bar{x}$ is known |
| chicken-and-egg problem | good localization needs a map, mapping needs localization; resolved only by SLAM (Lec 16) |
| geometric primitive map | obstacles as polygons/spheres (Lab 4 RRT world); memory efficient, weak on complex shapes |
| occupancy grid map | each cell occupied/free; arbitrary geometry at a chosen precision; $n^2$ cells in 2D, $n^3$ in 3D |
| semantic / feature map | object locations + meaning; good for high-level planning, weak for obstacle avoidance |
| $\bar{m} \in \{0,1\}^N$ | the map as a length-$N$ binary vector; $2^N$ possible maps |
| $2^N$ explosion | a distribution over all maps is intractable ($100 \times 100 \Rightarrow 2^{10000}$) |
| Naive Bayes / independence | assume cells independent: $p(\bar{m} \mid \cdots) = \prod_i p(m_i \mid \cdots)$; turns $2^N$ into $N$ numbers |
| $\prod$ (product) | the "$\sum$ for multiplication"; here it **is** the independence assumption |
| independence caveat | false in reality (neighboring cells correlate, e.g. walls); accepted for tractability |
| prior 0.5 | "know nothing"; a cell at exactly 0 or 1 could never be revised (Cromwell's rule) |
| conditional Bayes | $p(a \mid b, c) = p(b \mid a, c)\,p(a \mid c)/p(b \mid c)$; the background condition $c = \bar{x}$ rides all terms |
| likelihood $p(z_i \mid m_i, \bar{x})$ | the Lec 14 sensor model, reused; one forward model inverted for either $\bar{x}$ or $\bar{m}$ |
| prior approximation | $p(m_i \mid \bar{x}) \approx p(m_i)$ is only approximate — a robot standing in a cell implies it is free |
| normalizer $p(z_i \mid \bar{x})$ | requires summing over $2^N$ maps → uncomputable |
| odds trick | divide the $m_i = 1$ equation by the $m_i = 0$ one; the shared normalizer **cancels** |
| binary requirement | the odds/`1 − other` step is clean only for two states (cf. 3-state weather failure) |
| no dynamics update | the map is static, so only the measurement update survives (half-filter, like A6 P2b) |
| log-odds | carry the recursion additively; 0 = neutral, + = occupied, − = free; avoids floating-point underflow |
