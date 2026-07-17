# Lecture 14: Localization

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

Lectures 11–13 built the *tool*: a belief carried as a set, then a distribution, then a Gaussian or a particle cloud. Lecture 14 is where that tool meets its first real robotics problem — **localization: "Where am I?"** The robot must recover its position and orientation, and it gets one extra piece of help: a **map** $\bar{m}$ of the environment it operates in.

Almost nothing new is introduced. Localization **is** the Bayes filter; the only structural change is that both models now condition on the map. The real work of this lecture is not new theory but **how to actually write down the two models once a map is in play**.

> **One-line summary:** Localization is the Bayes filter with a map added to the conditioning  $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m})$ and $p(\bar{z}_t \mid \bar{x}_t, \bar{m})$  so once those two models are specified, any technique from Lec 12–13 (discrete Bayes, Kalman, particle) applies unchanged.

> **Where this sits:** the first application of the estimation module. It drops the "the robot knows the map" assumption only halfway — the map is *given* here, and Lec 15–16 go on to build the map and finally to do both at once (SLAM).

---

## Motivation — Why a Map Changes Anything

Through Lec 13 the sensor model $p(\bar{z}_t \mid \bar{x}_t)$ had to answer "what would the sensor read at this state?" out of thin air. With a map, that question becomes concrete: from a candidate pose, the distance to a wall can be **computed geometrically** from the map. The map is what makes the state-to-measurement translation possible in the first place.

> **💡 The map is what lets the sensor model exist.** In Lec 11 the sensor function $h$ turned a state into an expected reading; in Lec 13 the matrix $C$ did the linear version. For a range finder in a room, neither can be written down *without knowing where the walls are*. The map supplies exactly that missing ingredient.

---

## What Is a Map?

A map is a representation of the robot's environment. There are roughly **two kinds** used for localization.

**Location-based map (a.k.a. volumetric map)** associates a *property* with **every** location. The standard example is the **occupancy grid map**, which records whether each location is occupied or not — the same kind of map used for discrete motion planning with A\* in Lec 6–7. Occupancy is the most common property, but others are possible (e.g. the color of an object at that location).

**Feature-based map** stores the properties and locations of a *number of features* in the environment — distinct objects such as a door, a table, or a window. In robotics such objects are called **landmarks**, and the map is simply **a list of landmarks and their locations**.

### The difference that matters

| | Location-based (volumetric) | Feature-based (landmark) |
|---|---|---|
| coverage | a property for **every** location | only **specific** locations |
| encodes absence? | **yes** — free/unoccupied space is recorded | no — silent about everything else |
| good for | **motion planning** (A\* needs to know what is free) | **localization** (convenient, computationally efficient) |
| example | occupancy grid map | list of doors, tables, windows |

> **💡 Satellite photo versus a set of directions.** A location-based map is a satellite image — every square metre is labelled. A feature-based map is "in front of the Starbucks, next to the post office." To *walk a path* you need to know where every wall is; to *figure out where you are standing*, a few landmarks suffice. That is exactly why planning wants the former and localization is often happy with the latter.

---

## Localization Is Just the Bayes Filter

Let $\bar{x}_t$ denote the robot's state. (Localization usually cares about position and orientation, though sometimes velocities too — we just say "the state.")

**Algorithm — Localization.** For every time-step $t$, for all $\bar{x}_t$:

$$\overline{\text{bel}}(\bar{x}_t) = \sum_{\bar{x}_{t-1}} p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m})\,\text{bel}(\bar{x}_{t-1})$$

$$\text{bel}(\bar{x}_t) = \frac{p(\bar{z}_t \mid \bar{x}_t, \bar{m})\,\overline{\text{bel}}(\bar{x}_t)}{p(\bar{z}_t)}$$

Set these beside the Lec 12 Bayes filter and the difference is a single symbol.

| | Lec 12 Bayes filter | Lec 14 Localization |
|---|---|---|
| dynamics model | $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1})$ | $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m})$ |
| sensor model | $p(\bar{z}_t \mid \bar{x}_t)$ | $p(\bar{z}_t \mid \bar{x}_t, \bar{m})$ |
| structure | predict → correct | **identical** |

The map $\bar{m}$ is simply added to the conditioning of both models. Once the dynamics and sensor models are specified, we **apply the techniques from the previous lectures unchanged** — the discrete Bayes filter, the Kalman filter, or the particle filter.

> **💡 Nothing about the filter changed; the modelling did.** Everything interesting in this lecture lives inside those two conditional distributions. That is why the rest of the notes are entirely about how to build them.

---

## Dynamics Model With a Map

Consider **grid localization**: localizing given an occupancy grid map. How should the map enter the dynamics? The simple answer is to **assign zero probability to occupied cells**:

$$p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m}) = 0, \quad \text{if } \bar{x}_t \text{ is occupied}$$

Concretely, if the robot tries to move into an occupied cell, **it remains in its current cell**. The same idea works for continuous state spaces: specify a model that assigns zero probability to occupied locations, for instance with a **truncated Gaussian** (a bell curve cut off where the wall begins).

> **💡 This is Assignment 6, Problem 1.** The five-cell corridor where "pressing Left at cell 1 leaves you at cell 1" *is* this rule. And recall what that problem taught: the wall made the dynamics **non-injective**, collapsing several possible states into one, and that collapse is what shrank the belief to a single cell — with a completely useless sensor. So putting the map into the dynamics is not merely a prohibition; **the map's walls are themselves a source of localization information**.

> **⚠️ A probability of exactly 0 — didn't Assignment 6 warn against this?** Assignment 6(c) concluded that a prior of 0 can never be revived by evidence (Cromwell's rule). The difference is *what* the zero describes. There it was a **prior over the quantity being inferred** (the door's state), which froze the belief forever. Here it is a **dynamics constraint close to physics** (a robot cannot be inside a wall). The tension is still real, though, because this lecture immediately concedes that **maps are not perfect** — if the map is wrong, a hard zero can permanently exclude the robot's true location. This is part of why the *sensor* model below deliberately budgets for map errors.

### Rejection sampling — sampling beats writing it down

In general, writing $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m})$ **explicitly is challenging** — with complicated geometry the formula becomes hopeless. But there is a way out: *for the particle filter, we only need to be able to **sample** from the dynamics model.* There is a simple "hack" to incorporate the map:

```
Algorithm: Rejection Sampling

sample_with_map(x̄_{t-1}, ū_{t-1}, m̄):
    do until π > 0:
        x̄_t = sample_without_map(x̄_{t-1}, ū_{t-1})   # ignore the map, just propose
        π   = p(x̄_t | m̄)                              # is this pose valid on the map?
    return x̄_t
```

Propose while ignoring the map, throw the sample away if it landed inside an obstacle, and repeat until a valid one appears.

> **💡 This is the payoff of the $\sim$ versus $=$ distinction from Lec 13.** *Writing down* a density ($=$) can be intractable while *drawing from it* ($\sim$) stays easy. A particle filter only ever samples from the dynamics — the step that was `update_state(...)` in Lab 6 — so it can sidestep the impossible formula entirely. A Kalman filter cannot use this trick, since it must manipulate the distribution explicitly. This is a concrete reason **particle filters dominate in practical localization**.

---

## Sensor Models With Maps

### Range finders — four factors

A very common sensor for mobile robots is a **range finder** (laser range finder / LIDAR, or ultrasound), which provides distances along different directions. Let $z_t$ be the range measurement along one particular direction. Specifying its model requires accounting for **four factors**:

1. **The correct range.** Given a state $\bar{x}_t$ and a location-based map, this is obtained by **doing some geometry** — trace the ray until it hits an obstacle.
2. **Unexpected obstacles.** Maps are not perfect; something may have moved after the map was built.
3. **Failure to detect an obstacle**, which can happen due to weird reflections.
4. **Random / unexplainable measurements** — e.g. SONARs generating phantom readings when they bounce off walls.

These combine as a weighted **mixture**:

$$p(z_t \mid \bar{x}_t, \bar{m}) = w_\text{hit}\,p_\text{hit} + w_\text{unexp}\,p_\text{unexp} + w_\text{fail}\,p_\text{fail} + w_\text{rand}\,p_\text{rand}$$

(See Ch. 6.3 of *Probabilistic Robotics* for the individual components.)

> **💡 The sensor model budgets for its own failures.** Only the first term describes the sensor working correctly; the other three describe **ways it goes wrong**. If we modelled only correct readings, a single phantom value would look wildly improbable under every hypothesis and could drag the belief somewhere absurd. Keeping $p_\text{rand}$ in the mixture means "nonsense readings happen sometimes," which makes the filter **robust to outliers**. It is Cromwell's rule pointed the other way: leave a little probability for the ridiculous, so the ridiculous cannot break you.

### Landmark maps — much simpler

For landmark-based maps a measurement model can simply be the **distance from the landmark(s)**:

$$p(z_t \mid \bar{x}_t, \bar{m}) = \mathcal{N}(\|\bar{x}_t - \bar{l}_t\|, \sigma^2), \quad \text{where } \bar{l}_t \text{ is the landmark location}$$

> **💡 This is exactly Lab 6's sensor.** Lab 6 used $\mathcal{N}(\|\bar{x}_t\|, \sigma^2)$ — the distance to the **origin**. That is this model with a single landmark sitting at the origin. Lab 6 was landmark-based localization all along.

### Data association

**Practical consideration:** the **data association problem** (or **correspondence problem**) arises when landmarks **cannot be uniquely identified**. Using a camera to detect doors, the robot may get confused about *which* door it detected — believing the sensor reports the distance from Door 1 when it is really reporting the distance from Door 2. A good sensor model should account for such errors in data association.

> **💡 The two-door problem, wearing its third disguise.** The same phenomenon has now appeared three times: in **Lec 13** two identical doors made the belief bimodal, which is why a Gaussian could not represent it and particle filters were introduced; in **Lab 6** a distance-only sensor gave identical weights to $(r, 0)$ and $(-r, 0)$, so the particles clustered in two places; and here it is **data association**. All three have one root — *when a measurement is equally consistent with several states, the belief splits* — and one family of remedies: add information that breaks the symmetry (distinguishable landmarks, more sensors, a bearing), or use a filter that can represent the split.

---

## Concluding Notes

The lecture has discussed a few dynamics and sensor models, but in general **these depend on the particular robot and sensor**. Once the dynamics and sensor models are specified, we simply apply the Bayes filter to perform localization.

> **💡 Why particle filters are "particularly" popular for localization** (as Lec 13 claimed). Two reasons converge here. First, **rejection sampling**: the map-aware dynamics model is painful to write down but trivial to sample from, and sampling is all a particle filter asks for. Second, **data association**: ambiguous landmarks produce multimodal beliefs, which a Gaussian belief (Kalman/EKF) structurally cannot represent but a particle cloud handles for free. Localization is precisely the setting that punishes the Gaussian assumption and rewards sampling.

**Next lecture:** Mapping.

---

## Concept Flow & What's Next

```
Lec 11-13: built the TOOL — belief as a set → a distribution → a Gaussian / particles
   assumed away: the robot knows the map
        ↓
Lec 14: Localization — "Where am I?", given a MAP
   Bayes filter, unchanged, with m̄ added to both models:
      predict:  bel‾(x̄ₜ) = Σ p(x̄ₜ | x̄ₜ₋₁, ūₜ₋₁, m̄) bel(x̄ₜ₋₁)
      correct:  bel(x̄ₜ)  ∝ p(z̄ₜ | x̄ₜ, m̄) · bel‾(x̄ₜ)
   ├── map types:   location-based (occupancy grid; encodes free space → planning)
   │                feature-based  (landmark list; compact → localization)
   ├── dynamics + map:  p = 0 on occupied cells / truncated Gaussian
   │                    hard to write ⇒ REJECTION SAMPLING (particle filter only samples)
   └── sensor + map:    range finder = w_hit·p_hit + w_unexp·p_unexp + w_fail·p_fail + w_rand·p_rand
                        landmark    = N( ‖x̄ₜ − l̄ₜ‖ , σ² )     ← Lab 6, landmark at origin
                        ambiguity   ⇒ DATA ASSOCIATION problem → multimodal belief
        ↓   (here the map was GIVEN — but where does it come from?)
Lec 15: Mapping — build the map, given the robot's poses
Lec 16: SLAM — estimate the state AND the map simultaneously
```

> **💡 Presentation one-liner:** *"Localization adds exactly one symbol to the Bayes filter — the map — and every remaining difficulty moves out of the filter and into the two models: walls in the dynamics, and four ways of lying in the sensor."*

---

## Quick Reference

| Term | Meaning |
|------|---------|
| localization | "Where am I?" — estimate the robot's state given a **map** of the environment |
| map $\bar{m}$ | a representation of the robot's environment; the only new ingredient in this lecture |
| location-based map | a property for **every** location (e.g. occupancy grid); encodes **free space** too → useful for motion planning |
| feature-based map | a **list of landmarks and their locations**; only specific locations → convenient and efficient for localization |
| landmark | a distinct object in the environment (door, table, window) used as a map feature |
| localization algorithm | the Bayes filter with $\bar{m}$ added: $p(\bar{x}_t \mid \bar{x}_{t-1}, \bar{u}_{t-1}, \bar{m})$ and $p(\bar{z}_t \mid \bar{x}_t, \bar{m})$ |
| grid localization | localization given an occupancy grid map |
| dynamics with map | $p = 0$ for occupied cells; a robot that tries to enter a wall **stays put** (cf. A6 Problem 1) |
| truncated Gaussian | the continuous-space version — a Gaussian cut off at obstacles |
| rejection sampling | propose ignoring the map, discard if invalid, repeat; lets a particle filter use a dynamics model too hard to write down |
| sampling vs writing down | $\sim$ (sample) stays easy where $=$ (evaluate/derive the density) is intractable — why particle filters win here |
| range finder | LIDAR / ultrasound; gives distances along directions |
| four factors | correct range (geometry) + unexpected obstacles + failure to detect + random readings |
| mixture sensor model | $w_\text{hit}p_\text{hit} + w_\text{unexp}p_\text{unexp} + w_\text{fail}p_\text{fail} + w_\text{rand}p_\text{rand}$; modelling failures makes the filter robust to outliers |
| landmark sensor model | $\mathcal{N}(\|\bar{x}_t - \bar{l}_t\|, \sigma^2)$ — Lab 6 is this with one landmark at the origin |
| data association | ambiguity about **which** landmark was observed (Door 1 vs Door 2); a good sensor model accounts for it |
| multimodality, three faces | Lec 13 two doors = Lab 6 $(\pm r, 0)$ = Lec 14 data association: one measurement fitting many states splits the belief |
| maps are not perfect | the lecture's own caveat — motivates $p_\text{unexp}$ and tempers the hard zeros in the dynamics |
