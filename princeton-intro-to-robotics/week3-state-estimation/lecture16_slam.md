# Lecture 16: SLAM (Simultaneous Localization and Mapping)

**Course:** Introduction to Robotics (Princeton MAE 345/549)
**Instructor:** Anirudha Majumdar

---

## Overview

This is the finale of the estimation module. Lec 14 (localization) assumed the **map was given**; Lec 15 (mapping) assumed the **state was given**. In reality the robot has neither — a **chicken-and-egg problem**, since good localization needs a map and mapping needs localization. **SLAM** estimates the state and the map **simultaneously**, and it does so not with a new algorithm but by bolting together the three filters already built: particle filter (Lec 13) as the skeleton, occupancy grid mapping (Lec 15) inside each particle, and the sensor model (Lec 14) as the weight.

> **One-line summary:** Maintain $M$ particles, each carrying **both a state and its own map**. Every step: push each particle through the motion model, update that particle's map from its pose, weight it by how well its (pose, map) explains the measurement, then resample the **(state, map) pairs** — so particles whose pose and map are mutually consistent survive.

> **Where this sits:** the point where the module closes. The arc — set (11) → distribution (12) → Gaussian/particles (13) → localization (14) → mapping (15) → SLAM (16) — ends by lifting *both* of Lec 11's hidden assumptions at once. The lecture is candid that SLAM has a vast literature; this is one decent starting point (grid-based FastSLAM).

---

## Two Versions of SLAM

The lecture opens by splitting SLAM into two problems, distinguished by **whether we estimate the current state or the whole trajectory**.

**Online SLAM** — the *instantaneous* state plus the map:

$$p(\bar{x}_t, \bar{m} \mid \bar{z}_{1:t}, \bar{u}_{1:t})$$

**Full SLAM** — the *entire trajectory* plus the map:

$$p(\bar{x}_{1:t}, \bar{m} \mid \bar{z}_{1:t}, \bar{u}_{1:t})$$

The only difference is $\bar{x}_t$ (one pose) versus $\bar{x}_{1:t}$ (every pose from the start).

The lecture adds two notes. **Full SLAM is useful** when the robot must know *where it has already been* — e.g. a search-and-rescue robot avoiding re-searching explored areas. And **online is "simpler"** than Full: solving Full SLAM also yields the online solution.

> **⚠️ "Simpler" means "a sub-problem," not "a fancier method."** Full SLAM carries *more* information (the whole path), so it is the **stronger** problem and is computationally heavier. Online is obtained from Full by **marginalizing out the past trajectory**, $\int\!\cdots\!\int p(\bar{x}_{1:t}, \bar{m} \mid \cdots)\, d\bar{x}_1 \cdots d\bar{x}_{t-1}$. So online is easier only in the sense of being a slice of Full.

> **💡 This lecture solves online SLAM.** Grid-based FastSLAM is a recursive filter — it advances $\bar{x}_t$ each step and never revisits the past trajectory — which is exactly the online formulation. The lecture notes it generalizes to Full SLAM but does not dwell on that. (A hint for later: loop closure, which *does* correct the past, has the flavor of Full SLAM.)

---

## The Core Idea — One Map Per Particle

The idea (grid-based FastSLAM, Ch. 13.10) is to **combine particle-filter localization (Lec 14) with occupancy grid mapping (Lec 15)**. As in particle-filter localization we keep $M$ particles, but now **each particle also carries a map**:

$$\text{particle } k = \big(\underbrace{\bar{x}_t^{[k]}}_{\text{"I am here"}},\ \underbrace{\bar{m}_t^{[k]}}_{\text{"and this is my map"}}\big)$$

The set of $M$ such particles approximately represents the **joint** distribution over states and maps. The lecture's figure shows this literally: particle 1 says "I am here and this is the corresponding map," particle 2 says the same with a *different* pose and a *different* map, and so on — each particle is one hypothesis of pose-and-map together.

> **💡 Many blindfolded explorers, each drawing their own map.** Picture $M$ explorers walking a building with eyes closed, each sketching a map as they go. Some have an accurate path and a clean map; others drifted and their map is warped. Every time a measurement arrives we ask each explorer "does your map agree with what the sensor just saw?" — duplicating those who agree and discarding those who do not. The survivors are the ones whose pose and map are internally consistent.

---

## The Algorithm, Line by Line

Two loops per time step: the first advances, maps, and scores each particle; the second resamples.

```
At each time-step:
  for k = 1 to M do
      x̄ₜ^[k] = sample_motion_model(x̄ₜ₋₁^[k], ūₜ₋₁, m̄ₜ₋₁^[k])   [Dynamics update]
      m̄ₜ^[k] = updated_occupancy_grid(z̄ₜ, x̄ₜ^[k], m̄ₜ₋₁^[k])    [Update map]
      wₜ^[k] = measurement_model(z̄ₜ, x̄ₜ^[k], m̄ₜ^[k])          [Importance weight]
  end
  Initialize particle set Sₜ as empty
  for k = 1 to M do
      Draw i with probability proportional to wₜ^[i]
      Add (x̄ₜ^[i], m̄ₜ^[i]) to Sₜ
  end
  Return Sₜ
```

**Line 1 — Dynamics update (Lec 13 predict).** Sample a new pose from the motion model:

$$\bar{x}_t^{[k]} \sim p(\bar{x}_t \mid \bar{x}_{t-1}^{[k]}, \bar{u}_{t-1}, \bar{m}_{t-1}^{[k]})$$

The three inputs are the particle's previous pose, the control input (shared by all particles — the robot issued one command), and the particle's own map.

> **⚠️ Why the map appears in the dynamics.** From Lec 14, the map constrains motion ("cannot move into a wall," $p = 0$ if occupied), so pushing a particle forward must consult *its* map. And because particles hold different maps, the same command $\bar{u}_{t-1}$ can send them to different places — one particle's map blocks a doorway the other's leaves open.

> **💡 `sample` is the $\sim$ operation.** This draws from the dynamics distribution (Lab 6's `update_state`), not evaluates a density — the $\sim$/$=$ distinction from Lec 13. Process noise is where particle diversity is born.

**Line 2 — Update map (Lec 15 mapping).** From the new pose, fold the measurement into that particle's map, $\bar{m}_{t-1}^{[k]} \to \bar{m}_t^{[k]}$, using the occupancy-grid technique from last lecture.

> **⚠️ Order matters: pose first, then map.** Updating a map requires knowing *where* the measurement was taken. Lec 15 assumed the state was given; in SLAM that "given state" is exactly what Line 1 just produced. Each particle thus maps the world **assuming its own pose is correct**, accumulating a coherent map over time — clean if its pose track is accurate, warped if it drifted.

**Line 3 — Importance weight (Lec 14 sensor model).** Score how well this particle's (pose, map) explains the reading:

$$w_t^{[k]} = p(\bar{z}_t \mid \bar{x}_t^{[k]}, \bar{m}_t^{[k]})$$

> **💡 The weight scores pose-map consistency — and that is how SLAM cracks the chicken-and-egg.** A particle whose map (from Line 2) agrees with its pose explains the measurement well → high weight; a drifted particle's map clashes with the reading → low weight. So "accurate pose" and "clean map" are rewarded *together*, in one number. SLAM never aligns pose and map separately; it keeps particles where the two are already consistent. (`measurement_model` is the $=$ operation — evaluating a density, Lab 6's `.pdf(z_t)`.)

**Resampling loop (Lec 13 resample).** Draw $M$ indices with replacement in proportion to the weights, adding the chosen **(state, map) pairs** to $S_t$. Identical to the particle filter, this discards particles+maps of low weight.

> **⚠️ The one difference from localization resampling: the map rides along.** Drawing index $i$ copies $(\bar{x}_t^{[i]}, \bar{m}_t^{[i]})$ **as a pair** — a duplicated particle brings its map, a killed particle takes its map with it. Choosing a good pose automatically keeps the good map bound to it; the two are selected as a unit, never optimized apart.

**Extracting a single estimate.** For tasks like motion planning that need one (state, map), pick the **particle with the highest importance weight** as the best estimate.

> **💡 This answers Lec 12's opening question** — "given a distribution/set, what do we feed a controller?" A particle filter's answer: take the single most probable particle.

| line | concept | operation | Lab 6 analogue |
|---|---|---|---|
| `sample_motion_model` | Lec 13 predict | sample ($\sim$) | `update_state(...)` |
| `updated_occupancy_grid` | Lec 15 mapping | log-odds update | (none — map was fixed) |
| `measurement_model` | Lec 14 sensor model | evaluate density ($=$) | `st.norm(...).pdf(z_t)` |
| resample loop | Lec 13 resample | draw $\propto w$, with replacement | `np.random.choice` + `particles[idx]` |

---

## Loop Closure

As the map is built, **errors accumulate**: small pose errors warp the map, so after a long loop the start point fails to line up (the "before loop closure" figure, where the rectangle does not close). **Loop closure** corrects this by recognizing that the robot has **returned to a previously visited location** and using that to realign the map (the "after" figure closes cleanly).

### FastSLAM closes loops automatically

Remarkably, there is **no separate loop-closure code** — running the algorithm above is enough.

- **Intuition:** on returning to a known place, the particle whose map is **correct** already has that place drawn where it belongs, so the new measurement matches its map and it earns a **high importance weight**. Drifted particles have the revisited spot in the wrong place, so they mismatch and get **low weight**.
- **Recall:** resampling throws away low-weight particles. So the accurate-map (accurate-trajectory) particles are duplicated and the drifted ones die, and the whole belief snaps into a consistent, closed map.

> **💡 Loop closure here is emergent, not explicit.** No particle announces "I've been here before." The revisit measurement simply hands higher scores to particles whose maps are self-consistent, and natural selection does the rest. The Line-3 weight, which enforced *local* pose-map consistency, enforces *global* consistency once the path loops. This is Lec 14's **data-association** idea — "which past place is this?" — resolved by weight competition rather than by matching landmarks by hand.

### The catch — maintain particle diversity

For loop closure to actually happen, two conditions are needed:

- FastSLAM must keep a **sufficiently large number of particles**.
- The particle set must retain **diversity**.

> **⚠️ Resampling can exhaust the very hypothesis loop closure needs — particle depletion.** At the moment of revisit, the correct-map particle must still be *alive* in the set to win and reproduce. But resampling repeatedly duplicates a few survivors, so the particles drift toward being copies of one another (diversity collapse, a.k.a. sample impoverishment — the same family of failure as Lab 6's "all weights near zero"). Loop closure arrives only after a long journey, and resampling is short-sighted: it culls whatever has low weight *now*, potentially killing the particle that would have been vindicated when the loop finally closed. Practical FastSLAM defends diversity — e.g. resampling only when the effective number of particles drops, or injecting small randomness — so a future-correct hypothesis survives long enough to close the loop.

---

## Mapping: Broader Implications

SLAM is a **powerful technology with many applications** (e.g. the **Roomba 980** navigating a home with VSLAM), but it raises **societal questions**. Chief among them: **how do we ensure privacy when millions of mapping robots are deployed across homes?** A vacuum that maps your house produces data about your home's layout. There is room for technical solutions (privacy-preserving control), but broader **policy and legislation** deserve thought too.

> **💡 A bookend with the Lec 7 bonus.** This echoes the value-alignment / Paperclip-Maximizer discussion — judging a technology not only by its performance but by its consequences once embedded in society. It is a recurring thread in the course.

---

## Concluding Notes

SLAM does not introduce new machinery; it composes what the module already built. A particle filter (Lec 13) is the skeleton, each particle runs occupancy grid mapping (Lec 15), and the sensor model (Lec 14) supplies the weight — with pose and map bound together so resampling selects consistent pairs. Loop closure then falls out for free, provided the particle set stays diverse enough to keep the right hypothesis alive.

> **💡 One forward model, every unknown.** Lec 14 inverted the sensor model for the state (localization); Lec 15 inverted it for the map (mapping); Lec 16 estimates **both at once** by carrying joint (state, map) hypotheses. From "we know neither where we are nor what the world looks like" to "we recover both simultaneously" — the module's whole point.

**Next:** broader topics — jobs, ethics, and laws in robotics.

---

## Concept Flow & What's Next

```
Lec 11-13: built the TOOL — set → distribution → Gaussian / particles
Lec 14: Localization — estimate x̄, map GIVEN
Lec 15: Mapping      — estimate m̄, state GIVEN
        ↓   (in reality we have NEITHER — chicken and egg)
Lec 16: SLAM — estimate state AND map together:  p(x̄ₜ, m̄ | z̄₁:ₜ, ū₁:ₜ)
   ├── two versions:
   │      Online SLAM  p(x̄ₜ,  m̄ | ...)   ← instantaneous state (THIS lecture: FastSLAM)
   │      Full SLAM    p(x̄₁:ₜ, m̄ | ...)   ← entire trajectory (stronger; online = marginalize past)
   ├── Grid-based FastSLAM = particle filter(13) + occupancy grid mapping(15)
   │      each particle = (state x̄^[k], its own map m̄^[k])   → joint distribution
   │      per particle:  sample_motion_model (13, ~)
   │                     updated_occupancy_grid (15)
   │                     measurement_model (14, =)   ← scores pose-map consistency
   │      resample (state, map) PAIRS (13)           ← keeps consistent particles
   │      single estimate = highest-weight particle
   ├── Loop closure: AUTOMATIC — correct-map particle wins weight on revisit
   │                 needs enough particles + DIVERSITY (else particle depletion)
   └── Broader implications: privacy (Roomba), policy — bookend to Lec 7 bonus
        ↓
Lec 17+: broader topics in robotics 
```

> **💡 Presentation one-liner:** *"SLAM is not a new algorithm — it's the module assembling itself: give every particle its own map, run mapping inside it, weight it by the sensor model, and resample the (pose, map) pair. Loop closure then happens for free, as long as you keep enough diverse particles alive to catch it."*

---

## Quick Reference

| Term | Meaning |
|------|---------|
| SLAM | Simultaneous Localization And Mapping — estimate state **and** map together |
| chicken-and-egg | localization needs a map, mapping needs localization; SLAM does both at once |
| online SLAM | estimate the **instantaneous** state + map, $p(\bar{x}_t, \bar{m} \mid \cdots)$; recursive, real-time (this lecture) |
| full SLAM | estimate the **entire trajectory** + map, $p(\bar{x}_{1:t}, \bar{m} \mid \cdots)$; stronger, heavier; useful when the robot must know where it has been |
| online ⊂ full | marginalize the past trajectory out of Full to get online; "online is simpler" = it is a sub-problem |
| grid-based FastSLAM | the algorithm here: particle-filter localization (14) + occupancy grid mapping (15) |
| particle = (state, map) | each particle carries a pose **and** its own map; $M$ particles ≈ the joint distribution |
| `sample_motion_model` | Line 1, Lec 13 predict; samples $\bar{x}_t^{[k]} \sim p(\bar{x}_t \mid \bar{x}_{t-1}^{[k]}, \bar{u}_{t-1}, \bar{m}_{t-1}^{[k]})$ |
| map in the dynamics | motion must respect the particle's own map (no moving into walls, Lec 14) |
| `updated_occupancy_grid` | Line 2, Lec 15 mapping; updates the particle's map from its new pose — pose first, then map |
| `measurement_model` | Line 3, Lec 14 sensor model; $w_t^{[k]} = p(\bar{z}_t \mid \bar{x}_t^{[k]}, \bar{m}_t^{[k]})$; scores pose-map consistency |
| resample pairs | draw $\propto w$ with replacement, carrying **(state, map)** together; good pose keeps its good map |
| best estimate | for planning, take the single highest-weight particle |
| loop closure | recognizing a revisited place to correct accumulated map error; FastSLAM does it **automatically** via weights |
| emergent loop closure | no explicit code — the correct-map particle simply wins weight and resampling realigns the belief |
| particle depletion | resampling collapses diversity; the correct hypothesis may die before the loop closes → keep many, diverse particles |
| broader implications | privacy of home maps (Roomba VSLAM); technical + policy solutions; bookend to the Lec 7 value-alignment bonus |
