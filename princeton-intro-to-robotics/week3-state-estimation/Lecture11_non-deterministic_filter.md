# Lecture 11 — Non-deterministic Filter

## Where this lecture fits

Everything so far — feedback control and motion planning — quietly rested on two assumptions: that the robot can perfectly measure its state $\bar{x}(t)$, and that it already knows a full map of its environment. Neither holds in reality. Lectures 11–16 relax both, building up to estimating state from noisy sensors, constructing a map, and eventually doing both at once (SLAM). This lecture is the conceptual entry point: state estimation stripped down to its bare logic, reasoning with **sets of possible states** rather than probabilities. That set-based framing is what the *non-deterministic* in the name refers to.

## Motivation: intersecting circles

Take a 2-DOF point robot with state $\bar{x} = (x, y)$. Suppose one sensor reports distance to the origin. A single measurement $r$ tells the robot only that $\sqrt{x^2 + y^2} = r$ — it lies **somewhere on a circle** of radius $r$. That one number does not pin down the state. Adding a second sensor, distance to another point such as $(1,1)$, pins the robot to the **intersection of two circles**, which is generically two points. A third sensor, distance to a third reference point, cuts that down to a **single point**.

This is exactly how GPS trilateration works: no single distance fixes you, but enough independent distances, intersected, do. The whole lecture generalises this one idea — take independent constraints and intersect them to narrow things down.

For the three-sensor example, the sensor function is a stacked vector of distances:

```
h(x̄) = [ √(x² + y²)            ]   ← distance to (0,0)
        [ √((x-1)² + (y-1)²)    ]   ← distance to (1,1)
        [ √((x-x₃)² + (y-y₃)²)  ]   ← distance to (x₃,y₃)
```

Each component defines one circle, and the state consistent with all three measurements at once is the intersection of the three circles.

## Single-step estimation: sensor function and preimage

Let $\bar{z}$ be a **measurement** (observation). With three distance sensors, $\bar{z} \in \mathbb{R}^3$. We assume a known **sensor function** (or sensor mapping) that says what measurement a given state produces:

$$\bar{z} = h(\bar{x})$$

This runs *forward*: state in, measurement out. Estimation needs the *reverse* direction — given a measurement, which states are consistent with it? That set is the **preimage**:

$$h^{-1}(\bar{z}) = \{\, \bar{x} \in \mathcal{X} \mid h(\bar{x}) = \bar{z} \,\}$$

In words: the set of all states that would have produced measurement $\bar{z}$. For the single-circle case, that preimage *is* the circle.

> **$h^{-1}$ is not a true inverse.** The map $h$ is generally not invertible — one distance value corresponds to a whole circle of states. So $h^{-1}(\bar{z})$ is not the mathematical inverse function; it is standard notation for "h-reverse," going from an observation back to the set of observation-consistent states. When the preimage happens to be a single point (a singleton), the state is uniquely determined, which corresponds to $h$ being locally invertible.

## Adding dynamics: the idealized multi-step case

The single-step view ignores that the robot moves. Introduce a discrete-time dynamics model (control inputs ignored for now):

$$\bar{x}_{t+1} = f(\bar{x}_t)$$

Represent the time-0 estimate as a **set** $\hat{X}_0 \subseteq \mathcal{X}$, not a single point — the robot believes it is *somewhere in this region*. Pushing that whole set one step forward through the dynamics gives every state reachable at $t = 1$:

$$f(\hat{X}_0) = \{\, \bar{x} \in \mathcal{X} \mid \exists\, \bar{x}_0 \in \hat{X}_0 \text{ s.t. } f(\bar{x}_0) = \bar{x} \,\}$$

The existential quantifier is the whole point: a state $\bar{x}$ belongs to $f(\hat{X}_0)$ if there exists *some* starting state in $\hat{X}_0$ that maps to it. This is the image of the belief set under $f$ — the set pushed forward one step.

> **Three different objects share the subscript 0 — keep them apart.** Earlier, $\bar{x}_0 = (1,1)$ was a fixed *sensor reference point* with no time meaning; $\hat{X}_0$ is the *belief set* at time 0; and $\bar{x}_0$ inside the definition above is a *dummy variable* ranging over $\hat{X}_0$. In a general time-$t$ step the dummy variable should carry the source set's index — write $\bar{x}_{t-1} \in \hat{X}_{t-1}$, not $\bar{x}_0$ — so every subscript stays consistent.

## Fusing prediction and measurement

At $t = 1$ the robot also receives a measurement $\bar{z}_1$, giving two independent pieces of information. From the dynamics: the robot is somewhere in $f(\hat{X}_0)$. From the sensor: it is somewhere in $h^{-1}(\bar{z}_1)$. Both must hold simultaneously, and "both" means **intersection**:

$$\hat{X}_1 = f(\hat{X}_0) \cap h^{-1}(\bar{z}_1)$$

This is the same move as intersecting circles in the motivation, except one set is a **dynamics-sourced prediction** and the other a **measurement-sourced constraint**. A useful way to feel the difference: $f(\hat{X}_0)$ is the *eyes-closed guess* — where the robot could be based only on how it moved — and $h^{-1}(\bar{z}_1)$ is the *eyes-open check* — where the sensor says it could be. Neither set is "the answer" on its own; each is a cloud of candidates, and only states lying in both survive.

> **Why intersection, not union.** The robot must be at a state that is *both* reachable given where it was *and* consistent with what it now senses. Logical AND corresponds to set intersection. A union would keep states supported by only one source, which is the opposite of narrowing down.

## The non-deterministic filter

Iterating the fuse step gives the filter. At each time $t$:

1. **Dynamics update** — compute $f(\hat{X}_{t-1})$, the states reachable from the previous belief.
2. **Measurement update** — compute $h^{-1}(\bar{z}_t)$, the states consistent with the current measurement.
3. **Fuse** — intersect them: $\hat{X}_t = f(\hat{X}_{t-1}) \cap h^{-1}(\bar{z}_t)$.

This predict-then-correct rhythm is the skeleton of every filter that follows (Kalman, particle). The dynamics update tends to **grow** the set — uncertainty spreads as the robot moves and one starting state can fan out to many — while the measurement update **shrinks** it, since each observation carves away inconsistent states. Filtering is that repeated expand–contract cycle.

## Adding control inputs (Exercise 1)

The lecture's first exercise asks how this changes with control inputs. The robot *knows* the input it commanded, so $\bar{u}_t$ is a known quantity, not something to estimate. Only the dynamics update changes. The dynamics becomes $\bar{x}_{t+1} = f(\bar{x}_t, \bar{u}_t)$, and the reachable-set definition simply carries that known input:

$$f(\hat{X}_{t-1}, \bar{u}_{t-1}) = \{\, \bar{x} \in \mathcal{X} \mid \exists\, \bar{x}_{t-1} \in \hat{X}_{t-1} \text{ s.t. } f(\bar{x}_{t-1}, \bar{u}_{t-1}) = \bar{x} \,\}$$

Only one input appears, $\bar{u}_{t-1}$, because a single transition from $\bar{x}_{t-1}$ to $\bar{x}_t$ consumes exactly one command. The measurement update and the intersection are untouched — the sensor function $h$ reads state only, never the input. Intuitively, knowing the input *sharpens* the prediction: rather than allowing every direction the robot might have drifted, it pushes the belief the way the command intended. The full step becomes:

$$\hat{X}_t = f(\hat{X}_{t-1}, \bar{u}_{t-1}) \cap h^{-1}(\bar{z}_t)$$

## Realistic case: non-deterministic uncertainty

So far $f$ and $h$ were assumed perfect. Reality adds two kinds of uncertainty. **Dynamics uncertainty**: from a given state and input, the robot does not land in an exactly predictable next state — wheels slip, wind pushes. **Measurement uncertainty**: at a given state, the sensor does not return an exactly predictable reading. If a distance sensor is only accurate to within $\pm\epsilon$, then instead of a clean circle the state lies **somewhere in a ring (annulus)** between radii $r - \epsilon$ and $r + \epsilon$.

These bounded-but-unknown error models — sets of possibilities with no probabilities attached — are exactly what *non-deterministic uncertainty* means. Extending the filter to handle them is conceptually easy: the predicted set and the preimage simply get **fatter**, while the three-step predict–correct–fuse structure stays identical.

### Writing it out formally (Exercise 2)

Model each error as membership in a bounded set rather than an exact value. Dynamics gains a disturbance $\bar{w}_t$ drawn from a bounded set $\mathcal{W}$, and the measurement gains noise $\bar{v}_t$ from a bounded set $\mathcal{V}$:

$$\bar{x}_{t+1} = f(\bar{x}_t, \bar{u}_t, \bar{w}_t), \quad \bar{w}_t \in \mathcal{W}$$

$$\bar{z}_t = h(\bar{x}_t) + \bar{v}_t, \quad \bar{v}_t \in \mathcal{V}$$

The **dynamics update** now sweeps over every starting state *and* every allowed disturbance, which is what fattens it:

$$f(\hat{X}_{t-1}, \bar{u}_{t-1}) = \{\, \bar{x} \in \mathcal{X} \mid \exists\, \bar{x}_{t-1} \in \hat{X}_{t-1},\ \exists\, \bar{w} \in \mathcal{W} \text{ s.t. } f(\bar{x}_{t-1}, \bar{u}_{t-1}, \bar{w}) = \bar{x} \,\}$$

The **measurement update** keeps every state whose ideal reading could have produced $\bar{z}_t$ under some allowed noise:

$$h^{-1}(\bar{z}_t) = \{\, \bar{x} \in \mathcal{X} \mid \exists\, \bar{v} \in \mathcal{V} \text{ s.t. } h(\bar{x}) + \bar{v} = \bar{z}_t \,\}$$

For the distance-to-origin sensor with $\|\bar{v}\| \le \epsilon$, this preimage is precisely the annulus $\{\, (x,y) \mid |\sqrt{x^2 + y^2} - r| \le \epsilon \,\}$. The **fuse** step is unchanged — still an intersection:

$$\hat{X}_t = f(\hat{X}_{t-1}, \bar{u}_{t-1}) \cap h^{-1}(\bar{z}_t)$$

> **A known input does not collapse the prediction to a point.** With dynamics uncertainty, even a perfectly known $\bar{u}_{t-1}$ leaves the predicted set spread out, because the disturbance $\bar{w}$ ranges over all of $\mathcal{W}$. Knowing the input reduces the spread; it does not eliminate it.

## Why it is not used directly, and what comes next

The lecture is candid that this filter is a conceptual foundation rather than a practical tool. Three reasons: representing and intersecting arbitrary sets $f(\hat{X}_{t-1})$ and $h^{-1}(\bar{z}_t)$ is **computationally hard**; specifying a non-deterministic uncertainty model precisely is **awkward**; and the filter is **very conservative**, tracking every possible state and reasoning about worst cases without any sense of which states are more likely. That last point is the deepest limitation — a worst-case set says *where the robot might be*, never *where it probably is*.

The fix is to swap sets for **probability distributions** and intersection for **Bayes' rule**. The same three stages carry over — dynamics update, measurement update, fuse — which is why the next lecture opens with a probability review, with Bayes' rule as the central tool.
