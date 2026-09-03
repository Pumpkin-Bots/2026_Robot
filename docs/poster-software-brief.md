# Team 8793 — Software Content Brief for the 2026 Design Poster

**What this document is:** source material for the *Software* portions of a 24×36" design poster.
Everything here was read directly out of Team 8793's 2026 robot code (Java 17, WPILib command-based,
Phoenix 6, PathPlanner, PhotonVision) on 2026-08-30. Numbers, equations, and behavior described
below are what the code actually does — not aspirations.

**Who reads the poster:** outreach visitors (non-technical), other teams in the pits (technical,
skimming), and an MIT maker-portfolio reviewer (technical, will read closely and will notice
overclaiming). Write for all three by layering: headline → one-sentence hook → technical detail.

**How to use this doc:** the ✍️ blocks are poster-ready copy you can lift verbatim or trim. The
prose around them is background so you can rewrite safely without inventing anything. The
"Accuracy guardrails" section at the end lists things you must *not* claim.

The poster spec calls for three software bullets — physics-based shooting, dashboard shot
calibration, and auto-unjam. Section 4 (localization software) is included because the poster's
"Robot Localization" section makes software claims (Kalman filter, sensor mixing) that need
accurate backing copy.

---

## 1. Physics-Based Shot Calculation

This is the flagship software item. Give it the most poster real estate of anything in the
software column — it is the piece with real math to show.

### The one-paragraph version

✍️ **Most teams shoot from a lookup table: drive to a known distance, measure the flywheel speed
that works, and interpolate between measurements. We solve the projectile motion problem in closed
form instead, every 20 milliseconds. The robot knows where it is on the field, where the goal is,
how fast it's moving, and how hard the feeder shoves the ball — and it computes the turret angle,
hood angle, and flywheel speed from physics rather than from a table of guesses. The payoff is that
it works at distances we've never tested from, and it works while driving.**

### The math, step by step

The solver lives in `AimSolver.java` as a pure function — no motors, no network tables, no state.
That makes it unit-testable off-robot and replayable against logged data to explain a miss after
the fact.

**Step 1 — Pick the launch angle.**
Given horizontal distance *d* to the goal and height difference *Δz*, the *minimum-energy* launch
angle (the angle that reaches the target with the least possible speed) is:

$$\theta_{\min} = \frac{\pi}{4} + \frac{1}{2}\arctan\!\left(\frac{\Delta z}{d}\right)$$

We deliberately do **not** shoot at that angle. A minimum-energy shot arrives exactly at the apex
of its arc, which at close range means the ball can reach the goal while still travelling
*upward* and skim the rim. So we bias steeper by a fixed **descent margin of 12°**, which
guarantees the ball is descending when it arrives:

$$\theta = \text{clamp}\big(\theta_{\min} + 12^\circ,\ \theta_{\text{flattest}},\ \theta_{\text{steepest}}\big)$$

The clamp is the hood's real range: rack angle 15°–42°, and `launch_angle = 90° − rack_angle`, so
the reachable launch window is 48°–75°.

**Step 2 — Solve for launch speed.**
Rearranging the standard no-drag trajectory equation

$$\Delta z = d\tan\theta - \frac{g\,d^{2}}{2v^{2}\cos^{2}\theta}$$

for *v* gives a closed-form answer — no iteration, no search:

$$v = \sqrt{\frac{g\,d^{2}}{2\cos^{2}\theta\,(d\tan\theta - \Delta z)}}$$

The denominator has physical meaning: it's how far the ball's straight-line aim clears the target
by. If it isn't positive, the shot is climbing too steeply to ever get there and *no* speed fixes
it — the solver detects that case and flags the shot infeasible instead of commanding nonsense.

**Step 3 — Correct for air drag empirically.**
A real ball loses energy to air the whole flight, so the no-drag solve always under-asks, and the
shortfall grows with range. Two calibration terms absorb it:

$$v' = v \cdot k_{\text{scalar}} + k_{\text{perMeter}} \cdot d$$

The flat scalar fixes "every shot is short by the same fraction"; the per-meter term (currently
**0.35 m/s per meter**) fixes "close shots land but long shots fall short," which is drag's
signature.

**Step 4 — Shoot on the move, by vector subtraction.**
This is the part worth bragging about, because the usual approach is worse. Most implementations
guess a flight time, aim at a "virtual target" offset by robot velocity × flight time, then iterate
because moving the target changed the flight time.

We don't iterate. We express the stationary solution as the field-relative velocity vector the ball
must end up with:

$$\vec{v}_{\text{ball}} = \big(v'\cos\theta\cos\beta,\ \ v'\cos\theta\sin\beta,\ \ v'\sin\theta\big)$$

(β = bearing to target), and then subtract every velocity the ball is going to receive **for free**:

$$\vec{v}_{\text{shooter}} = \vec{v}_{\text{ball}} - \vec{v}_{\text{launch point}} - \vec{v}_{\text{feeder}}$$

Whatever remains is what the shooter itself must impart. This is *exact, not an approximation*: the
ball's resulting field-relative velocity is identical to the stationary solution, so it flies the
identical trajectory and lands in the identical place. Because the corrections are subtractions on
one vector, they compose exactly and in any order.

**Step 4a — The launch point isn't the robot center.**
The ball leaves from a point offset from the chassis center, so when the robot *rotates*, that point
swings through an arc and picks up ground speed the chassis itself doesn't have:

$$\vec{v}_{\text{launch point}} = \vec{v}_{\text{center}} + \vec{\omega} \times \vec{r}$$

Aiming against chassis velocity instead would throw every shot fired while turning off to one side.

**Step 4b — The feeder shoves the ball.**
The feed wheel imparts velocity the shooter never asked for, along the robot's fore/aft axis, and
which way it acts depends on where the turret is pointing:

$$p(\phi) = F\cos^{2}\phi - B\sin^{2}\phi$$

With the turret forward or backward (φ = 0° or 180°) the shove runs down the barrel and the ball
leaves fast. At φ = ±90° that help vanishes and the ball instead exits carrying velocity toward the
robot's rear. The **sin²** (not sin) is what makes the rearward term symmetric — the same push at
+90° and −90°, which is how the real effect behaves. Because *p* depends on the turret angle we're
solving *for*, this one term is iterated — but only **3 passes**, and it's converged after 2: the
push is well under 1 m/s against a launch speed around 15 m/s.

**Step 5 — Convert back to motor commands.**

$$\text{turret} = \arctan2(v_y, v_x) - \psi_{\text{robot}} + \text{offset}$$
$$\text{rack} = 90° - \arctan2\!\left(v_z, \sqrt{v_x^2+v_y^2}\right) + \text{offset}$$
$$\text{flywheel RPS} = \frac{|\vec{v}|}{\pi D_{\text{eff}}} + \text{offset}, \qquad D_{\text{eff}} = 0.0762\ \text{m}$$

Effective diameter is 3", the average of the 4" bottom and 2" top opposed flywheels.

**Step 6 — Report whether the shot is even possible.**
Every solution carries flags: `rackClamped` (physics wanted an angle the hood can't reach),
`speedClamped` (wanted more than 70 RPS), and `feasible`. `achievable() = feasible && !clamped`.
This is what lets the driver know a shot is out of range *before* pulling the trigger, rather than
watching the ball fall short.

### Two supporting details worth a callout box

- **The turret counter-rotates to stay field-locked.** When the chassis spins at ω rad/s, the
  turret must spin at −ω just to keep pointing at the same spot on the field. Rather than let the
  PID chase that error after it appears, we feed −ω forward through the position controller's
  velocity term, so the correction is applied *before* error accumulates.
- **500° of turret travel, and the software knows it.** The commanded angle is unwrapped relative
  to the last command so the turret tracks smoothly across the ±180° `atan2` discontinuity instead
  of spinning a full circle; if unwrapping would exceed a soft limit, it wraps 360° to the other
  side of the range. This is what makes the custom 520°-capable chain track actually usable.

### Honest framing (read before writing this section)

The codebase contains **both** aiming paths: the physics solver (`calculatePhysicsShooterActions`,
used by the primary shooting and shuttling modes) and an older empirical lookup-table path
(`calculateShooterActions`, distance → rack angle / flywheel RPS interpolation, still used by the
manual jam-clear mode). Correct framing is "we moved from lookup tables to a closed-form physics
solver, and kept the tables as a fallback," not "we never used tables."

---

## 2. Live Shot Calibration on the Dashboard

### The one-paragraph version

✍️ **Physics gets you a trajectory; it doesn't tell you that the hood reads 20° when it's really at
22°, or exactly how much energy the ball loses to air. So every calibration constant in the aiming
math is exposed as a live-editable value on the driver dashboard. We tune between shots by typing a
number, instead of burning an edit-compile-redeploy cycle on every guess — and a master switch means
a value someone fat-fingered in the pits can never follow the robot into a match.**

### What's actually tunable (14 values, all under `Tuning/Shooter/`)

| Knob | What it fixes |
|---|---|
| `LaunchHeightM`, `LaunchForwardOffsetM`, `LaunchLeftOffsetM` | Where the ball actually leaves the robot. Measured, not guessed — everything downstream is built on these. |
| `TurretOffsetDeg` | Consistent left/right bias (mechanical zero ≠ encoder zero) |
| `RackOffsetDeg` | Same, for the hood |
| `SpeedScalar` | Every shot short by the same *fraction* |
| `SpeedPerMeterMps` | Close shots good, long shots short → that's drag |
| `DescentMarginDeg` | Shots skimming the rim on the way up |
| `FlywheelRpsOffset` | Small constant bias, e.g. ball compression at the exit roller |
| `FlywheelDiameterM` | Effective wheel diameter for the speed→RPS conversion |
| `GravityMps2` | 9.80665 on the real robot; 11.0 in simulation (see below) |
| `ShootOnTheMoveGain` | 0 → aim as if stopped, 1 → fully compensate. Set to 0 to isolate a stationary aiming problem from a motion-compensation problem. |
| `FeederForwardPushMps`, `FeederBackwardPushAt90Mps` | The two feeder-shove terms from §1 |

Turret PID gains (kP/kI/kD/kV) are live-tunable the same way, and re-applied to the motor
controller only when a value actually changes.

### The three design decisions worth showing

1. **A master switch, defaulting to OFF.** `Tuning/TuningModeEnabled` decides whether tunables read
   the dashboard or the compiled-in constant. Off means whatever is typed on a laptop is ignored
   *entirely*. Flipping it on is one click, and the switch sitting visibly on the dashboard is the
   reminder that the robot isn't running the numbers in source control. There's also a momentary
   `ResetToDefaults` button that rewrites every tunable back to its compiled value when a tuning
   session wanders somewhere bad.
2. **One atomic snapshot per solve.** All 14 values are read *once* at the top of each solve, so a
   number changed mid-loop can't produce an answer computed half from old values and half from new.
3. **A documented tuning order.** The constants file carries a numbered procedure — measure launch
   geometry → turret offset → speed scalar → per-meter drag → arc shape → feeder push → *only then*
   start driving and tune shoot-on-the-move. One variable at a time, robot stationary until the last
   step. Once a value is proven it gets copied back into source so it survives a reboot.

### Telemetry is designed for debugging, not decoration

When a shot misses, the first question is *why*, and the dashboard is built to answer it. Alongside
the commanded turret/rack/flywheel values, the robot publishes the **error** between commanded and
actual for all three — because "the solution was right but the hood was still travelling" looks
nothing like "the solution was wrong." The velocity filter publishes every *input* too (fused vs.
wheel vs. vision velocity side by side), so you can tell in one plot whether bad aim came from the
ballistics or from a bad velocity estimate.

---

## 3. Automatic Jam Recovery

### The one-paragraph version

✍️ **A ball wedged in the intake used to mean a driver noticing, reaching for a button, and holding
it — several seconds of a match spent on something the robot can detect about itself. Now the intake
watches its own motor: high current plus no rotation means something is stuck. The robot reverses
the entire intake path at full speed for half a second, then goes straight back to intaking. The
flywheel never spins down, so the robot is ready to shoot the instant the jam clears — and the
driver frequently never knows it happened.**

### How the detection works

A jam is diagnosed from two conditions that must hold **simultaneously**, for **0.25 seconds
continuously**:

| Condition | Threshold | Why alone it isn't enough |
|---|---|---|
| Roller stator current | **> 55 A** | High current alone is just a hard-working intake pulling a lot of balls |
| Roller motor speed | **< 10 rotations/sec** | Low speed alone is just a stopped intake |

For scale: the roller's stator current limit is 70 A, and its free speed while intaking is roughly
75 rotations/sec — so a genuine jam pins the current at the limit and drops the speed by ~90%. The
0.25 s hold requirement exists because a motor *spinning up* also momentarily shows high current at
low speed; without it, every time the intake started would look like a jam.

### The recovery

Roller and both horizontal indexers reverse at **100% output for 0.5 seconds**, then resume
intaking automatically. The turret feeder reverses with them, so the throat above the intake clears
too. The shooter flywheel is untouched by the whole sequence and holds its commanded speed
throughout. If the ball is *still* wedged when the burst ends, the detector re-arms and fires
another burst a fraction of a second later, so it retries until clear.

### Details that make it a good engineering story

- **Zero new hardware.** No jam sensor, no beam break, no extra wiring. The detection reuses
  telemetry the motor controller already reports over CAN.
- **We had to explicitly ask for the data.** The code aggressively silences unused CAN status
  frames to keep bus utilization low. Current and velocity had to be re-enabled *by name* on the
  roller before that optimization ran — otherwise the jam detector would have quietly read stale
  zeros forever.
- **The state is inspectable.** Roller current, roller velocity, the stall flag, and an "unjamming
  now" flag are all published to the dashboard, so the thresholds can be tuned by watching real
  numbers during a real jam rather than by guessing.
- **It's a state machine, not a reflex.** The unjam owns the intake for its full 0.5 s regardless of
  what the sensors do mid-burst, and the detector's history is cleared afterward so it earns a fresh
  full detection window instead of instantly re-triggering on the jam it already handled.

---

## 4. Localization Software (backing copy for the "Robot Localization" poster section)

The poster's localization bullets are mostly hardware (4 cameras, 2 Orange Pis, 70° FOV, mounting).
These are the software claims behind "uses a Kalman filter to use a calibrated mix of cameras,
drivetrain, and Pigeon 2.0."

### Vision → pose

Each of the 4 cameras is polled independently. **Multi-tag estimation is preferred** — solving one
pose against several tags at once is far better conditioned than trusting a single tag — with a
fallback to the lowest-ambiguity single-tag solve when only one tag is visible.

Every candidate pose then has to survive validation before it's allowed to touch the robot's
position: single-tag estimates with pose ambiguity above 0.2 are rejected, poses landing more than
0.5 m outside the field are rejected, tags farther than 6 m are rejected, and eight specific tag IDs
(tower-back and outpost tags) are permanently ignored because they're prone to producing plausible
but wrong poses.

Surviving estimates aren't trusted equally. Each one is assigned a **standard deviation** — a
formal statement of how much to believe it — that scales with both tag count and distance:

$$\sigma = \sigma_{\text{base}} \cdot \left(1 + \frac{d^{2}}{30}\right)$$

with σ_base four times looser for single-tag than multi-tag estimates. The WPILib pose estimator is
a Kalman filter: it weights each measurement against the odometry prediction *by these standard
deviations*, so a two-tag reading from 2 m away moves the robot's belief a great deal and a
one-tag reading from 5.5 m barely nudges it. Measurements are submitted with the timestamp of the
frame they came from, so latency is accounted for rather than ignored.

### The velocity filter (the part shoot-on-the-move depends on)

Aiming while driving needs to know how fast the robot is going *right now*, and the obvious source —
wheel odometry — is least trustworthy at exactly the moment that matters: a direction change, or a
wheel slip. So velocity is a **complementary filter over three sources chosen for opposite failure
modes**:

- **Pigeon 2 accelerometer (fast, primary).** Integrated forward every loop. Responds instantly to a
  direction change and keeps telling the truth while wheels slip. Weakness: integrating a slightly
  biased acceleration drifts over a long straight.
- **Wheel odometry (medium).** Folded back as a first-order correction with a 0.15 s time constant.
  This is the term that *bounds* the IMU's drift.
- **Vision-fused pose (slow).** Differentiated over a 0.25 s window: noisy, but *unbiased*. Applied
  very weakly, it catches systematic wheel error — wrong wheel radius, carpet scrub — that wheel
  odometry cannot detect about itself. A pose jump differentiates into a fictional velocity spike,
  so samples disagreeing with the estimate by more than 2 m/s are discarded.

Net effect: the fast content of the estimate comes from the gyro, the slow content from wheels and
cameras.

Two refinements a technical reader will appreciate:

- **The bias is learned, not fought.** A persistent residual between the integrated IMU velocity and
  wheel odometry is exactly what a constant accelerometer offset looks like, so that residual is
  integrated into a per-axis bias estimate and subtracted from future readings — in the *robot*
  frame, since that's where the physical sensor offset actually lives.
- **The Pigeon isn't at the robot's center, and that matters enormously.** Mounting position was
  irrelevant when the Pigeon was only a gyro (yaw rate is identical everywhere on a rigid body) but
  became critical the moment its accelerometer was used: spinning in place at 8 rad/s with the
  Pigeon 0.29 m off-center produces ~13 m/s² of pure centripetal acceleration that the robot's
  center never experiences. Integrate that and the estimate is garbage the moment the robot turns.
  The filter recovers the center's acceleration with rigid-body kinematics,
  $\vec{a}_{\text{pigeon}} = \vec{a}_{\text{center}} + \vec{\alpha}\times\vec{r} + \vec{\omega}\times(\vec{\omega}\times\vec{r})$,
  subtracting both the centripetal and tangential terms.

---

## 5. Optional: Simulation (strong material if there's space)

Not in the poster outline, but it's genuinely impressive and photographs well:

✍️ **The whole robot runs in physics simulation before it ever touches the field. Simulated
AprilTag cameras with realistic noise and latency feed the real localization code, simulated fuel
gets collected by a collision model of the intake, and shots are launched as real projectiles with
real ballistics — so aiming logic, mode switching, and autonomous routines all get debugged on a
laptop.**

One detail that makes a nice aside: the simulator's projectiles fall under a flat 11 m/s² instead of
9.81 as its own stand-in for air drag, so the aiming code deliberately uses a *different* gravity
constant in simulation than on the real robot — otherwise the predicted trajectory wouldn't match
the ball the simulator draws.

---

## 6. Suggested figures and photos for the software column

Software is hard to photograph. These are the shots worth staging — listed in priority order.

1. **Ballistic arc diagram (vector illustration, not a photo).** Side view: robot, launch point at
   0.48 m, arc to the goal. Annotate θ_min, the +12° descent margin, and show the ball *descending*
   into the goal. This is the single most valuable graphic in the software section.
2. **Shoot-on-the-move vector diagram (vector illustration).** Top view: three arrows — the required
   ball velocity, the robot's velocity, and the difference the shooter actually fires. One picture
   explains the entire "subtract, don't iterate" idea.
3. **Dashboard screenshot during tuning.** Elastic/Shuffleboard with the `Tuning/Shooter/` values
   and the aiming telemetry visible. Crop tight; make sure the tuning-mode switch is in frame.
4. **AdvantageScope 3D view.** Robot model with the turret rotated and hood raised to a live firing
   solution, ideally beside the real robot in the same pose. A "digital twin" side-by-side is a
   strong portfolio image.
5. **Jam-recovery current plot.** Deliberately jam the intake with logging on and export the roller
   current + velocity traces. Annotate: normal intaking → the 55 A / 10 RPS crossing → the 0.25 s
   detection window → the 0.5 s reverse burst → back to normal. This turns an invisible feature into
   a visible one.
6. **Simulation screenshot.** Field view with simulated fuel and a projectile mid-flight.
7. **Camera coverage overlay.** Top-down robot outline with four 70° FOV wedges drawn on.
8. **Team photo at the driver station**, dashboard visible on the laptop — humanizes the software
   section for the outreach audience.

Leave the photo boxes sized generously; #1 and #2 should be large enough that the annotations are
readable from ~3 feet away, since those are what a technical reader will actually stop and study.

---

## 7. Key numbers (quick reference — all verified against the code)

| Quantity | Value |
|---|---|
| Control loop period | 20 ms (50 Hz) |
| Descent margin above minimum-energy angle | 12° |
| Hood (rack) angle range | 15°–42° → launch angle 48°–75° |
| Turret travel | ~500° effective (−200° to +300°) |
| Flywheel max commanded speed | 70 rotations/sec |
| Effective flywheel diameter (4" + 2" opposed) | 0.0762 m (3") |
| Ball launch height | 0.4826 m |
| Drag compensation | 0.35 m/s of extra launch speed per meter of range |
| Feeder-solve iterations | 3 (converged after 2) |
| Jam detection | >55 A stator **and** <10 RPS, held 0.25 s |
| Jam recovery | 100% reverse, 0.5 s, flywheel unaffected |
| Vision: max usable tag distance | 6 m |
| Vision: max single-tag ambiguity | 0.2 |
| Vision std-dev distance scaling | σ_base × (1 + d²/30) |
| Velocity filter: wheel-correction time constant | 0.15 s |
| Velocity filter: vision-correction time constant | 0.75 s |
| Cameras | 4 PhotonVision, multi-tag preferred |

---

## 8. Accuracy guardrails — do not claim these

The MIT-portfolio audience is the reason this section exists. Everything above is true; these
specific overstatements are not, and would be caught.

- **Don't say the auto-unjam is competition-proven.** It is implemented, compiles, and is wired into
  both intaking modes, but as of 2026-08-30 the 55 A / 10 RPS thresholds are engineering estimates
  chosen from the motor's 70 A current limit and free speed. They have not yet been validated
  against a real jam on the real robot. "Designed to be tuned from logged current traces" is
  accurate and honest; "tested and tuned in competition" is not.
- **Don't claim the lookup tables were replaced entirely.** Both aiming paths exist in the code.
- **Don't claim the physics solver accounts for air drag from first principles.** It solves the
  *no-drag* trajectory and corrects for drag with two empirically-tuned terms. That's a legitimate
  and common engineering choice — describe it as such rather than implying a drag model.
- **Don't claim the shot calibration values are final.** Several (the feeder push terms, the exact
  launch-point offsets) are still marked for empirical tuning in the source. The *system* for
  tuning them is the accomplishment; present it that way.
- **Don't call the velocity filter a Kalman filter.** The *pose* estimator is a Kalman filter
  (WPILib's). The velocity estimator is a complementary filter with learned bias — a different and
  correctly-named thing. Mixing them up is exactly the kind of error a reviewer notices.
- **Backspin.** The poster's mechanical section credits the opposed 4"/2" flywheels with imparting
  backspin. True mechanically — but note the software's aiming math does **not** model the Magnus
  effect from that spin; its aerodynamic effects are absorbed into the empirical speed calibration.
  Don't imply the solver accounts for spin.

---

## 9. Suggested layout weighting for the software column

If the software section gets roughly one column of a 24×36" poster:

- **~50%** — physics-based shot calculation (the math, the arc diagram, the vector-subtraction
  diagram). This is the differentiator; give it the equations, not just prose.
- **~20%** — auto-unjam (short, punchy, with the current plot). It's the most *legible* feature to a
  non-technical outreach visitor: "the robot notices it's stuck and fixes itself."
- **~15%** — dashboard calibration (screenshot + the "master switch defaults to off" safety point,
  which reads as maturity to a technical reviewer).
- **~15%** — localization/velocity-filter backing copy, likely merged into the poster's existing
  Robot Localization section rather than standing alone.

For the outreach audience specifically: lead each software block with the plain-language hook (the
✍️ paragraphs), and let the equations sit below as visual texture that signals depth without
requiring anyone to read them.
