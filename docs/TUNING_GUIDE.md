# Shooter Tuning Guide — Do It In This Order

Team 8793, 2026 robot. Everything you have to tune on the shooter and the power system, in the order
to do it, with a test that tells you when each stage is finished. Appendix D is the full knob index
if you just need to look one up.

The order is not a suggestion. Every stage assumes the ones above it are correct, and tuning out of
order means chasing two unknowns at once — which is how a whole practice day disappears.

---

## Read this first

### Two things that will waste your afternoon if you don't know them

**1. Tunables do nothing until `Tuning/TuningModeEnabled` is ON.** With it off, every
`Tuning/...` value returns the constant compiled into `Constants.java` and the dashboard is ignored
entirely. This is deliberate — a number someone fat-fingered must never follow the robot into a
match — but it means your first ten minutes can vanish wondering why nothing changed.

**2. Dashboard values are lost on roboRIO reboot.** Once a number is proven, copy it into
`Constants.java`. `Tuning/ResetToDefaults` is a momentary button that puts every tunable back to its
compiled value when a session has wandered somewhere bad.

### The lookup tables are not used for shooting

`flywheelRPSTable`, `rackAngleTable`, and `flightTimeTable` in `ShooterSubsystem` are reached only
by `calculateShooterActions`, which is called only by **JamMode** — where the robot is reversing its
rollers to clear a jam and aiming is irrelevant.

Every real shot — `ShooterMode` and `ShuttleMode` — goes through the **physics solver**
(`AimSolver`). So you do **not** re-take those tables. You calibrate the physics instead: stage 3
below. (An earlier note of mine said the tables needed re-taking after the gearing change. That was
wrong — they don't affect a single scored shot.)

### What "done" means at each stage

Each stage below has an **exit test**. Don't move on until it passes. If a later stage misbehaves,
the first thing to do is re-run the exit test of the stage above it.

---

## Stage 0 — Measure. Don't tune.

These are physical facts, not knobs. Getting them wrong means every stage below is fitting
calibration factors to compensate for a bad measurement, and those factors won't generalise.

| What | Where it goes | How |
|---|---|---|
| Ball launch height, forward offset, lateral offset | `Tuning/Shooter/LaunchHeightM`, `LaunchForwardOffsetM`, `LaunchLeftOffsetM` | Tape measure or CAD, to the point the ball actually leaves. **Everything downstream is built on these.** |
| Flywheel inertia about the 4" shaft | `FLYWHEEL_MOI_KG_M2` (source) | CAD → Mass Properties → inertia about the spin axis, for everything that spins with it. Sets `kA` *and* the droop cold-start seed. |
| Ball mass | `FUEL_MASS_KG` (source) | Kitchen scale. |
| Gyro pitch/roll on flat carpet | `Tuning/Shooter/TiltRollOffsetDeg`, `TiltPitchOffsetDeg` | Park on flat carpet, read `Shooter/Tilt/RollDeg` and `PitchDeg`, enter what they say. They should be zero and generally aren't. |
| Turret mechanical zero | `zeroTurretEncoderOnce()` (source) | Turret physically forward at enable. |

**Do not touch `Tuning/Shooter/FlywheelDiameterM`.** It is *derived* from the two gear ratios and
the two wheel diameters — it's the single point where motor RPS becomes ball speed. Overriding it
silently breaks the relationship between the gearing and the shot.

**Exit test:** `Shooter/Tilt/RollDeg` and `PitchDeg` read ~0 on flat carpet after the offsets are
entered. Drive one wheel onto a block and confirm they move the way the robot physically did.

---

## Stage 1 — The three mechanism loops

**Why first:** a solution that was correct but never *reached* looks nothing like a solution that
was wrong, and from the ball's point of view they're identical. If the rack is still travelling or
the flywheel hasn't spun up when the ball goes through, no amount of aiming calibration will help.

All three are live-tunable and all three snap back to compiled constants when tuning mode goes off.

### 1a. Turret — `Tuning/Turret/kP, kI, kD, kV, kS, kFF`

Watch `Shooter/Physics/TurretErrorDeg`.

- `kP` up until it tracks crisply without hunting. Current 4.0; 12 was optimal but too violent for
  the wiring chain.
- `kV` is the counter-rotation feedforward — the turret must spin at −ω to stay field-locked while
  the robot turns. Raise until the turret holds its aim through a spin.
- `kS` breaks static friction symmetrically.
- `kFF` is **one-directional**, for the direction the wiring chain spools against. Sign picks the
  direction, magnitude is volts. Only that direction gets help.

**Exit test:** spin the robot in place at a decent rate with the turret commanded at a fixed field
point. `TurretErrorDeg` should stay inside a couple of degrees.

### 1b. Rack — `Tuning/Rack/kP, kI, kD`

Watch `Shooter/Physics/RackErrorDeg`.

The rack carries the launch angle, so a loop that quietly undershoots reads as a bad physics
calibration and will send you to stage 3 chasing a mechanism problem.

**Exit test:** command a large rack move; `RackErrorDeg` settles to under ~0.5° and stays there.

### 1c. Flywheel — `Tuning/Flywheel/kP, kI, kD, kV, kS`

Watch `Shooter/Flywheel/ErrorRps`.

- `kV` ≈ 0.125 (12 V / 100 RPS free speed) should already be close.
- `kP` up until recovery from a disturbance is fast without the wheel humming. **This is worth real
  effort** — faster recovery means a shallower trough on the next ball, which is strictly better
  than compensating for a deep one.
- `kI` closes the standing error under sustained fire. Watch for windup on shots the motor can't
  reach.
- `kA` is **not** a knob. It's computed from `FLYWHEEL_MOI_KG_M2`. If you want to change it, measure
  the inertia properly.

**Exit test:** commanded at a fixed speed with no balls, `ErrorRps` sits within ±1 and
`Shooter/Flywheel/AtSpeed` is true.

---

## Stage 2 — Droop detection

**Why before compensation:** the compensator can only learn from balls it correctly counts. If
detection is wrong, it learns confidently wrong numbers — worse than learning nothing.

Set `Tuning/Flywheel/CompensationGain` = **0** for this stage. You're only checking the detector.

Fire **exactly 10 balls one at a time**, then **10 in a continuous burst**. Watch
`Shooter/Flywheel/ShotCount`.

| Reads | Meaning | Fix |
|---|---|---|
| 10, then 10 | Correct | Move to stage 3 |
| 10, then ~4 | Burst balls merging into one | Lower `ShotReboundRps` |
| 14, then 18 | Noise splitting one ball into several | Raise `ShotReboundRps`, then `ShotDetectDropRps` |
| 6, then 6 | Dips not registering at all | Lower `ShotDetectDropRps` |

Then sanity-check the physics the detector is reporting:

- `Shooter/Flywheel/LastContactSec` — should read **10–25 ms**. If it reads 100 ms+, the detector is
  tracking something that isn't a ball. Go back and fix detection.
- `Shooter/Flywheel/RejectedCount` — a high ratio to `ShotCount` means a threshold is wrong.

Before trusting any of the above, check the sampler is actually seeing the whole dip:

- `Shooter/Flywheel/Sampler/Hz` — should sit at `TargetHz` (200) whenever the robot is on. This is
  *distinct* frames per second, not callbacks per second.
- `Shooter/Flywheel/Sampler/Gaps` — frames that were published and never read. Should stay flat.
  Any climb means the trough is being measured off an incomplete picture, and every number below it
  is suspect.
- `Shooter/Flywheel/Sampler/Timeouts` — flat on a real robot. Climbing means the velocity signal
  isn't being published near 200 Hz, so look at the signal's update frequency, not the detector.
  (In simulation this is the normal path and will climb — the signals come from the sim loop rather
  than from a bus.)
- `Shooter/Flywheel/Sampler/Faults` — any value above zero is a bug. The sampler caught an exception
  and kept going rather than dying silently.

Those counters are cumulative since boot, so a total from an hour ago tells you nothing about now.
**`Shooter/Flywheel/Sampler/ResetStats`** (momentary toggle, always live) zeroes all four so you can
watch one clean window — e.g. reset, fire a batch, and confirm `Gaps` is still zero afterwards.

> Two different resets, don't confuse them. `Sampler/ResetStats` clears **diagnostics only** and
> touches nothing the robot has learned. **`Shooter/Flywheel/ClearLearned`** is the one that clears
> the collected droop data itself — the live curve *and* its saved file. Use that one after changing
> a wheel or a belt; see stage 4.

**Exit test:** `Hz` at 200 with `Gaps` flat, both counts correct, contact time plausible, rejections
rare.

> **Don't lower the sample rates.** The velocity signal publishes at 200 Hz and the detector consumes
> every frame of it, exactly once each, on its own real-time thread. A ball's contact is 10–25 ms, so
> the trough is only 2–5 frames wide: at the 50 Hz main loop rate it is one sample wide and usually
> missed entirely, and dropping even one or two frames *moves* the apparent minimum rather than just
> blurring it — which makes the detector learn a confidently wrong number instead of no number.
>
> Note that raising `setUpdateFrequency` alone would not have fixed a sampling problem, and lowering
> it is not the only way to create one. Phoenix caches one frame per signal with no queue, so what
> matters is that something is there to read each frame as it lands. That is why the sampler blocks
> on `BaseStatusSignal.waitForAll` — woken by the data — instead of polling on a timer, where two
> independent clocks beat against each other and a main-loop overrun silently eats frames.

---

## Stage 3 — Stationary shot calibration, compensation OFF

**Why compensation off:** the compensator adds speed on top of whatever the solver asks for. Tuning
both at once is two unknowns and no way to separate them.

Keep `CompensationGain` = 0. **Robot stationary. Turret at 0°.** One knob at a time.

Order matters here too — each of these fixes a different *signature* of miss:

| Symptom | Knob | Notes |
|---|---|---|
| Consistent left/right bias | `Tuning/Shooter/TurretOffsetDeg` | Pure command offset. Fixes "reads 20° but is physically at 22°". Positive = CCW. |
| Every shot short (or long) by a similar **fraction** | `Tuning/Shooter/SpeedScalar` | Flat multiplier on required launch speed. **Start here.** |
| Close shots good, long shots fall short | `Tuning/Shooter/SpeedPerMeterMps` | Extra m/s per metre of range. That signature is air drag. |
| Small constant bias at all ranges | `Tuning/Shooter/FlywheelRpsOffset` | Motor-RPS trim after the speed conversion. Use this rather than distorting the diameter. |
| Shots skim the rim on the way **up** | `Tuning/Shooter/DescentMarginDeg` | How much steeper than minimum-energy to aim, so the ball arrives descending. Raise if rimming, lower if dropping short and steep. |
| Arc shape wrong generally | `Tuning/Shooter/RackOffsetDeg` | Mechanical zero correction. Positive = flatter. |

Watch while you do this:

- `Shooter/Physics/Feasible` and `Achievable` — if either is false the solver is telling you the shot
  can't be made from here; calibration won't fix that.
- `Shooter/Physics/SpeedClamped` — the solution wants more than `FLYWHEEL_MAX_REV_PER_SEC`.
- `Shooter/Physics/RackClamped` — the rack is against a stop.

**Exit test:** from three distances across your real shooting range, shots land centred with
compensation still off. They will be slightly **short** — that's the droop you're about to
compensate, and it should get worse at longer range.

### About the 28:18 reduction

Ball speed per motor rotation dropped ~43% when the gearing changed, so the solver now asks for
proportionally more motor RPS than it used to. Consequence: **long shuttle passes may be
unreachable** — the old worst case wanted 69 motor RPS, the same shot now needs ~122, past the
Kraken's 100 RPS free speed. The solver reports this rather than hiding it. Check
`Shooter/Physics/SpeedClamped` before trusting a long pass. Hub shots are unaffected.

---

## Stage 4 — Droop compensation ON

Now set `Tuning/Flywheel/CompensationGain` = **1.0**. This is the physically correct value: the ball
leaves at the wheel's speed *at separation*, so pre-biasing by the full learned deficit puts
separation on target.

### Fill the bins across your speed range

This is the step people skip. The deficit is learned into 10-RPS bins over commanded speed, because
the droop is badly non-linear — the motor's ability to push back during contact collapses as it
nears free speed.

**Fire from at least two clearly different distances.** With one populated bin all the algorithm can
do is scale a single measurement with speed. With two or more it learns the actual curve *and* can
carry that slope past the range you've shot from.

Watch:
- `Shooter/Flywheel/PopulatedBins` — climbing. 1 is thin, 3+ is good.
- `Shooter/Flywheel/LearnedDeficitHereRps` — should be meaningfully larger at long range than short.
  If it isn't, either the bins aren't filling or your speed spread is narrower than you think.
- `Shooter/Flywheel/LastTroughDeficitRps` — what's being learned.
- `Shooter/Flywheel/LastPerBallDropRps` — one ball's own contribution. In a burst this stays roughly
  constant while the trough deficit accumulates.

### Knobs

| Key | Default | When |
|---|---|---|
| `CompensationGain` | 1.0 | Shots now going **long** → back off. Still short → check `Saturated` first. |
| `LearningRate` | 0.20 | Up toward 0.4 to adapt faster within a match; down toward 0.1 if the estimate looks jumpy. |
| `MaxCompensationRps` | 10.0 | Hard ceiling. Leave alone unless you have a reason. |

### If shots are still short

Check `Shooter/Flywheel/Saturated` and `DutyCycleAvg` **before** touching anything else.

`Saturated` means the motor is flat out *continuously* — filtered duty ≥ 0.98 for ~1.6 s — so it
can't reach the speed it's already being asked for. Learning freezes there, deliberately, because
more setpoint produces no more speed. **That's a gearing or battery answer, not a tuning one.** A
brief pin during recovery from each ball is normal and does not count.

**Exit test:** shots from your full range land centred, and a burst of 10 doesn't walk progressively
short down the burst.

### The bins survive a reboot

What you fill in this stage is saved to `/home/lvuser/flywheel-droop.json` and loaded back at the
next boot, so a practice session's worth of balls is still there after the power cycle between
matches — and after a code deploy, since that directory isn't the deploy directory.

- `Shooter/Flywheel/Store/Restored` — true if this boot started from a saved curve rather than the
  physics seed. **Check this before the first match of the day.**
- `Shooter/Flywheel/Store/Status` — one line saying what happened: `loaded 47 balls`, `no saved
  file`, `different shooter geometry - discarded`.
- `Shooter/Flywheel/Store/Unsaved` — there are new balls not yet on disk. Clears on disable.

Saves happen automatically ~20 s after shooting settles, and always when the robot is disabled.
Nothing is written in simulation.

**A saved curve is refused when the shooter it was measured on isn't the shooter that's running**:
change `FLYWHEEL_GEAR_RATIO`, `FLYWHEEL_EFFECTIVE_DIAMETER_METERS`, `FLYWHEEL_ROTOR_MOI_KG_M2`,
`FUEL_MASS_KG`, or `FLYWHEEL_MAX_REV_PER_SEC` and the old file is discarded on the next boot — that
is correct, and it means you re-fill the bins after a gearing change.

What the signature *can't* see is hardware: a new flywheel wheel, a retensioned belt, a different
ball batch. After any of those, flip **`Shooter/Flywheel/ClearLearned`** (a plain dashboard toggle,
live at all times, no tuning mode needed). It wipes the live curve *and* the file and puts itself
back to off; the next shots start from the physics seed.

### Watching the curve converge

The whole curve is published two ways, because the dashboards want different things:

- **Elastic** has no XY plot, so each bin also goes out under its own key —
  `Shooter/Flywheel/Bins/40-50Rps` and friends. Drag the whole `Bins/` folder onto **one Graph
  widget**: you get every bin converging live as you shoot. Flat-at-zero lines are bins you haven't
  fired into yet, which is the fastest way to see where your speed spread is thin.
- **AdvantageScope / Glass** plot arrays against arrays. `Shooter/Flywheel/Curve/BinCenterRps` vs
  `Curve/BinDeficitRps` draws the measured shape, with `Curve/BinSamples` for confidence.
  `Curve/SampleRps` vs `Curve/CompensationRps` is the more useful one for a sanity check — that's
  post-gain, post-clamp, post-extrapolation, i.e. what the robot will *actually* add at each speed,
  so you can see `MaxCompensationRps` biting and see the extrapolation running off the end of the
  bins you've filled.

---

## Stage 5 — Shooting on the move, and tilt

Only now start driving. Everything above must pass stationary first.

### 5a. Feeder push — robot stationary, one turret angle at a time

The feeder shoves the ball as it enters, so it leaves carrying velocity the shooter never asked for.
Two independent terms; tune each at the angle where the other contributes nothing:

| Key | Tune at | Symptom when wrong |
|---|---|---|
| `Tuning/Shooter/FeederForwardPushMps` | turret **0°** | Shots long/short by an amount that changes as the robot rotates |
| `Tuning/Shooter/FeederBackwardPushAt90Mps` | turret **90°** | Fine forward, consistently off out the side, mirrored left and right |

### 5b. Shoot on the move — `Tuning/Shooter/ShootOnTheMoveGain`

Set to **0** first and confirm stationary aim is still good — that isolates an aiming problem from a
motion-compensation problem. Then walk it to 1.0.

The correction is taken by the **rack**, not the flywheel: the solver picks the arc whose required
launch speed is the speed the flywheel would be held at standing in the same spot, so the flywheel
command tracks distance alone while the rack swings with the sticks. Drive at the hub and watch:

| Row | What it should do |
|---|---|
| `Shooter/Physics/MotionArcShiftDeg` | swing up with speed — this is the rack taking the correction |
| `Shooter/Physics/FlywheelMotorRPS` | stay on `Shooter/Physics/NominalFlywheelRPS` |
| `Shooter/Physics/MotionRackSaturated` | false while driving **at** the target |

Driving **away** from the target it saturates and the flywheel goes back to carrying the
correction — that is expected, not a bug. Flattening the arc is the mirror of steepening and the
geometry makes it worthless (on an 8 m shot, spending the whole descent margin recovers ~0.2 m/s),
so the solver declines the trade. See the class comment in `AimSolver` for the numbers.

`Tuning/Shooter/MotionRackMaxSwingDeg` caps how far the arc may be steepened, in degrees of launch
elevation. Default 20° is wider than the rack's whole travel, i.e. effectively off. Lower it if
moving shots start arriving late and scattered (more arc = more hang time = longer for the velocity
estimate to be wrong by) while stationary shots are still good.

> **`Shooter/MotionRackFirst` is a toggle, not a tunable** — live at all times, default **ON**. Off
> puts the whole correction back on the flywheel, which is a different thing from setting
> `ShootOnTheMoveGain` to 0: the correction still happens, just in the slower joint.

### 5c. Tilt — `Tuning/Shooter/TiltCompensationGain`

Offsets were set in stage 0. Drive one wheel onto the depot and confirm
`Shooter/Tilt/AppliedRollDeg` / `AppliedPitchDeg` move the way the robot physically did *before*
trusting a shot from there. Gain 0 takes the whole correction back out if they don't.

**Exit test:** shots land centred while translating and while turning.

---

## Stage 6 — Power protection

Independent of everything above — do it last, or in parallel on a different day.

### What it does

The roboRIO's own brownout protection has no opinion about what matters: at 6.8 V it cuts everything
at once, flywheel included, and every ball fired in the next second is wasted. `PowerBudget` watches
the same battery a long way further up and spends the margin deliberately:

| Priority | What happens |
|---|---|
| **Never cut** | Flywheel, rack, turret rotator, turret feeder |
| **Cut first** | Ground intake rollers + indexers |
| **Cut second** | Swerve drive — **teleop only**, because a silently speed-limited path follower doesn't drive its path slower, it drives a different path |

| Tier | Enters | Leaves | Intake | Drive |
|---|---|---|---|---|
| `NORMAL` | — | — | 100% | 100% |
| `REDUCED` | ≤ 9.5 V | ≥ 10.3 V | 40% | 55% |
| `CRITICAL` | ≤ 8.5 V | ≥ 9.3 V | 0% | 30% |

Entry is below exit so a tier has to be clearly left, not brushed. `CRITICAL` steps back out through
`REDUCED`, never straight to full authority.

### Procedure

1. Drive a full practice match with `Power/BrownoutProtectionEnabled` ON and tuning mode OFF. Log it.
2. Check `Power/Tier` against `Power/FilteredVolts`, and whether `Power/RioBrownedOut` ever went true.

| Observation | Fix |
|---|---|
| `Power/RioBrownedOut` ever true | `Tuning/Power/ReducedEnterVolts` UP by 0.5 — protection engaged too late |
| Tier went `REDUCED` constantly during ordinary driving | `ReducedEnterVolts` DOWN by 0.5, or `ReducedDriveScale` UP |
| Tier flickering on and off | `MinTierHoldSec` UP |
| Protection triggers on spikes that aren't real sags | `VoltageFilterTauSec` UP toward 0.15 |
| Protection reacts too late | `VoltageFilterTauSec` DOWN toward 0.04 |
| Robot feels undrivable under protection | `ReducedDriveScale` UP toward 0.75 |
| Intake stops picking up entirely under protection | `ReducedIntakeScale` UP toward 0.6 |
| Protection engages but battery still sags | `ReducedIntakeScale` DOWN, then `ReducedDriveScale` DOWN |
| A stalled roller still browns you out | `ReducedIntakeStatorAmps` DOWN |

> **`Power/BrownoutProtectionEnabled` is a toggle, not a tunable** — it is read at all times,
> regardless of tuning mode, because turning protection off has to work mid-match. Same for
> `Shooter/HoldFeedUntilAtSpeed`.

> **Interaction worth knowing:** jam detection compares roller stator current against
> `ROLLER_STALL_CURRENT_AMPS` (55 A), which a roller cannot reach once its stator limit is pulled to
> 35 A — detection would go silent exactly when jams get most likely. So while a reduced limit is in
> force the stall bar automatically drops to 80% of whatever the limit currently is. Watch
> `Intake/RollerStallThresholdAmps` for the live value.

**Exit test:** a full match with no `RioBrownedOut` and no tier flapping.

### Optional: hold fire until at speed

`Shooter/HoldFeedUntilAtSpeed` — toggle, default **OFF**.

- **Off** (shipped): the feeder runs continuously; balls go through whenever they arrive, including
  while the wheel is still recovering. Trades accuracy for rate — the droop compensator buys most of
  that accuracy back.
- **On**: the feeder waits for the flywheel to reach speed before each ball. Every shot leaves right,
  and the fire rate becomes whatever recovery allows. Watch `Shooter/Flywheel/LastContactSec` and the
  recovery between balls to see what it costs. Worth switching on when a specific shot has to land;
  switch off when the hopper needs emptying before the buzzer.

The intake keeps running either way — fuel collected while waiting stacks in the throat and goes
through the moment the wheel is back.

---

## Appendix A — What is *not* tunable, and why

| Thing | Why | Change it by |
|---|---|---|
| `kA` (flywheel) | Derived from inertia — it's physics, not a gain | Measuring `FLYWHEEL_MOI_KG_M2` |
| `FlywheelDiameterM` | Derived from both gear ratios and both wheel diameters | Editing the ratios in source |
| Gear ratios | Hardware | Source, if the gearbox changes |
| Droop detector internals (bin width, max descent, saturation threshold, publish period) | Shape the detector, not the shot | Source |
| Power current-limit *enable* flags | Safety | Source |

## Appendix B — Fast diagnosis

| What you see | Look at first |
|---|---|
| Shots scattered, not biased | Stage 1 — a mechanism loop isn't reaching its command |
| Shots biased short, worse at range | `SpeedPerMeterMps` (stage 3), then droop (stage 4) |
| Shots biased short, uniformly | `SpeedScalar` (stage 3) |
| First shot good, later shots short | Droop / `CompensationGain`, or `kP` for faster recovery |
| Shots short only in bursts | `kP` up; check burst `ShotCount` is right |
| Shots fine close, impossible far | `Shooter/Physics/SpeedClamped` — gearing limit, not tuning |
| `ShotCount` wrong | Stage 2. Nothing downstream is trustworthy until this is right |
| Aim drifts as robot turns | Turret `kV` (stage 1a) |
| Good stationary, bad moving | `ShootOnTheMoveGain` (stage 5b) |
| Bad only driving *away* from the target | `MotionRackSaturated` — the flywheel is carrying it and lagging; nothing to tune |
| Bad only driving *at* the target | Rack tracking, not the solve. Watch `Physics/RackErrorDeg` |
| Good on flat, bad on the depot | Tilt offsets (stage 0) then gain (stage 5c) |
| Intake dies mid-match | `Power/Tier` — protection is working; stage 6 to retune it |

## Appendix C — Telemetry worth having on the dashboard

**Shooter/Flywheel/** — the droop compensator

| Key | Meaning |
|---|---|
| `ShotCount` | Balls accepted. **Compare against balls actually fired** — this is how stage 2 is calibrated. |
| `RejectedCount` | Turning points found but thrown out. High ratio to `ShotCount` → a threshold is wrong. |
| `LastTroughDeficitRps` | How far below setpoint the wheel was when the ball left — **the quantity being learned** |
| `LastPerBallDropRps` | What that one ball alone took out. Roughly constant through a burst while the trough deficit accumulates. |
| `LastContactSec` | Ball contact time, descent start to trough. **Expect 10–25 ms.** |
| `LastBallIntervalSec` / `BallsPerSec` | Your real fire rate |
| `LearnedDeficitHereRps` | What the bins say for the speed being commanded right now |
| `PopulatedBins` | 1 → scaling a single measurement. 2+ → learning the real curve and able to extrapolate. |
| `CompensationRps` | The bias currently being added |
| `DutyCycleAvg` | Filtered motor output 0–1. How close the flywheel is running to flat out. |
| `Saturated` | Motor flat out **continuously**, not just a recovery spike. Learning frozen. Hardware answer, not tuning. |
| `SetpointRps` / `MeasuredRps` / `ErrorRps` | Commanded (compensated), actual, difference |
| `AtSpeed` / `Falling` | Detector state |
| `Bins/<lo>-<hi>Rps` | Learned deficit per speed bin. **Put the whole folder on one Elastic graph** to watch them converge. |
| `Curve/BinCenterRps` + `Curve/BinDeficitRps` | The measured curve as arrays, for an AdvantageScope XY plot |
| `Curve/SampleRps` + `Curve/CompensationRps` | What will actually be added at each speed — after gain, clamp and extrapolation |
| `Sampler/Hz` vs `Sampler/TargetHz` | **Distinct** velocity frames consumed per second, against the 200 Hz the signal publishes at. Should match. Reads zero if the sampler thread has stopped. |
| `Sampler/Gaps` | Frames published and never read. **Should stay flat** — any climb and the trough is being measured off an incomplete dip. |
| `Sampler/Timeouts` | No frame inside several periods. Flat on a real robot; climbs in sim, where signals come from the sim loop. |
| `Sampler/Faults` | Exceptions the sampler caught and carried on through. Anything above zero is a bug. |
| `Sampler/Samples` | Frames consumed since boot or since the last `ResetStats` |
| `Store/Restored` | This boot started from a saved curve, not the physics seed |
| `Store/Status` | Why, in one line: `loaded 47 balls`, `no saved file`, `different shooter geometry - discarded` |
| `Store/Unsaved` | New balls not yet written to disk. Clears on disable. |

`Shooter/Flywheel/ClearLearned` (toggle, always live) wipes the learned curve and its saved file —
for after a wheel or belt change, which the geometry signature can't detect.

**Shooter/Physics/** — the aim solver

| Key | Meaning |
|---|---|
| `TurretErrorDeg` / `RackErrorDeg` / `FlywheelErrorRPS` | Solution vs reality. **Stage 1 lives here.** |
| `SpeedClamped` | Solution wants more than `FLYWHEEL_MAX_REV_PER_SEC` — check before trusting a long pass |
| `RackClamped` | Rack against a stop |
| `Feasible` / `Achievable` | The solver saying the shot can't be made from here |
| `FlywheelReady` | Wheel within tolerance of its compensated setpoint |

**Power/** — `Tier`, `BatteryVolts`, `FilteredVolts`, `DriveScale`, `IntakeScale`, `RioBrownedOut`

**Shooter/Tilt/** — `RollDeg`, `PitchDeg` (raw, for stage 0), `AppliedRollDeg`, `AppliedPitchDeg`,
`TotalDeg`

---

## Appendix D — Every knob, with defaults

Requires `Tuning/TuningModeEnabled` = ON.

```
--- Stage 1: mechanism loops -------------------------------------------
Tuning/Turret/kP                          4.0      (12 optimal but too violent for the wiring)
Tuning/Turret/kI                          0.0
Tuning/Turret/kD                          0.25
Tuning/Turret/kV                          0.25     counter-rotation feedforward
Tuning/Turret/kS                          0.65     symmetric static friction
Tuning/Turret/kFF                        -0.30     ONE-directional; sign picks the direction
Tuning/Rack/kP                           20.0
Tuning/Rack/kI                            0.0
Tuning/Rack/kD                            0.25
Tuning/Flywheel/kP                        0.5      raise for faster recovery between balls
Tuning/Flywheel/kI                        0.0
Tuning/Flywheel/kD                        0.0
Tuning/Flywheel/kV                        0.125    12 V / 100 RPS free speed
Tuning/Flywheel/kS                        0.0

--- Stage 2: droop detection -------------------------------------------
Tuning/Flywheel/ShotDetectDropRps         1.5      fall off a peak that starts a descent
Tuning/Flywheel/ShotReboundRps            1.0      climb off a trough that confirms ONE ball
Tuning/Flywheel/MaxPlausibleDroopRps     25.0      deeper than this is a stall, discarded
Tuning/Flywheel/MinDetectRps             10.0      below this the compensator does nothing
Tuning/Flywheel/MaxSetpointSlewRpsPerSec 25.0      faster setpoint moves disarm detection
Tuning/Flywheel/AtSpeedToleranceRps       1.0

--- Stage 3: stationary aim (compensation OFF) -------------------------
Tuning/Shooter/TurretOffsetDeg            0.0      + = CCW
Tuning/Shooter/RackOffsetDeg              0.0      + = flatter
Tuning/Shooter/SpeedScalar                1.0      START HERE for uniform short/long
Tuning/Shooter/SpeedPerMeterMps           0.35     for "close good, far short" = air drag
Tuning/Shooter/FlywheelRpsOffset          2.0      small constant motor-RPS trim
Tuning/Shooter/DescentMarginDeg          12.0      raise if shots skim the rim going up
Tuning/Shooter/GravityMps2                9.807    11.0 in sim (maple-sim fakes drag)

--- Stage 4: droop compensation ON -------------------------------------
Tuning/Flywheel/CompensationGain          1.0      0 while calibrating stage 3
Tuning/Flywheel/LearningRate              0.20
Tuning/Flywheel/MaxCompensationRps       10.0

--- Stage 5: moving shots ----------------------------------------------
Tuning/Shooter/FeederForwardPushMps       0.45     tune at turret 0 deg
Tuning/Shooter/FeederBackwardPushAt90Mps  0.15     tune at turret 90 deg
Tuning/Shooter/ShootOnTheMoveGain         1.0      set 0 first to isolate
Tuning/Shooter/MotionRackMaxSwingDeg     20.0      wider than the rack travel = off
Tuning/Shooter/TiltCompensationGain       1.0      set 0 to remove the whole correction

--- Stage 0: measured, not tuned ---------------------------------------
Tuning/Shooter/LaunchHeightM              0.4826   MEASURE
Tuning/Shooter/LaunchForwardOffsetM      -0.114    MEASURE
Tuning/Shooter/LaunchLeftOffsetM          0.0      MEASURE
Tuning/Shooter/TiltRollOffsetDeg          0.0      read on flat carpet
Tuning/Shooter/TiltPitchOffsetDeg         0.0      read on flat carpet
Tuning/Shooter/FlywheelDiameterM          0.043154 DERIVED - do not override

--- Stage 6: power ----------------------------------------------------
Tuning/Power/ReducedEnterVolts            9.5
Tuning/Power/ReducedExitVolts            10.3
Tuning/Power/CriticalEnterVolts           8.5
Tuning/Power/CriticalExitVolts            9.3
Tuning/Power/VoltageFilterTauSec          0.08
Tuning/Power/MinTierHoldSec               0.75
Tuning/Power/ReducedDriveScale            0.55
Tuning/Power/CriticalDriveScale           0.30
Tuning/Power/ReducedIntakeScale           0.40
Tuning/Power/CriticalIntakeScale          0.0
Tuning/Power/ReducedIntakeSupplyAmps     15
Tuning/Power/ReducedIntakeStatorAmps     35
Tuning/Power/CriticalIntakeSupplyAmps     5
Tuning/Power/CriticalIntakeStatorAmps    10
```

**Toggles — always live, no tuning mode needed:**
```
Tuning/TuningModeEnabled                  OFF      the master switch for everything above
Tuning/ResetToDefaults                    -        momentary; restores all compiled values
Power/BrownoutProtectionEnabled           ON
Shooter/HoldFeedUntilAtSpeed              OFF
Shooter/MotionRackFirst                   ON       off = flywheel carries motion again
Shooter/TrenchTowerStorageEnabled         ON
Shooter/Flywheel/ClearLearned             -        momentary; wipes the learned curve AND its file
Shooter/Flywheel/Sampler/ResetStats       -        momentary; zeroes sampler diagnostics only
```

**Source-only, must be measured or edited in `Constants.java`:**
```
FLYWHEEL_MOI_KG_M2              0.0013     TODO: CAD. Sets kA AND the droop cold-start seed.
FUEL_MASS_KG                    0.145      TODO: weigh one
FLYWHEEL_SHOT_ENERGY_EFFICIENCY 0.5
FLYWHEEL_GEAR_RATIO             28/18      motor rot per 4" wheel rot
FLYWHEEL_SMALL_PER_LARGE_RATIO  18/28      2" rot per 4" rot
FLYWHEEL_MAX_REV_PER_SEC        90.0       90% of Kraken free speed
RACK_MIN_ANGLE / RACK_MAX_ANGLE 18 / 38
TURRET_ROTATOR_MIN/MAX_ANGLE    -200 / 300
ROLLER_STALL_CURRENT_AMPS       55.0
```

---

## Appendix E — One-line summary of the order

```
0  Measure launch point, inertia, ball mass, gyro offsets     (facts, not knobs)
1  Turret / rack / flywheel loops reach their commands        (mechanism before aim)
2  Detector counts balls right, gain = 0                      (measurement before learning)
3  Stationary physics calibration, gain still 0               (aim before compensation)
4  Compensation on, fill bins across the speed range          (learning)
5  Feeder push, shoot-on-the-move, tilt                       (only now start driving)
6  Power tiers over a full match                              (independent)
```
