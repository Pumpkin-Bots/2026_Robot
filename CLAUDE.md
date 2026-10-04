# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 8793 robot code for the 2026 season. Java 17 command-based robot using WPILib, CTRE Phoenix 6, PathPlanner, and PhotonVision.

## Build & Deploy Commands

```bash
./gradlew build          # Build the project
./gradlew deploy         # Deploy to RoboRIO
./gradlew simulateJava   # Run in simulation
./gradlew test           # Run unit tests (JUnit 5)
./gradlew test --tests "TestClassName"           # Run single test class
./gradlew test --tests "TestClassName.testMethod" # Run single test method
```

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                  # Entry point
├── Robot.java                 # TimedRobot lifecycle (50ms loop)
├── RobotContainer.java        # Subsystem init, command bindings
├── Telemetry.java             # NetworkTables + SignalLogger telemetry
├── subsystems/
│   ├── CommandSwerveDrivetrain.java  # 4-module swerve drive (Phoenix 6)
│   └── VisionSubsystem.java         # Dual PhotonVision cameras, AprilTag pose estimation
├── constants/
│   └── VisionConstants.java          # Vision config (camera names, transforms, std devs)
├── utils/
│   ├── FlywheelDroopCompensator.java # Learns per-shot flywheel sag, biases commanded RPS
│   ├── FlywheelDroopStore.java       # Persists that learned curve to the RIO across reboots
│   └── PowerBudget.java              # Brownout tiers: cuts intake, then drive, never the shooter
└── generated/
    └── TunerConstants.java           # CTRE Tuner X generated swerve constants
src/main/deploy/
└── pathplanner/              # PathPlanner autos, paths, navgrid, settings
```

## Architecture

- **Pattern:** Command-based (WPILib New Commands framework)
- **Drive:** Swerve drive with TalonFX motors, CANcoders, Pigeon 2 IMU
- **Vision:** Dual PhotonVision cameras with multi-tag AprilTag pose estimation fused into drivetrain odometry
- **Autonomous:** PathPlanner with AutoBuilder and SendableChooser on SmartDashboard

### Vision Fusion Pipeline
1. `VisionSubsystem` runs as a default command, polling cameras via `getAllUnreadResults()`
2. Multi-tag estimation is preferred; falls back to lowest-ambiguity single-tag
3. Estimates are validated (ambiguity, field bounds) and distance-scaled standard deviations calculated
4. Valid estimates passed to `CommandSwerveDrivetrain.addVisionMeasurement()` for Kalman filter fusion

### Controller Bindings (Xbox port 0)
- Left stick: Field-centric translation
- Right stick X: Rotation
- A button: Shooter mode — intake + auto shoot/shuttle/storage by field position (25% drive speed)
- B button: Trench mode (intake deployed, rollers off, normal drive speed)
- X button: Jam mode (rollers reverse, normal drive speed)
- Y button: Shuttle mode — force a shuttle pass from anywhere (25% drive speed)
- Right bumper: Defense mode (intake stowed, normal drive speed)
- Left trigger (≥20%): Storage mode — flywheel/feeder off, rack down, intake acts as a hopper (25% drive speed)
- Right trigger (≥20%): Boost mode — storage outputs at 100% drive speed
- Left bumper: Reset field-centric heading
- Back+Y/X: SysId dynamic forward/reverse
- Start+Y/X: SysId quasistatic forward/reverse

The five button modes latch until another takes over. Storage and boost are sub-states of shooter
mode: the triggers do nothing unless shooter mode is active, and releasing either returns to shooter
mode (`RobotContainer.m_shooterModeActive` / `restoreShooterMode`). Both triggers are disabled in
simulation — the sim's raw axis order on this Mac doesn't match the Driver Station's, and the left
trigger is already the sim's manual-fire control.

### Shoot On The Move — the rack pays, not the flywheel
For a fixed target there is a whole family of launch angles that reach it, each with its own launch
speed, so "which arc" is free and the motion correction can be charged to whichever joint handles it
better. `AimSolver` picks the arc whose required shooter speed equals the speed the flywheel would be
held at **standing in the same spot**, so the flywheel command tracks only the distance (slow) and
the rack absorbs the driver (fast). A loaded flywheel needs most of a second to move a few RPS and a
ball fed mid-change leaves at whatever speed the wheel is passing through; the rack is a geared screw
that makes a few degrees in a fraction of that.

The search only ever goes **steeper**, and that is physics, not a shortcut. Steepening moves speed
out of the horizontal — the axis the robot's velocity acts along — so a few degrees swallow several
m/s. Flattening is the mirror trade and does not work: the lower total energy comes back out as a
longer horizontal component for the motion to add to, so on an 8 m shot spending the entire 12° of
descent margin recovers ~0.2 m/s. Driving *away* from the target the flywheel still has to spin up,
and `Shooter/Physics/MotionRackSaturated` says so. Accuracy is never at stake either way: every
candidate arc is an exact solution, so a bad search gives a shot that is harder on the flywheel,
never one that misses. Operator switch `Shooter/MotionRackFirst` (default on) puts it all back on the
flywheel; `Tuning/Shooter/MotionRackMaxSwingDeg` caps the steepening to protect hang time.

### Flywheel Speed Delivery
The aim solve produces the speed the ball must *leave* at; a wheel held exactly there does not
deliver it, because the ball takes energy out of the wheel on its way through.
`FlywheelDroopCompensator` learns **how far below setpoint the wheel sits at the instant the ball
separates** — that trough is what sets the ball's speed, so biasing the command up by it puts
separation on target. Balls are found as *turning points* in the tracking error: a confirmed
rebound off a local minimum is one ball. That matters because during rapid fire the wheel only
partly recovers between balls, and a detector waiting for a return to baseline reads a whole burst
as one sag and learns nothing from it.

The deficit is learned into 10-RPS bins over commanded speed, not as a single number: the motor's
push-back during contact collapses near free speed, so the net dip is strongly non-linear (roughly
0 RPS at 50 motor RPS, ~8 RPS at 90). Outside measured bins the interpolation clamps rather than
extrapolating. Learning freezes while the motor is voltage-saturated, since asking a maxed-out
wheel for more speed cannot produce any. Cold start falls back to a physics seed from
`FLYWHEEL_MOI_KG_M2`; inert in simulation (maple-sim projectiles leave at exactly wheel speed).

The velocity signal is published at **200 Hz** and the detector consumes **every frame of it exactly
once**, on its own real-time thread (`ShooterSubsystem.flywheelSamplerLoop`) blocking on
`BaseStatusSignal.waitForAll`. Both halves are load-bearing. A ball's contact is 10–25 ms, so the
trough is 2–5 frames wide: at the 50 Hz loop rate it is one sample wide and usually missed, and
dropping even one or two frames *moves* the apparent minimum rather than blurring it — the detector
then learns a confidently wrong number instead of no number.

Being woken by frame arrival is what rules out the two ways a fast *timer* loses samples, neither of
which `setUpdateFrequency` can fix (Phoenix caches one frame per signal, with no queue): a 200 Hz
poll and a 200 Hz publisher are independent clocks that beat, reading some frames twice and missing
others; and `addPeriodic` callbacks are interleaved with the main loop on the same thread, so any
loop overrun starves the sampler and several frames land with only the last surviving — worst
exactly when the robot is busy, i.e. shooting. `Shooter/Flywheel/Sampler/Hz` is the *distinct* frame
rate and should read 200; `Sampler/Gaps` counts frames published and never read and should stay
flat. Lowering the publish rate, or moving the sampler back onto the main thread, breaks this.

Two consequences of the thread: `FlywheelDroopCompensator` is `synchronized` on every public method
(the learned curve is a `double[]` and an `InterpolatingDoubleTreeMap` that must agree, and an
unlocked reader risks a torn double or a hang inside the tree map), and
`getShooterFlywheelVelocityRps()` reads the signal's **cache** rather than calling the no-arg
`getVelocity()`, which refreshes — two threads refreshing one `StatusSignal` is not safe.

A flywheel command of **zero** coasts rather than holding zero. Trench, defense, and storage all ask
for zero, and the velocity loop reads that as a real command: it drags a spinning wheel down with
reverse voltage, dumping its stored energy into the motor and battery, then keeps kS fighting it to
stay stopped. Coasting costs nothing and leaves the wheel part-way to the next shot. `CoastOut`, not
`NeutralOut`, so it freewheels regardless of the motor's configured neutral mode.

The learned bins are saved to `/home/lvuser/flywheel-droop.json` (`FlywheelDroopStore`) and reloaded
at boot, so a session's learning survives the power cycle between matches and a code deploy. Writes
are atomic and happen on a low-priority daemon thread — never inside the 200 Hz sampler — triggered
~20 s after shooting settles and on `disabledInit()`. A saved file is refused unless its geometry
signature (gearing, effective diameter, rotor MOI, ball mass, bin layout) matches the running code;
a wheel or belt change is invisible to that, so `Shooter/Flywheel/ClearLearned` wipes both the live
curve and the file. Nothing is written in simulation. The curve is published per-bin under
`Shooter/Flywheel/Bins/` (Elastic has no XY plot, so one graph widget over that folder shows the
bins converging) and as arrays under `Shooter/Flywheel/Curve/` for AdvantageScope.

There are two resets and they are not interchangeable. `Shooter/Flywheel/ClearLearned` clears the
**collected droop data** — live curve and saved file. `Shooter/Flywheel/Sampler/ResetStats` zeroes
the **sampler diagnostics** only (`Samples`/`Gaps`/`Timeouts`/`Faults`), which are cumulative since
boot and otherwise can't tell you whether the sampler is healthy *now*. Both are momentary
`DashboardToggle`s. The counters are owned by the sampler thread, so the button only raises a flag
and the thread clears them itself between samples.

`FLYWHEEL_EFFECTIVE_DIAMETER_METERS` is **derived**, not hand-entered — it is the single point where
motor RPS becomes ball speed, and it carries both the 28:18 motor reduction and the 28:18
large-to-small wheel ratio. Before this, `FLYWHEEL_GEAR_RATIO` reached only the simulation and had
no effect on a real shot. Note the 28:18 reduction cut ball speed per motor rotation by ~43%:
`flywheelRPSTable` needs re-taking, and long shuttle passes may exceed
`FLYWHEEL_MAX_REV_PER_SEC` (watch `Shooter/Physics/SpeedClamped`).

### Power Priority
`PowerBudget` (ticked from `Robot.robotPeriodic()`, before the scheduler) watches battery voltage
and enforces a stated priority well above the RoboRIO's own 6.8 V cutoff: the flywheel, rack,
turret, and feeder are **never** reduced; the intake rollers/indexers are cut first; the swerve
drive is cut second, and teleop only — the scale is applied at the driver's stick so PathPlanner
autos are untouched. Tiers are hysteretic with a minimum dwell. Cutting the roller's current limit
also lowers the jam-detection threshold to match, so detection doesn't go silent under protection.

### Tuning & Documentation
Both systems are fully live-tunable under `Tuning/Power/` and `Tuning/Flywheel/`, gated behind
`Tuning/TuningModeEnabled` like every other `TunableDouble`. The two operator switches
(`Power/BrownoutProtectionEnabled`, `Shooter/HoldFeedUntilAtSpeed`) are `DashboardToggle`s and are
live at all times. See `docs/TUNING_GUIDE.md` for the order to tune everything in, the exit test for
each stage, and an index of every knob with its default.

### Field-Position Logic
`ShooterMode` decides what to do every loop from where the **shooter's launch point** is (not the
robot centre), using `FieldZones` + `Constants.FieldConstants`: own alliance zone → shoot at the hub;
neutral or opponent zone → shuttle; under a trench arm or tower → storage outputs (drive speed
unchanged). All boundaries are sticky by `ZONE_HYSTERESIS_METERS`. The trench/tower/alliance-zone
geometry in `FieldConstants` is derived from the 2026 AprilTag layout and the game manual — re-measure
and edit those constants rather than the command logic.

## Naming Conventions

- Private fields: `m_fieldName` (e.g., `m_robotContainer`)
- Constants: `kConstantName` (e.g., `kSpeedAt12Volts`) or `UPPER_SNAKE_CASE` (e.g., `MAX_SPEED`)
- Packages: `frc.robot.{subsystems, constants, generated}`
- Use WPILib Units system for type-safe measurements (e.g., `MetersPerSecond`, `RotationsPerSecond`)

## Key Dependencies & Versions

| Library | Version | Purpose |
|---------|---------|---------|
| GradleRIO | 2026.2.1 | WPILib build plugin |
| Phoenix 6 | 26.1.0 | TalonFX, CANcoder, Pigeon 2 |
| PathPlannerLib | 2026.1.2 | Autonomous path planning |
| PhotonVision | v2026.1.1 | AprilTag vision processing |

## CAN Bus IDs

- **Gyro (Pigeon 2):** 5
- **Front-Left:** Drive=6, Steer=1, Encoder=10
- **Front-Right:** Drive=9, Steer=4, Encoder=13
- **Back-Left:** Drive=7, Steer=2, Encoder=11
- **Back-Right:** Drive=8, Steer=3, Encoder=12

## Commutation (FOC)

Every control request in the project states `withEnableFOC(true)`, and every one of them stays in
the **voltage/duty-cycle** family. Phoenix 6 already defaults `EnableFOC` to true on those requests,
so the flag is documentation of intent rather than behaviour — but stating it has a point: it marks
the one way an "add FOC" change goes wrong. FOC is a *commutation mode*, orthogonal to the units a
request is expressed in. Switching to the `*TorqueCurrentFOC` request family, or setting a swerve
`ClosedLoopOutputType` to `TorqueCurrentFOC`, does **not** enable anything that is not already on —
it reinterprets every Slot0 gain from volts to **amps**, so kV 0.125 goes from 0.125 V per rot/s to
0.125 A per rot/s and the mechanism crawls. On the flywheel that reads as a shot that leaves far too
slowly at every distance. Those gains do not carry over and would need re-characterising in amps.

The drivetrain has no `EnableFOC` knob: the CTRE swerve layer builds its module requests natively and
already asks for FOC. `kDriveClosedLoopOutput`/`kSteerClosedLoopOutput` stay `Voltage`.

FOC is a Phoenix Pro feature — an unlicensed TalonFX ignores the request, runs trapezoidal, and sets
`Fault_UnlicensedFeatureInUse`. `Shooter/Flywheel/FocActive` publishes the flywheel's license state
so "we asked for FOC" and "FOC is running" can be told apart from the driver station.

## Important Notes

- `TunerConstants.java` is generated by CTRE Tuner X — avoid manual edits unless updating specific values
- Flywheel stator limit is 160 A (supply 60 A), both raised from 120/40 at the Oakland field. Above
  ~37% duty cycle the supply limit is what actually binds, so if post-shot recovery still looks
  slow, raise supply — not stator
- PathPlanner settings in `src/main/deploy/pathplanner/settings.json` must match drivetrain capabilities (currently configured for Kraken X60s, 5.67:1 gearing, 5.44 m/s max speed)
- Vision standard deviations scale with tag distance and count (see `VisionSubsystem.calculateStandardDeviations`)
- SysId characterization routines are bound to controller (Back/Start + X/Y) for tuning drive motor PID
- Team number: 8793
- Default CAN bus: "rio" (RoboRIO native)
