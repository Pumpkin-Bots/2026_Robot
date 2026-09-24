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

The detector runs at **200 Hz** via `Robot.addPeriodic`, not the 50 Hz main loop, and the flywheel
velocity signal is published at 200 Hz to match — a ball's contact is 10–25 ms, so at loop rate the
trough is one sample wide and usually missed. Lowering either rate breaks it.

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
live at all times. See `docs/POWER_AND_FLYWHEEL_TUNING.md` for what each knob is for and when to
reach for it.

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

## Important Notes

- `TunerConstants.java` is generated by CTRE Tuner X — avoid manual edits unless updating specific values
- PathPlanner settings in `src/main/deploy/pathplanner/settings.json` must match drivetrain capabilities (currently configured for Kraken X60s, 5.67:1 gearing, 5.44 m/s max speed)
- Vision standard deviations scale with tag distance and count (see `VisionSubsystem.calculateStandardDeviations`)
- SysId characterization routines are bound to controller (Back/Start + X/Y) for tuning drive motor PID
- Team number: 8793
- Default CAN bus: "rio" (RoboRIO native)
