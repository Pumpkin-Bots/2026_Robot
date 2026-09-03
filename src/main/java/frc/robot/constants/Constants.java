// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

    public static final class DriveConstants {
        // Normal driving speeds (100% of max)
        public static final double NORMAL_MAX_SPEED_MULTIPLIER = 1.0;
        public static final double NORMAL_MAX_ANGULAR_RATE_MULTIPLIER = 1.0;

        // Shooter mode speeds (reduced for precise positioning)
        public static final double SHOOTER_MODE_MAX_SPEED_MULTIPLIER = 0.25; // 25% of max speed
        public static final double SHOOTER_MODE_MAX_ANGULAR_RATE_MULTIPLIER = 0.5; // 50% of max rotation speed

        // Boost mode speeds — full authority, for sprinting between fuel piles while the shooter
        // is parked. Separate from NORMAL_* so the two can be tuned apart later.
        public static final double BOOST_MAX_SPEED_MULTIPLIER = 1.0;
        public static final double BOOST_MAX_ANGULAR_RATE_MULTIPLIER = 1.0;

        // How far an analog trigger must be pressed before its mode engages.
        public static final double MODE_TRIGGER_THRESHOLD = 0.2;
    }

    /**
     * Field geometry for the 2026 REBUILT field, used to decide what the shooter should be doing
     * from where it is standing. All coordinates are WPILib blue-origin field coordinates: X runs
     * from the blue alliance wall (0) to the red alliance wall, Y runs from the scoring-table side
     * (0) to the far side.
     *
     * <p>Sources for the numbers below: field length/width and the hub/trench/tower stations come
     * from the 2026 AprilTag layout ({@code 2026-rebuilt-welded.json}); structure sizes come from
     * the game manual's arena chapter. Everything here is a plain constant precisely so it can be
     * re-measured and corrected on the field without touching command logic.
     */
    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS  = 8.069;

        // ---- Alliance zones ----
        // Each ALLIANCE ZONE runs 158.6 in from its own alliance wall toward midfield; the NEUTRAL
        // ZONE is everything between them. 4.028 m lands exactly on the near face of the hub /
        // trench line, which is the boundary you can actually see on the carpet.
        public static final double ALLIANCE_ZONE_DEPTH_METERS = 4.028; // 158.6 in

        /** Largest X still inside the BLUE alliance zone. */
        public static final double BLUE_ALLIANCE_ZONE_MAX_X_METERS = ALLIANCE_ZONE_DEPTH_METERS;
        /** Smallest X still inside the RED alliance zone. */
        public static final double RED_ALLIANCE_ZONE_MIN_X_METERS =
            FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS;

        // ---- Trenches ----
        // A TRENCH sits against each long guardrail at the same field-length station as that
        // alliance's hub — four of them in total (two per alliance, one per side of the field).
        // The X centres below are the hub centres, taken from the AprilTag layout: the trench
        // AprilTags (17/22/23/28 blue, 1/6/7/12 red) sit on the same station line.
        public static final double BLUE_TRENCH_CENTER_X_METERS =  4.6255;
        public static final double RED_TRENCH_CENTER_X_METERS  = 11.9155;

        // Front-to-back depth of the trench structure along X (47 in per the manual).
        public static final double TRENCH_DEPTH_METERS = 1.194;

        // How far the drivable channel under the trench arm reaches in from the guardrail. The
        // manual gives 50.34 in of clearance under the arm; beyond that the structure is solid.
        public static final double TRENCH_CHANNEL_DEPTH_METERS = 1.279;

        // Padding added around the trench box before it counts as "in the trench". Grow this if
        // the shooter is still spinning up as the robot noses into the trench.
        public static final double TRENCH_MARGIN_METERS = 0.30;

        // ---- Towers ----
        // A TOWER is built into each alliance wall between driver stations 2 and 3, 49.25 in wide
        // and 45 in deep. The Y centres are the midpoints of that wall's tower AprilTag pairs
        // (31/32 blue, 15/16 red), which is why they are not exactly on the field centreline.
        public static final double TOWER_DEPTH_METERS = 1.143; // 45 in, measured off the wall
        public static final double TOWER_WIDTH_METERS = 1.251; // 49.25 in, along Y
        public static final double BLUE_TOWER_CENTER_Y_METERS = 3.962;
        public static final double RED_TOWER_CENTER_Y_METERS  = 4.107;

        // Padding around the tower box, same idea as TRENCH_MARGIN_METERS.
        public static final double TOWER_MARGIN_METERS = 0.30;

        // ---- Boundary hysteresis ----
        // Every zone test above is a hard edge, and a robot parked on one would otherwise flip
        // decisions every loop — swinging the turret between the hub and the shuttle aim point, or
        // strobing the flywheel on and off. Once a boundary has been crossed, the robot has to come
        // back this far past it before the decision flips again.
        public static final double ZONE_HYSTERESIS_METERS = 0.15;
    }

    public static final class ShooterConstants{
        public static final int TURRET_ROTATOR_ID = 25;
        public static final int SHOOTER_RACK_ID = 26;
        public static final int SHOOTER_FLYWHEEL_ID = 27;


        public static final double ROTATOR_KP = 18; // optimal is 12, but too violent, need stronger wiring chain.
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.4;
        // Velocity feedforward for turret omega tracking (V·s/rot, motor units).
        // Start at 0 (disabled), increment by ~0.05 until turret tracks robot rotation smoothly.
        public static final double ROTATOR_KV = 0.6;

        public static final double TURRET_ROTATOR_GEAR_RATIO = -20 / 200.0;
        public static final double TURRET_ROTATOR_MIN_ANGLE = -200;
        public static final double TURRET_ROTATOR_MAX_ANGLE = 300;


        public static final double RACK_KP = 20;
        public static final double RACK_KI = 0.0;
        public static final double RACK_KD = 0.25;

        public static final double RACK_GEAR_RATIO = -1.0 / 333.33;
        public static final double RACK_MIN_ANGLE = 15; // 15 deg
        public static final double RACK_MAX_ANGLE = 42; // 45 deg
        // Rack gear ratio is backwards
        // MAX Rotations is at maximum height (lower shot)
        // MIN Rotations is at minimum height (higher shot)

        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;
        public static final double FLYWHEEL_KV = 0.13;

        public static final double FLYWHEEL_GEAR_RATIO = 1;
        public static final double FLYWHEEL_LARGE_DIAMETER_METERS = 0.1016; // 4 inches
        public static final double FLYWHEEL_SMALL_DIAMETER_METERS = 0.0508; // 2 inches
        // Ceiling on the commanded flywheel speed. Sized from the longest shot the robot is
        // actually asked to make: a shuttle pass from the far side of the opposing alliance zone,
        // turret ~0.46 m (1.5 ft) off their back wall, driving parallel to that wall at the
        // drivetrain's full 5.44 m/s. Solved over the whole length of that wall, the worst case
        // asks for 69.3 RPS — a lateral sprint costs ~8 RPS over the same shot standing still,
        // because the shooter has to cancel the robot's sideways velocity as well as reach the
        // target. 80 leaves margin for that number to grow when the drag term (SPEED_PER_METER,
        // still a first guess) gets tuned up, and is 80% of a Kraken X60's 100 RPS free speed,
        // which is direct-drive here.
        public static final double FLYWHEEL_MAX_REV_PER_SEC = 80.0;
        // Effective flywheel diameter used to convert motor RPS to muzzle velocity
        // for motion compensation flight-time estimation.
        // raw average: (0.1016 + 0.0508) / 2 = 0.0762m (3 inches)
        public static final double FLYWHEEL_EFFECTIVE_DIAMETER_METERS = 0.0762;

        // Ball launch position relative to robot center
        // X: forward offset (meters, positive = toward robot front)
        // Y: lateral offset (meters, positive = toward robot left)
        // Z: height above floor (meters)
        // TODO: measure from CAD or physical robot
        public static final double BALL_LAUNCH_FRONT_OFFSET_METERS = -0.114;
        public static final double BALL_LAUNCH_LATERAL_OFFSET_METERS = 0.0;
        public static final double BALL_LAUNCH_HEIGHT_METERS = 0.4826;

        // ---- Physics aiming calibration ----
        // Every value below is exposed live on SmartDashboard under "Tuning/Shooter/..." (see
        // ShooterTuning). Tune on the dashboard, then copy the winning number back here so it
        // survives a reboot. Suggested tuning order is 1 → 6.

        // (0) Gravity used by the ballistic solve. maple-sim's projectiles use a flat 11.0 m/s^2
        // instead of 9.81 to fake air drag, so sim and the real robot want different values here.
        // ShooterTuning picks the right default automatically; override only if you know why.
        public static final double PHYSICS_GRAVITY_SIM_MPS2  = 11.0;
        public static final double PHYSICS_GRAVITY_REAL_MPS2 = 9.80665;

        // (1) How much steeper than the minimum-energy angle to aim. The minimum-energy angle
        // reaches the target exactly at the apex of its arc, which at close range can arrive on
        // the way UP and skim the rim. Biasing steeper guarantees the ball is descending on
        // arrival. Raise if shots ride the rim, lower if they drop short and steep.
        public static final double DESCENT_MARGIN_DEG = 12.0;

        // (2) Mechanical zero calibration. Pure command offsets applied AFTER the physics solve —
        // these correct "the rack reads 20 deg but is physically at 22 deg", not the physics.
        // RACK: positive = flatter shot (rack angle up). TURRET: positive = counter-clockwise.
        public static final double RACK_ANGLE_OFFSET_DEG   = 0.0;
        public static final double TURRET_ANGLE_OFFSET_DEG = 0.0;

        // (3) Speed calibration. The no-drag solve always UNDER-predicts the speed a real ball
        // needs, and the shortfall grows with range, so there are two knobs:
        //   SPEED_SCALAR      — flat multiplier on required launch speed. Fixes "every shot is
        //                       short/long by the same fraction". Start here.
        //   SPEED_PER_METER   — extra m/s added per meter of distance. Fixes "close shots are
        //                       right but long shots fall short" (that's air drag).
        public static final double SPEED_SCALAR_DEFAULT    = 1.0;
        public static final double SPEED_PER_METER_DEFAULT = 0.35;

        // (4) Final flywheel trim in motor RPS, applied after the speed→RPS conversion. Use this
        // for a small constant bias (e.g. ball compression losses at the exit roller) rather than
        // distorting FLYWHEEL_EFFECTIVE_DIAMETER_METERS, which also affects the sim projectile.
        public static final double FLYWHEEL_RPS_OFFSET_DEFAULT = 2;

        // (5) Shoot-on-the-move authority, 0 to 1. 1.0 = fully compensate for robot velocity,
        // 0.0 = ignore it entirely (aim as if stopped). Set to 0 to isolate a stationary aiming
        // problem from a motion-compensation problem, then walk it back up.
        public static final double SHOOT_ON_THE_MOVE_GAIN = 1.0;

        // Field-relative 3D position of the shooting target (AprilTag 26)
        private static final Pose3d TAG_26_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(26)
            .orElseThrow();
        public static final double BLUE_TARGET_X_METERS = TAG_26_POSE.getX() + 0.597;
        public static final double BLUE_TARGET_Y_METERS = TAG_26_POSE.getY() + 0;
        // Lowered from +0.610 — maple-sim's RebuiltHub scores fuel between z=1.5748m and
        // z=1.8288m (a 10 in tall zone starting at the hub's own position), so +0.610 (aiming
        // near the top of that zone) was causing shots to overshoot, worse at longer range.
        // +0.45 aims near the low edge of the real scoring zone instead.
        public static final double BLUE_TARGET_Z_METERS = TAG_26_POSE.getZ() + 0.45;

        // Field-relative 3D position of the red side shooting target (AprilTag 10)
        private static final Pose3d TAG_10_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(10)
            .orElseThrow();
        public static final double RED_TARGET_X_METERS = TAG_10_POSE.getX() - 0.597;
        public static final double RED_TARGET_Y_METERS = TAG_10_POSE.getY() + 0;
        public static final double RED_TARGET_Z_METERS = TAG_10_POSE.getZ() + 0.45;


        // Shuttle passes are thrown to the carpet, not into the hub, so the aim point is the floor.
        // The physics solver takes a real z and solves for where the ball actually comes down;
        // borrowing the tag's height here (as this used to) told it to land the ball 1.2 m in the
        // air, which lands every pass short of where it was aimed.
        public static final double SHUTTLE_TARGET_Z_METERS = 0.0;

        // How far to the side of the hub a shuttle pass lands. The pass is aimed to whichever side
        // of the hub the shooter is already on, so the ball stays off the hub structure and comes
        // down where a teammate on that side of the field can pick it up.
        public static final double SHUTTLE_SIDE_OFFSET_METERS = 2.5;

        // Rack angle held in storage/boost mode. RACK_MIN_ANGLE is the rack's lowest physical
        // position (0 rack rotations), which is where it has to be to fit under a trench arm.
        public static final double RACK_STORAGE_ANGLE_DEG = RACK_MIN_ANGLE;

        // How far in front of the hub tag the pass lands, toward midfield. This used to sit 0.25 m
        // *behind* the tag; moving it 1 m toward midfield takes a meter off every shuttle shot,
        // which is a meter further back the robot can be standing when it takes one. The sideways
        // offset applied in ShuttleMode keeps the ball clear of the hub structure itself, so
        // landing level with the hub does not mean landing on it.
        private static final double SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS = 0.75;

        public static final double BLUE_SHUTTLE_TARGET_X_METERS =
            TAG_26_POSE.getX() + SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS;
        public static final double BLUE_SHUTTLE_TARGET_Y_METERS = TAG_26_POSE.getY();
        public static final double BLUE_SHUTTLE_TARGET_Z_METERS = SHUTTLE_TARGET_Z_METERS;

        // Red mirrors it: midfield is the other direction from the red hub.
        public static final double RED_SHUTTLE_TARGET_X_METERS =
            TAG_10_POSE.getX() - SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS;
        public static final double RED_SHUTTLE_TARGET_Y_METERS = TAG_10_POSE.getY();
        public static final double RED_SHUTTLE_TARGET_Z_METERS = SHUTTLE_TARGET_Z_METERS;

        // Translation3d constants for easy use in commands
        public static final Translation3d BLUE_TARGET_POSITION = new Translation3d(
            BLUE_TARGET_X_METERS,
            BLUE_TARGET_Y_METERS,
            BLUE_TARGET_Z_METERS);
        public static final Translation3d RED_TARGET_POSITION = new Translation3d(
            RED_TARGET_X_METERS,
            RED_TARGET_Y_METERS,
            RED_TARGET_Z_METERS);
        public static final Translation3d BLUE_SHUTTLE_TARGET_POSITION = new Translation3d(
            BLUE_SHUTTLE_TARGET_X_METERS,
            BLUE_SHUTTLE_TARGET_Y_METERS,
            BLUE_SHUTTLE_TARGET_Z_METERS);
        public static final Translation3d RED_SHUTTLE_TARGET_POSITION = new Translation3d(
            RED_SHUTTLE_TARGET_X_METERS,
            RED_SHUTTLE_TARGET_Y_METERS,
            RED_SHUTTLE_TARGET_Z_METERS);

    }

    /**
     * Tuning for the fused field-relative velocity estimate used by shoot-on-the-move.
     *
     * <p>The Pigeon 2's accelerometer is the primary source: it responds instantly to a direction
     * change, where wheel odometry lags and lies during a slip. Its weakness is drift — integrating
     * acceleration accumulates error over a long straight. So wheel odometry is folded back in as a
     * slow correction, and the vision-corrected pose as an even slower one, which pins the estimate
     * down without giving up the IMU's fast response.
     */
    public static final class VelocityEstimatorConstants {
        public static final double GRAVITY_MPS2 = 9.80665;

        // How hard wheel odometry pulls the IMU-integrated velocity back, as a first-order time
        // constant in seconds. This is THE main knob.
        //   Larger (0.5+) = trust the IMU more: snappier response to direction changes, more drift.
        //   Smaller (0.1) = trust the wheels more: less drift, back toward plain wheel odometry.
        public static final double WHEEL_TRUST_TAU_SECONDS = 0.15;

        // Accelerometer bias learning rate. The residual between the IMU-integrated velocity and
        // wheel odometry is integrated into a per-axis bias estimate that gets subtracted from raw
        // acceleration. This is what stops a long straight from slowly drifting.
        // Set to 0 to disable bias learning entirely (pure complementary filter).
        public static final double ACCEL_BIAS_GAIN = 0.20;

        // Vision correction: velocity derived by differentiating the vision-fused pose over
        // VISION_SAMPLE_WINDOW_SECONDS. Slow and noisy, but unbiased — it catches systematic wheel
        // odometry error (wrong wheel radius, carpet scrub) that the wheels alone cannot see.
        // Set VISION_TRUST_TAU_SECONDS very high to disable.
        public static final double VISION_TRUST_TAU_SECONDS     = 0.75;
        public static final double VISION_SAMPLE_WINDOW_SECONDS = 0.25;
        // Ignore a vision-derived sample this far off the current estimate — a vision pose jump
        // differentiates into a huge bogus velocity spike, and this rejects it.
        public static final double VISION_REJECT_THRESHOLD_MPS = 2.0;

        // Rotation of the Pigeon's accelerometer axes into robot axes, in degrees. Only needed if
        // the Pigeon is not mounted with its X axis pointing robot-forward. The proper fix is the
        // Pigeon 2 MountPose config in Tuner X; this is the quick field workaround.
        public static final double IMU_MOUNT_YAW_OFFSET_DEG = 0.0;

        // ---- Pigeon mounting position, relative to robot center (meters, robot frame) ----
        // X: positive toward the robot FRONT.  Y: positive toward the robot LEFT.
        //
        // This mattered not at all when the Pigeon was only a gyro: yaw rate is identical
        // everywhere on a rigid body. It matters a great deal now that its ACCELEROMETER is being
        // used, because an off-center point on a rotating robot is genuinely accelerating even when
        // the robot's center is not. Spinning in place at 8 rad/s with the Pigeon 0.2 m off center
        // makes it read ~13 m/s^2 of pure centripetal acceleration that the robot center never
        // experiences — integrate that and the velocity estimate is garbage the moment the robot
        // turns. The estimator subtracts both the centripetal (omega^2 * r) and tangential
        // (alpha * r) terms to recover the center's acceleration.
        //
        // Measure from CAD or by tape measure to the Pigeon chip itself. Leave both 0 only if the
        // Pigeon really is at the robot's rotational center.
        // TODO: measure on the real robot.
        public static final double PIGEON_OFFSET_FORWARD_METERS = -0.0127;
        public static final double PIGEON_OFFSET_LEFT_METERS    = -0.2921;

        // Smoothing time constant for the yaw acceleration (alpha) used by the tangential term.
        // Alpha comes from differentiating yaw rate, which is noisy, so it gets low-passed. Larger =
        // smoother but laggier. Only has any effect when the Pigeon offsets above are non-zero.
        public static final double YAW_ACCEL_FILTER_TAU_SECONDS = 0.04;

        // Hard sanity clamp on the fused estimate so a bad accelerometer can never run away.
        public static final double MAX_PLAUSIBLE_SPEED_MPS = 6.0;
    }

    public static final class TurretConstants {
        public static final int TURRET_INDEXER_ID = 28;

        public static final double TURRET_INDEXER_SPEED = -.65; // 60% before

        // ---- Feeder / indexer disturbance compensation ----
        // The feeder shoves the ball as it enters the turret, so the ball leaves carrying a little
        // velocity the shooter never asked for. Both terms below act along the robot's FORE/AFT
        // axis, and which one dominates depends on where the turret is pointing:
        //
        // FEEDER_FORWARD_PUSH_MPS: with the turret pointing straight ahead (or straight back), the
        //   feeder's shove runs down the barrel and the ball leaves faster than commanded. Scales
        //   as cos^2(turretAngle), so it is at full strength at 0/180 deg and gone at +/-90.
        //   Symptom when wrong: shots go long/short, by an amount that changes as the robot rotates
        //   relative to the target.
        //
        // FEEDER_BACKWARD_PUSH_AT_90_MPS: as the turret swings toward 90 deg to EITHER side, that
        //   help disappears and the ball instead comes out with extra velocity toward the ROBOT
        //   REAR. Scales as sin^2(turretAngle) — sin SQUARED, so the push is the same at +90 and
        //   -90, which is how the real effect behaves.
        //   Symptom when wrong: shots are fine with the turret forward but consistently off when
        //   shooting out the side, in a way that is mirrored left and right.
        //
        // Corrected by plain vector subtraction, the same mechanism as shoot-on-the-move.
        //
        // Both are live-tunable on SmartDashboard under "Tuning/Shooter/". The two terms are
        // independent, so tune each at the turret angle where the other contributes nothing: park
        // the turret at 0 deg for the forward term, then at 90 deg for the rearward term. Robot
        // STATIONARY for both.
        // TODO: tune empirically from test shots.
        public static final double FEEDER_FORWARD_PUSH_MPS        = 0.45;
        public static final double FEEDER_BACKWARD_PUSH_AT_90_MPS = 0.15;
    }

    public static final class GroundIntakeConstants {

        // ---- Motor CAN IDs ----
        public static final int RIGHT_PIVOT_ID = 20;
        public static final int LEFT_PIVOT_ID  = 21;
        public static final int ROLLER_ID = 22;
        public static final int LEFT_INDEXER_ID = 23;
        public static final int RIGHT_INDEXER_ID = 24;

        // ---- Gear ratio ----
        // Number of motor rotor rotations per one full mechanism (pivot arm) rotation.
        // Example: if the pivot has a 100:1 gearbox, this is 100.0.
        // Used by Phoenix 6 SensorToMechanismRatio so all positions below are in
        // mechanism rotations (i.e. 0.25 = 90°), not raw rotor counts.
        public static final double PIVOT_GEAR_RATIO = 18.0; // 10 rotor rotations per 1 arm rotation

        // ---- Pivot target positions (mechanism rotations, 1.0 = full 360°) ----
        // ZERO CONVENTION: 0.0 mechanism rotations = arm horizontal (pointing straight out).
        //   - At this angle gravity torque is maximum → Arm_Cosine kG applies at 100%.
        //   - The encoder is seeded automatically in the constructor, assuming the robot
        //     starts with the intake resting on the UP hard stop.
        //
        // UP position: intake stowed, resting on the upper hard stop.
        //   Measure with a protractor or CAD: if the arm is 30° above horizontal, this is 30/360 = 0.0833
        public static final double PIVOT_UP_ROTATIONS   = 0.0; // TODO: measure (mechanism rotations)

        // DOWN position: intake deployed, resting on the lower hard stop.
        //   If the arm is 20° below horizontal, this is -20/360 = -0.0556
        public static final double PIVOT_DOWN_ROTATIONS = 0.355; // 0.35 mechanism rot × 10:1 gear ratio = 3.5 rotor rotations

        // ---- Motion Magic profile ----
        // Cruise velocity: max mechanism speed during a move (rotations/second).
        public static final double PIVOT_CRUISE_VELOCITY_RPS = 1.0;

        // Acceleration: how fast to ramp up to cruise velocity (rotations/second²).
        public static final double PIVOT_ACCELERATION_RPS2 = 2.0;

        // Jerk: limits rate of acceleration change (rotations/second³). 0 = disabled.
        public static final double PIVOT_JERK_RPS3 = 0.0;

        // ---- Pivot PID + feed-forward gains (Slot 0) ----
        public static final double PIVOT_KP = 60.0;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 2.0;
        public static final double PIVOT_KS = 0.0;
        public static final double PIVOT_KV = 0.96;
        public static final double PIVOT_KA = 0.0;
        public static final double PIVOT_KG = 1.3;

        // GravityOffsetPosition: position offset (mechanism rotations) applied to the
        // cosine calculation so that kG is correct when encoder zero ≠ horizontal.
        // Formula: kG × cos(2π × (position + PIVOT_GRAVITY_OFFSET))
        public static final double PIVOT_GRAVITY_OFFSET = 0.2;

        // ---- Named positions for commands (mechanism rotations) ----
        public static final double DEFENSE_POSITION = PIVOT_UP_ROTATIONS;   // intake stowed
        public static final double TRENCH_POSITION  = PIVOT_DOWN_ROTATIONS; // intake deployed for pickup
        public static final double SHOOTER_POSITION = PIVOT_DOWN_ROTATIONS; // intake deployed for feeding shooter

        // ---- Roller / indexer speeds ----
        public static final double INDEXER_TO_ROLLER_RATIO = 0.9244;
        public static final double ROLLER_INTAKE_SPEED = 0.75;
        public static final double RIGHT_INDEXER_SPEED = ROLLER_INTAKE_SPEED * INDEXER_TO_ROLLER_RATIO; // 60% duty cycle
        public static final double LEFT_INDEXER_SPEED = -RIGHT_INDEXER_SPEED; // 60% duty cycle
        public static final double ROLLER_JAM_SPEED = -0.2; // 20% duty cycle

        // ---- Automatic jam recovery ----
        // While intaking, a stalled roller motor (high stator current + not turning) means a ball
        // is wedged. The intake path reverses at full speed for UNJAM_DURATION_SECONDS to spit it
        // back out, then resumes intaking. The flywheel is never touched, so it stays at speed.
        //
        // Stall current: the roller's stator limit is 70 A, so a genuinely stalled roller pins at
        // 70 A. Anything below the limit but well above normal intaking draw works here.
        public static final double ROLLER_STALL_CURRENT_AMPS = 55.0;

        // Stall velocity, in MOTOR ROTOR rotations/second (the roller has no
        // SensorToMechanismRatio configured, so this is the motor's own speed). Free speed at
        // ROLLER_INTAKE_SPEED is roughly 75 rps, so this is "basically not turning".
        public static final double ROLLER_STALL_VELOCITY_RPS = 10.0;

        // How long both conditions must hold before it counts as a jam. Mainly there so the
        // current spike during roller spin-up (high current, still slow) doesn't read as a stall.
        // Lower it for faster recovery, raise it if spin-up false-triggers an unjam.
        public static final double ROLLER_STALL_DEBOUNCE_SECONDS = 0.25;

        // How long to run in reverse before going back to intaking.
        public static final double UNJAM_DURATION_SECONDS = 0.5;

        // Full-speed reverse burst used by the automatic recovery — the mirror of the intake
        // speeds above, at 100% instead of ROLLER_JAM_SPEED's gentler manual 20%.
        public static final double ROLLER_UNJAM_SPEED        = -1.0;
        public static final double RIGHT_INDEXER_UNJAM_SPEED = -1.0;
        public static final double LEFT_INDEXER_UNJAM_SPEED  =  1.0;

        // ---- Position tolerance ----
        // How close (in mechanism rotations) counts as "at position".
        public static final double PIVOT_TOLERANCE_ROTATIONS = 0.02; // ~7°


        public static final double PIVOT_FORCE_DOWN_POWER = 0;
    }

    public static final class LEDConstants {
        public static final int CANDLE_ID = 29;
        public static final int LED_COUNT = 60; // 1m strip at 60 LEDs/m
    }

    public static final class VisionConstants {

        // Camera names as configured in PhotonVision
        public static final String BACK_LEFT_CAMERA_NAME  = "Back_Left_Camera";
        public static final String BACK_RIGHT_CAMERA_NAME = "Back_Right_Camera";
        public static final String FRONT_RIGHT_CAMERA_NAME = "Front_Right_Camera";
        public static final String FRONT_LEFT_CAMERA_NAME  = "Front_Left_Camera";


        /**
         * Camera mounting transforms relative to robot center.
         * X: Forward, Y: Left, Z: Up
         * TODO: Update once final camera placement is confirmed.
         */
        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.3048, 0.1778, 0.3397),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(135))
        );

        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.3048, -0.1778, 0.3397),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(-135))
        );

        // TO DO: update location of front right camera
        public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
            new Translation3d(0.1651, -0.3143, 0.381),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(-45))
        );

        // TO DO: update location of front left camera
        public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
            new Translation3d(0.1651, 0.3143, 0.381),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(45))
        );

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Standard deviations — higher = less trust. Format: [x, y, theta]
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

        // These std devs above are conservatively tuned for real-world camera noise. maple-sim's
        // simulated cameras don't have anywhere near that much noise, so trusting them exactly
        // as little as real cameras makes odometry drift (e.g. from a wall collision) recover
        // unrealistically slowly in sim. Scales the final std dev down (only in simulation) to
        // let simulated vision correct drift faster — tune this if recovery still feels too slow
        // or corrections start looking too twitchy/aggressive.
        public static final double SIM_STD_DEV_SCALE_FACTOR = 0.1;

        public static final double MAX_TAG_DISTANCE_METERS = 6.0;
        public static final double MAX_POSE_AMBIGUITY      = 0.2;
        public static final int    MIN_TAGS_FOR_MULTI_TAG  = 2;

        // Tags to always ignore for pose estimation (tower back tags + outpost tags on both sides)
        public static final Set<Integer> IGNORED_TAG_IDS = Set.of(13, 14, 15, 16, 29, 30, 31, 32);
        //public static final Set<Integer> IGNORED_TAG_IDS = Set.of();
    }
}
