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
        public static final double SHOOTER_MODE_MAX_SPEED_MULTIPLIER = 0.75; // 50% of max speed
        public static final double SHOOTER_MODE_MAX_ANGULAR_RATE_MULTIPLIER = 1; // 50% of max rotation speed
    }

    public static final class ShooterConstants{
        public static final int TURRET_ROTATOR_ID = 25;
        public static final int SHOOTER_RACK_ID = 26;
        public static final int SHOOTER_FLYWHEEL_ID = 27;


        public static final double ROTATOR_KP = 12; // optimal is 12, but too violent, need stronger wiring chain.
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.35;
        // Velocity feedforward for turret omega tracking (V·s/rot, motor units).
        // Start at 0 (disabled), increment by ~0.05 until turret tracks robot rotation smoothly.
        public static final double ROTATOR_KV = 0.125;

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
        public static final double FLYWHEEL_MAX_REV_PER_SEC = 70.0;
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


        public static final double BLUE_SHUTTLE_TARGET_X_METERS = TAG_26_POSE.getX() - 0.25;
        public static final double BLUE_SHUTTLE_TARGET_Y_METERS = TAG_26_POSE.getY();
        public static final double BLUE_SHUTTLE_TARGET_Z_METERS = TAG_26_POSE.getZ() + 0;

        public static final double RED_SHUTTLE_TARGET_X_METERS = TAG_10_POSE.getX() + 0.25;
        public static final double RED_SHUTTLE_TARGET_Y_METERS = TAG_10_POSE.getY();
        public static final double RED_SHUTTLE_TARGET_Z_METERS = TAG_10_POSE.getZ() + 0;

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

    public static final class TurretConstants {
        public static final int TURRET_INDEXER_ID = 28;

        public static final double TURRET_INDEXER_SPEED = .65; // 60%

        // Indexer spin compensation constants.
        // The turret indexer imparts spin on the ball that causes trajectory error
        // depending on the turret's angle relative to the robot.
        //
        // INDEXER_SPIN_FORWARD_BACK_MAX_MS: maximum extra effective velocity (m/s)
        //   along the barrel axis (forward = ball goes long, backward = ball goes short).
        //   Scales as cos(turretAngle): full effect at 0°/180°, zero at 90°/270°.
        //
        // INDEXER_SPIN_LEFT_RIGHT_MAX_MS: maximum extra effective velocity (m/s)
        //   perpendicular to the barrel axis from the turret's perspective
        //   (positive = ball drifts left of target, negative = right of target).
        //   Scales as sin(turretAngle): full effect at 90°/270°, zero at 0°/180°.
        //
        // Both values are multiplied by flight time inside the virtual target loop
        // to offset the aim point and correct for spin-induced trajectory error.
        // TODO: tune empirically from test shots.
        public static final double INDEXER_SPIN_FORWARD_BACK_MAX_MS = 0.45;
        public static final double INDEXER_SPIN_LEFT_RIGHT_MAX_MS   = 0.15;
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
        public static final double ROLLER_INTAKE_SPEED = 1; // 95% duty cycle
        public static final double RIGHT_INDEXER_SPEED = 0.75; // 60% duty cycle
        public static final double LEFT_INDEXER_SPEED = -0.75; // 60% duty cycle
        public static final double ROLLER_JAM_SPEED = -0.2; // 20% duty cycle

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
