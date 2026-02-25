// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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

    public static final class ShooterConstants{
        public static final int TURRET_ROTATOR_ID = 25;
        public static final int SHOOTER_RACK_ID = 26;
        public static final int SHOOTER_FLYWHEEL_ID = 27;


        public static final double ROTATOR_KP = 12; // optimal is 12, but too violent, need stronger wiring chain.
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.35;

        public static final double TURRET_ROTATOR_GEAR_RATIO = -20 / 200.0;
        public static final double TURRET_ROTATOR_MIN_ANGLE = -200;
        public static final double TURRET_ROTATOR_MAX_ANGLE = 320;


        public static final double RACK_KP = 20;
        public static final double RACK_KI = 0.0;
        public static final double RACK_KD = 0.25;

        public static final double RACK_GEAR_RATIO = -1.0 / 333.33;
        public static final double RACK_MIN_ANGLE = 15; // 15 deg
        public static final double RACK_MAX_ANGLE = 42; // 45 deg
        // Rack gear ratio is backwards
        // MAX Rotations is at maximum height (lower shot)
        // MIN Rotations is at minimum height (higher shot)

        public static final double FLYWHEEL_KP = 0.04;
        public static final double FLYWHEEL_KI = 0.125;
        public static final double FLYWHEEL_KD = 0;

        public static final double FLYWHEEL_GEAR_RATIO = 24.0 / 36.0;
        public static final double FLYWHEEL_LARGE_DIAMETER_METERS = 0.1016; // 4 inches
        public static final double FLYWHEEL_SMALL_DIAMETER_METERS = 0.0508; // 2 inches
        public static final double FLYWHEEL_MAX_REV_PER_SEC = 70.0;
        // Effective flywheel diameter used to convert motor RPS to muzzle velocity
        // for motion compensation flight-time estimation.
        // raw average: (0.1016 + 0.0508) / 2 = 0.0762m (3 inches)
        public static final double FLYWHEEL_EFFECTIVE_DIAMETER_METERS = 0.0762;

        // Ball launch position relative to robot center
        // X: forward offset (meters), Z: height above floor (meters)
        // TODO: measure from CAD or physical robot
        public static final double BALL_LAUNCH_FRONT_OFFSET_METERS = -0.2159;
        public static final double BALL_LAUNCH_HEIGHT_METERS = 0.4826;

        // Field-relative 3D position of the shooting target (AprilTag 26)
        private static final Pose3d TAG_26_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(26)
            .orElseThrow();
        public static final double TARGET_X_METERS = TAG_26_POSE.getX() + 0.597;
        public static final double TARGET_Y_METERS = TAG_26_POSE.getY() + 0;
        public static final double TARGET_Z_METERS = TAG_26_POSE.getZ() + 0.610;

        // Field-relative 3D position of the red side shooting target (AprilTag 10)
        private static final Pose3d TAG_10_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(10)
            .orElseThrow();
        public static final double RED_TARGET_X_METERS = TAG_10_POSE.getX() - 0.597;
        public static final double RED_TARGET_Y_METERS = TAG_10_POSE.getY() + 0;
        public static final double RED_TARGET_Z_METERS = TAG_10_POSE.getZ() + 0.610;

        // Translation3d constants for easy use in commands
        public static final Translation3d BLUE_TARGET_POSITION = new Translation3d(
            TARGET_X_METERS,
            TARGET_Y_METERS,
            TARGET_Z_METERS);
        public static final Translation3d RED_TARGET_POSITION = new Translation3d(
            RED_TARGET_X_METERS,
            RED_TARGET_Y_METERS,
            RED_TARGET_Z_METERS);

    }

    public static final class TurretConstants {
        public static final int TURRET_INDEXER_ID = 24;

        public static final double TURRET_INDEXER_SPEED = 0.33; // 60%
    }

    public static final class GroundIntakeConstants {

        // ---- Motor CAN IDs ----
        public static final int LEFT_PIVOT_ID  = 21;
        public static final int RIGHT_PIVOT_ID = 22;
        public static final int ROLLER_ID      = 23;

        // ---- Pivot target positions (motor rotations) ----
        // NOTE: These are rotor rotations. Multiply by gear ratio if needed.
        public static final double HOME_POSITION    =  0.0;
        public static final double TRENCH_POSITION  = -2.0;
        public static final double SHOOTER_POSITION = -4.7;

        // ---- Pivot PID gains (Slot 0) ----
        //   kP: raise if pivot is slow, lower if it oscillates
        //   kD: dampens overshoot — increase if oscillating
        //   kS: static friction feed-forward (~0.1–0.5 V)
        //   kG: gravity feed-forward — add if pivot fights gravity
        public static final double PIVOT_KP = 1.25;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.1;

        // ---- Roller speed ----
        public static final double ROLLER_INTAKE_SPEED = -0.95; // 60% duty cycle
        public static final double ROLLER_JAM_SPEED = 0.2;

        // ---- Position tolerance ----
        public static final double PIVOT_TOLERANCE_ROTATIONS = 0.05;
    }

    public static final class VisionConstants {

        // Camera names as configured in PhotonVision
        public static final String BACK_LEFT_CAMERA_NAME  = "Back_Left_Camera";
        public static final String BACK_RIGHT_CAMERA_NAME = "Back_Right_Camera";
        public static final String INTAKE_RIGHT_CAMERA_NAME = "Intake_Right_Camera";
        public static final String INTAKE_LEFT_CAMERA_NAME  = "Intake_Left_Camera";


        /**
         * Camera mounting transforms relative to robot center.
         * X: Forward, Y: Left, Z: Up
         * TODO: Update once final camera placement is confirmed.
         */
        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.2413, 0.2286, 0.36195),
            new Rotation3d(0.0, Math.toRadians(14.036), Math.toRadians(116.194))
        );

        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.2413, -0.2286, 0.36195),
            new Rotation3d(0.0, Math.toRadians(14.036), Math.toRadians(-116.194))
        );

        // TO DO: update location of intake right camera
        public static final Transform3d ROBOT_TO_INTAKE_RIGHT_CAMERA = new Transform3d(
            new Translation3d(0.317, -0.305, 0.381),
            new Rotation3d(0.0, Math.toRadians(9), Math.toRadians(-45))
        );

        // TO DO: update location of intake left camera
        public static final Transform3d ROBOT_TO_INTAKE_LEFT_CAMERA = new Transform3d(
            new Translation3d(0.317, 0.305, 0.381),
            new Rotation3d(0.0, Math.toRadians(9), Math.toRadians(45))
        );

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Standard deviations — higher = less trust. Format: [x, y, theta]
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

        public static final double MAX_TAG_DISTANCE_METERS = 4.0;
        public static final double MAX_POSE_AMBIGUITY      = 0.2;
        public static final int    MIN_TAGS_FOR_MULTI_TAG  = 2;
    }
}
