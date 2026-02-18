// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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


        public static final double ROTATOR_KP = 12; // optimal is 12, but too violent, need stronger chain.
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.35;

        public static final double TURRET_ROTATOR_GEAR_RATIO = -1.0 / 10.0;
        public static final double TURRET_ROTATOR_MIN_ANGLE = -90; 
        public static final double TURRET_ROTATOR_MAX_ANGLE = 90; 
        

        public static final double RACK_KP = 20;
        public static final double RACK_KI = 0.0;
        public static final double RACK_KD = 0.25;

        public static final double RACK_GEAR_RATIO = -1.0 / 333.33;
        public static final double RACK_MIN_ANGLE = 21; // 15 deg
        public static final double RACK_MAX_ANGLE = 48; // 45 deg
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

        // Ball launch position relative to robot center
        // X: forward offset (meters), Z: height above floor (meters)
        // TODO: measure from CAD or physical robot
        public static final double BALL_LAUNCH_FRONT_OFFSET_METERS = -0.2159;
        public static final double BALL_LAUNCH_HEIGHT_METERS = 0.4826;

        // Field-relative 3D position of the shooting target (x, y, z in meters)
        // Derived from CAD: origin (-325.406, -158.84375, 0) in, target (-158.84375, 0, 56.5) in
        public static final double TARGET_X_METERS = 3.75; // long axis was 4.2307
        public static final double TARGET_Y_METERS = 4.5; // short axis was 4.0346
        public static final double TARGET_Z_METERS = 1.4351; // height

        // Distance thresholds for rack angle interpolation
        // At or below MIN_DISTANCE the rack is at its minimum angle (lowest shot).
        // At or above MAX_DISTANCE the rack is at its maximum angle (highest shot).
        // Linearly interpolated between the two.
        public static final double RACK_MIN_DISTANCE_METERS = 2.0;
        public static final double RACK_MAX_DISTANCE_METERS = 5.0;

    }

    public static final class TurretConstants {
        public static final int TURRET_INDEXER_ID = 24;

        public static final double TURRET_INDEXER_SPEED = 1; // 100%
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
        public static final double PIVOT_KP = 1.0;
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
        public static final String FRONT_RIGHT_CAMERA_NAME = "Front_Right_Camera";
        public static final String INTAKE_CAMERA_NAME = "Intake_Camera";

        /**
         * Camera mounting transforms relative to robot center.
         * X: Forward, Y: Left, Z: Up
         * TODO: Update once final camera placement is confirmed.
         */
        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.229, -0.229, 0.340),
            new Rotation3d(0.0, Math.toRadians(15.945), Math.toRadians(116.194))
        );

        public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.0635, 0.201, 0.340),
            new Rotation3d(0.0, Math.toRadians(15.945), Math.toRadians(-63.806))
        );

        // TO DO: update location of intake camera
        public static final Transform3d ROBOT_TO_INTAKE_CAMERA = new Transform3d(
            new Translation3d(0.4, 0.286, 0.39),
            new Rotation3d(0.0, Math.toRadians(9), Math.toRadians(0))
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
