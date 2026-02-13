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

    public static final class GroundIntakeConstants {

        // ---- Motor CAN IDs ----
        public static final int LEFT_PIVOT_ID  = 21;
        public static final int RIGHT_PIVOT_ID = 22;
        public static final int ROLLER_ID      = 23;

        // ---- Pivot target positions (motor rotations) ----
        // NOTE: These are rotor rotations. Multiply by gear ratio if needed.
        public static final double HOME_POSITION    =  0.0;
        public static final double TRENCH_POSITION  = -2.0;
        public static final double SHOOTER_POSITION = -4.0;

        // ---- Pivot PID gains (Slot 0) ----
        //   kP: raise if pivot is slow, lower if it oscillates
        //   kD: dampens overshoot — increase if oscillating
        //   kS: static friction feed-forward (~0.1–0.5 V)
        //   kG: gravity feed-forward — add if pivot fights gravity
        public static final double PIVOT_KP = 1.0;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.1;

        // ---- Roller speed ----
        public static final double ROLLER_INTAKE_SPEED = -0.60; // 60% duty cycle

        // ---- Position tolerance ----
        public static final double PIVOT_TOLERANCE_ROTATIONS = 0.05;
    }

    public static final class VisionConstants {

        // Camera names as configured in PhotonVision
        public static final String FRONT_LEFT_CAMERA_NAME  = "Front_Left_Camera";
        public static final String FRONT_RIGHT_CAMERA_NAME = "Front_Right_Camera";

        /**
         * Camera mounting transforms relative to robot center.
         * X: Forward, Y: Left, Z: Up
         * TODO: Update once final camera placement is confirmed.
         */
        public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.2667, 0.1143, 0.2286),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0.0))
        );

        public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
            new Translation3d(0.2667, -0.1143, 0.2286),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(0.0))
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
