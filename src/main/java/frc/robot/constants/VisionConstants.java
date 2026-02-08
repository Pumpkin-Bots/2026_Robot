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

/**
 * Constants for the vision subsystem including camera configurations
 * and pose estimation parameters.
 */
public final class VisionConstants {

    // Camera names as configured in PhotonVision
    public static final String FRONT_LEFT_CAMERA_NAME = "Front_Left_Camera";
    public static final String FRONT_RIGHT_CAMERA_NAME = "Front_Right_Camera";

    /**
     * Camera mounting transforms relative to robot center.
     * These transforms describe where each camera is mounted on the robot.
     *
     * The coordinate system is:
     * - X: Forward (positive = front of robot)
     * - Y: Left (positive = left side of robot)
     * - Z: Up (positive = above robot center)
     *
     * Rotation is defined as:
     * - Roll: Rotation around X axis
     * - Pitch: Rotation around Y axis (positive = tilted up)
     * - Yaw: Rotation around Z axis (positive = rotated left)
     *
     * TODO: Measure and update these values once we find the final placement for the drive cameras!
     * These are placeholder values assuming cameras are mounted at the front corners.
     */
    public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
        new Translation3d(
            -0.2667,  // X: 25cm forward from robot center
            0.1143,  // Y: 25cm to the left of robot center
            0.2286   // Z: 20cm above robot center
        ),
        new Rotation3d(
            0.0,                    // Roll: 0 degrees
            Math.toRadians(0),  // Pitch: tilted down 15 degrees
            Math.toRadians(0.0)    // Yaw: angled 15 degrees to the left
        )
    );

    public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
        new Translation3d(
            0.2667,   // X: 25cm forward from robot center
            -0.1143,  // Y: 25cm to the right of robot center
            0.2286    // Z: 20cm above robot center
        ),
        new Rotation3d(
            0.0,                     // Roll: 0 degrees
            Math.toRadians(0),   // Pitch: tilted down 15 degrees
            Math.toRadians(0.0)    // Yaw: angled 15 degrees to the right
        )
    );

    /**
     * The AprilTag field layout for the current game.
     * Uses the official WPILib default field layout for the current season.
     */
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Standard deviations for single-tag pose estimation.
     * Higher values = less trust in vision measurements.
     * Format: [x, y, theta] in meters and radians.
     */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);

    /**
     * Standard deviations for multi-tag pose estimation.
     * Lower values than single-tag because multi-tag is more accurate.
     * Format: [x, y, theta] in meters and radians.
     */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    /**
     * Maximum distance (in meters) at which we trust AprilTag detections.
     * Tags detected further than this will be ignored.
     */
    public static final double MAX_TAG_DISTANCE_METERS = 4.0;

    /**
     * Maximum ambiguity ratio for pose estimation.
     * Higher ambiguity means the pose could be one of multiple solutions.
     * Poses with ambiguity above this threshold will be rejected.
     */
    public static final double MAX_POSE_AMBIGUITY = 0.2;

    /**
     * Minimum number of tags required for high-confidence pose estimation.
     */
    public static final int MIN_TAGS_FOR_MULTI_TAG = 2;

    private VisionConstants() {
        // Prevent instantiation
    }
}
