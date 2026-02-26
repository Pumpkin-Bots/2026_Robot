package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants.VisionConstants;

/**
 * Subsystem for managing multiple PhotonVision cameras and fusing their
 * AprilTag pose estimates to provide accurate robot localization.
 */
public class VisionSubsystem extends SubsystemBase {

    /**
     * Represents a pose estimate from a camera with associated metadata.
     */
    public record VisionPoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> standardDeviations,
        int tagCount,
        double avgTagDistance,
        String cameraName
    ) {}

    // Camera instances
    private final PhotonCamera backLeftCamera;
    private final PhotonCamera backRightCamera;
    private final PhotonCamera intakeRightCamera;
    private final PhotonCamera intakeLeftCamera;

    // Pose estimators for each camera (using new 2-argument constructor)
    private final PhotonPoseEstimator backLeftEstimator;
    private final PhotonPoseEstimator backRightEstimator;
    private final PhotonPoseEstimator intakeRightEstimator;
    private final PhotonPoseEstimator intakeLeftEstimator;

    // List of all cameras and estimators for iteration
    private final List<CameraConfig> cameras = new ArrayList<>();

    // Reference to intake subsystem for conditional camera processing
    private GroundIntakeSubsystem m_intakeSubsystem;

    /**
     * Helper class to bundle a camera with its estimator and transform.
     */
    private static class CameraConfig {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;
        final Transform3d robotToCamera;
        final String name;

        CameraConfig(PhotonCamera camera, PhotonPoseEstimator estimator,
                     Transform3d robotToCamera, String name) {
            this.camera = camera;
            this.estimator = estimator;
            this.robotToCamera = robotToCamera;
            this.name = name;
        }
    }

    public VisionSubsystem() {
        this(null);
    }

    /**
     * Creates a new VisionSubsystem with a reference to the intake subsystem.
     * When an intake subsystem is provided, the intake cameras will only be used
     * when the intake is in the intake position.
     *
     * @param intakeSubsystem The ground intake subsystem (can be null)
     */
    public VisionSubsystem(GroundIntakeSubsystem intakeSubsystem) {
        this.m_intakeSubsystem = intakeSubsystem;

        // Initialize cameras
        backLeftCamera = new PhotonCamera(VisionConstants.BACK_LEFT_CAMERA_NAME);
        backRightCamera = new PhotonCamera(VisionConstants.BACK_RIGHT_CAMERA_NAME);
        intakeRightCamera = new PhotonCamera(VisionConstants.INTAKE_RIGHT_CAMERA_NAME);
        intakeLeftCamera = new PhotonCamera(VisionConstants.INTAKE_LEFT_CAMERA_NAME);

        // Initialize pose estimators using new 2-argument constructor (PhotonVision 2026 API)
        backLeftEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA
        );

        backRightEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA
        );

        intakeRightEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_INTAKE_RIGHT_CAMERA
        );

        intakeLeftEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_INTAKE_LEFT_CAMERA
        );

        // Register cameras for iteration
        cameras.add(new CameraConfig(
            backLeftCamera,
            backLeftEstimator,
            VisionConstants.ROBOT_TO_BACK_LEFT_CAMERA,
            VisionConstants.BACK_LEFT_CAMERA_NAME
        ));
        cameras.add(new CameraConfig(
            backRightCamera,
            backRightEstimator,
            VisionConstants.ROBOT_TO_BACK_RIGHT_CAMERA,
            VisionConstants.BACK_RIGHT_CAMERA_NAME
        ));
        cameras.add(new CameraConfig(
            intakeRightCamera,
            intakeRightEstimator,
            VisionConstants.ROBOT_TO_INTAKE_RIGHT_CAMERA,
            VisionConstants.INTAKE_RIGHT_CAMERA_NAME
        ));
        cameras.add(new CameraConfig(
            intakeLeftCamera,
            intakeLeftEstimator,
            VisionConstants.ROBOT_TO_INTAKE_LEFT_CAMERA,
            VisionConstants.INTAKE_LEFT_CAMERA_NAME
        ));
    }

    /**
     * Gets all valid pose estimates from all cameras.
     * Each estimate is validated for quality before being returned.
     *
     * @return List of valid pose estimates from all cameras
     */
    public List<VisionPoseEstimate> getEstimatedPoses() {
        List<VisionPoseEstimate> estimates = new ArrayList<>();

        for (CameraConfig config : cameras) {
            getEstimatesFromCamera(config, estimates);
        }

        return estimates;
    }

    /**
     * Gets pose estimates from a single camera configuration.
     * Uses the new PhotonVision 2026 API with getAllUnreadResults().
     *
     * @param config The camera configuration to get estimates from
     * @param estimates List to add valid estimates to
     */
    private void getEstimatesFromCamera(CameraConfig config, List<VisionPoseEstimate> estimates) {
        // Skip intake cameras if intake subsystem is available and intake is not in position
        if (m_intakeSubsystem != null && isIntakeCamera(config.name)) {
            if (!m_intakeSubsystem.isInIntakePosition()) {
                return; // Skip processing this camera
            }
        }

        // Use getAllUnreadResults() instead of deprecated getLatestResult()
        List<PhotonPipelineResult> results = config.camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            // Skip if no targets detected
            if (!result.hasTargets()) {
                continue;
            }

            // Try multi-tag pose estimation first (more accurate when multiple tags visible)
            Optional<EstimatedRobotPose> multiTagPose = config.estimator.estimateCoprocMultiTagPose(result);

            if (multiTagPose.isPresent()) {
                // Multi-tag estimate available
                processEstimate(multiTagPose.get(), config.name, estimates);
            } else {
                // Fall back to single-tag estimation using lowest ambiguity
                Optional<EstimatedRobotPose> singleTagPose = config.estimator.estimateLowestAmbiguityPose(result);
                if (singleTagPose.isPresent()) {
                    processEstimate(singleTagPose.get(), config.name, estimates);
                }
            }
        }
    }

    /**
     * Processes an estimated pose, validates it, and adds it to the estimates list if valid.
     *
     * @param estimate The estimated robot pose
     * @param cameraName Name of the camera that produced the estimate
     * @param estimates List to add the validated estimate to
     */
    private void processEstimate(EstimatedRobotPose estimate, String cameraName, List<VisionPoseEstimate> estimates) {
        List<PhotonTrackedTarget> targets = estimate.targetsUsed;

        // Validate the estimate
        if (!isValidEstimate(estimate, targets)) {
            return;
        }

        // Calculate average tag distance for standard deviation scaling
        double avgTagDistance = calculateAverageTagDistance(targets, estimate.estimatedPose);

        // Reject if tags are too far away
        if (avgTagDistance > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            return;
        }

        // Calculate standard deviations based on number of tags and distance
        Matrix<N3, N1> stdDevs = calculateStandardDeviations(targets.size(), avgTagDistance);

        estimates.add(new VisionPoseEstimate(
            estimate.estimatedPose.toPose2d(),
            estimate.timestampSeconds,
            stdDevs,
            targets.size(),
            avgTagDistance,
            cameraName
        ));
    }

    /**
     * Validates a pose estimate based on ambiguity and other quality metrics.
     *
     * @param estimate The estimated pose
     * @param targets The targets used to generate the estimate
     * @return true if the estimate is valid, false otherwise
     */
    private boolean isValidEstimate(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        // For single-tag estimates, check ambiguity
        if (targets.size() == 1) {
            PhotonTrackedTarget target = targets.get(0);
            if (target.getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                return false;
            }
        }

        // Check that the pose is within field bounds (basic sanity check)
        Pose3d pose = estimate.estimatedPose;
        double fieldLength = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength();
        double fieldWidth = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

        // Allow some margin outside the field for measurement error
        double margin = 0.5; // 0.5 meters
        if (pose.getX() < -margin || pose.getX() > fieldLength + margin ||
            pose.getY() < -margin || pose.getY() > fieldWidth + margin) {
            return false;
        }

        return true;
    }

    /**
     * Calculates the average distance from the robot to all detected tags.
     *
     * @param targets The detected targets
     * @param robotPose The estimated robot pose
     * @return Average distance in meters
     */
    private double calculateAverageTagDistance(List<PhotonTrackedTarget> targets, Pose3d robotPose) {
        double totalDistance = 0.0;
        int count = 0;

        for (PhotonTrackedTarget target : targets) {
            var tagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                double distance = robotPose.getTranslation().getDistance(tagPose.get().getTranslation());
                totalDistance += distance;
                count++;
            }
        }

        return count > 0 ? totalDistance / count : Double.MAX_VALUE;
    }

    /**
     * Calculates standard deviations for a pose estimate based on
     * the number of tags detected and their average distance.
     *
     * @param tagCount Number of tags used in the estimate
     * @param avgDistance Average distance to the tags in meters
     * @return Standard deviations matrix [x, y, theta]
     */
    private Matrix<N3, N1> calculateStandardDeviations(int tagCount, double avgDistance) {
        Matrix<N3, N1> baseStdDevs;

        // Use lower standard deviations for multi-tag estimates
        if (tagCount >= VisionConstants.MIN_TAGS_FOR_MULTI_TAG) {
            baseStdDevs = VisionConstants.MULTI_TAG_STD_DEVS.copy();
        } else {
            baseStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS.copy();
        }

        // Scale standard deviations based on distance
        // Further tags = less confidence = higher standard deviations
        double distanceScale = 1.0 + (avgDistance * avgDistance / 30.0);

        return baseStdDevs.times(distanceScale);
    }

    /**
     * Checks if a camera is an intake camera based on its name.
     *
     * @param cameraName The name of the camera
     * @return true if it's an intake camera, false otherwise
     */
    private boolean isIntakeCamera(String cameraName) {
        return cameraName.equals(VisionConstants.INTAKE_RIGHT_CAMERA_NAME) ||
               cameraName.equals(VisionConstants.INTAKE_LEFT_CAMERA_NAME);
    }

    /**
     * Checks if the back left camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isBackLeftConnected() {
        return backLeftCamera.isConnected();
    }

    /**
     * Checks if the back right camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isBackRightConnected() {
        return backRightCamera.isConnected();
    }

   /**
     * Checks if the intake right camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isIntakeRightConnected() {
        return intakeRightCamera.isConnected();
    }

    /**
     * Checks if the intake left camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isIntakeLeftConnected() {
        return intakeLeftCamera.isConnected();
    }

    @Override
    public void periodic() {
        // Periodic updates can be added here if needed
        // The pose estimation is pulled on-demand by RobotContainer
    }
}
