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
    private final PhotonCamera frontLeftCamera;
    private final PhotonCamera frontRightCamera;

    // Pose estimators for each camera (using new 2-argument constructor)
    private final PhotonPoseEstimator frontLeftEstimator;
    private final PhotonPoseEstimator frontRightEstimator;

    // List of all cameras and estimators for iteration
    private final List<CameraConfig> cameras = new ArrayList<>();

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
        // Initialize cameras
        frontLeftCamera = new PhotonCamera(VisionConstants.FRONT_LEFT_CAMERA_NAME);
        frontRightCamera = new PhotonCamera(VisionConstants.FRONT_RIGHT_CAMERA_NAME);

        // Initialize pose estimators using new 2-argument constructor (PhotonVision 2026 API)
        frontLeftEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA
        );

        frontRightEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA
        );

        // Register cameras for iteration
        cameras.add(new CameraConfig(
            frontLeftCamera,
            frontLeftEstimator,
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA,
            VisionConstants.FRONT_LEFT_CAMERA_NAME
        ));
        cameras.add(new CameraConfig(
            frontRightCamera,
            frontRightEstimator,
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA,
            VisionConstants.FRONT_RIGHT_CAMERA_NAME
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
     * Checks if the front left camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isFrontLeftConnected() {
        return frontLeftCamera.isConnected();
    }

    /**
     * Checks if the front right camera is connected.
     *
     * @return true if connected, false otherwise
     */
    public boolean isFrontRightConnected() {
        return frontRightCamera.isConnected();
    }

    @Override
    public void periodic() {
        // Periodic updates can be added here if needed
        // The pose estimation is pulled on-demand by RobotContainer
    }
}
