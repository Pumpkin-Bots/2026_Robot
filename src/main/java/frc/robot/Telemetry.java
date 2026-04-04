package frc.robot;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.VisionSubsystem.VisionPoseEstimate;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        // Set the logger to log to the first flashdrive plugged in
        SignalLogger.setPath("/media/sda1/");   
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Vision telemetry */
    private final NetworkTable visionTable = inst.getTable("Vision");
    private final BooleanPublisher backLeftConnected = visionTable.getBooleanTopic("BackLeftConnected").publish();
    private final BooleanPublisher backRightConnected = visionTable.getBooleanTopic("BackRightConnected").publish();
    private final BooleanPublisher frontRightConnected = visionTable.getBooleanTopic("FrontRightConnected").publish();
    private final BooleanPublisher frontLeftConnected = visionTable.getBooleanTopic("FrontLeftConnected").publish();
    private final IntegerPublisher totalTagsDetected = visionTable.getIntegerTopic("TotalTagsDetected").publish();
    private final StructArrayPublisher<Pose2d> visionPoses = visionTable.getStructArrayTopic("EstimatedPoses", Pose2d.struct).publish();
    private final DoublePublisher avgTagDistance = visionTable.getDoubleTopic("AvgTagDistance").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private double lastPosePrintTime = 0;

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        // Print robot pose to console once per second
        //if (state.Timestamp - lastPosePrintTime >= 1.0) {
        //    System.out.printf("Pose: X=%.2f Y=%.2f Rot=%.1f%n",
        //        state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees());
        //    lastPosePrintTime = state.Timestamp;
        //}

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    /**
     * Update vision telemetry data.
     *
     * @param estimates List of vision pose estimates from cameras
     * @param backLeftCamConnected Whether the back left camera is connected
     * @param backRightCamConnected Whether the back right camera is connected
     * @param frontRightCamConnected Whether the front right camera is connected
     * @param frontLeftCamConnected Whether the front left camera is connected
     */
    public void updateVision(List<VisionPoseEstimate> estimates, boolean backLeftCamConnected, boolean backRightCamConnected, boolean frontRightCamConnected, boolean frontLeftCamConnected) {
        // Publish camera connection status
        backLeftConnected.set(backLeftCamConnected);
        backRightConnected.set(backRightCamConnected);
        frontRightConnected.set(frontRightCamConnected);
        frontLeftConnected.set(frontLeftCamConnected);

        // Calculate totals from estimates
        int totalTags = 0;
        double totalDistance = 0.0;
        Pose2d[] poses = new Pose2d[estimates.size()];

        for (int i = 0; i < estimates.size(); i++) {
            VisionPoseEstimate estimate = estimates.get(i);
            poses[i] = estimate.pose();
            totalTags += estimate.tagCount();
            totalDistance += estimate.avgTagDistance() * estimate.tagCount();
        }

        // Publish aggregated data
        totalTagsDetected.set(totalTags);
        visionPoses.set(poses);
        avgTagDistance.set(totalTags > 0 ? totalDistance / totalTags : 0.0);

        // Log to SignalLogger
        SignalLogger.writeBoolean("Vision/BackLeftConnected", backLeftCamConnected);
        SignalLogger.writeBoolean("Vision/BackRightConnected", backRightCamConnected);
        SignalLogger.writeBoolean("Vision/FrontRightConnected", frontRightCamConnected);
        SignalLogger.writeBoolean("Vision/FrontLeftConnected", frontLeftCamConnected);
        SignalLogger.writeInteger("Vision/TotalTagsDetected", totalTags, "tags");
        SignalLogger.writeDouble("Vision/AvgTagDistance", totalTags > 0 ? totalDistance / totalTags : 0.0, "meters");

        // Log individual camera estimates
        for (VisionPoseEstimate estimate : estimates) {
            String prefix = "Vision/" + estimate.cameraName() + "/";
            SignalLogger.writeStruct(prefix + "Pose", Pose2d.struct, estimate.pose());
            SignalLogger.writeInteger(prefix + "TagCount", estimate.tagCount(), "tags");
            SignalLogger.writeDouble(prefix + "AvgDistance", estimate.avgTagDistance(), "meters");
        }
    }
}
