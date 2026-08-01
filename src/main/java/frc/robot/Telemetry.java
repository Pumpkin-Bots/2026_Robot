package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.ironmaple.simulation.SimulatedArena;
import frc.robot.constants.Constants;
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
        //SignalLogger.setPath("/media/sda1/");   
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

    /*
     * 3D mechanism component poses, for AdvantageScope's 3D Field robot model.
     * Each Pose3d is a transform relative to that component's zeroed pose in the
     * robot's 3D model asset (not a field-relative pose) — AdvantageScope applies
     * these on top of the model's component pivots, in this fixed order
     * (must match the model_0.glb ... model_6.glb numbering of the asset):
     *   [0] turret rotator    — yaw about the turret's mounting pivot
     *   [1] shooter rack      — pitch about the rack's mounting pivot
     *   [2] intake pivot      — pitch about the intake's mounting pivot
     *   [3] front-left wheel  — yaw about the module's steer axis
     *   [4] front-right wheel — yaw about the module's steer axis
     *   [5] back-left wheel   — yaw about the module's steer axis
     *   [6] back-right wheel  — yaw about the module's steer axis
     * Wheel order matches TunerConstants.createDrivetrain()'s module order.
     * Rotation signs assume the model's local axes match WPILib's convention;
     * flip a sign here if a part spins the wrong way once you have a real model.
     */
    private final NetworkTable mechanismTable = inst.getTable("Mechanism3d");
    private final StructArrayPublisher<Pose3d> componentPoses = mechanismTable.getStructArrayTopic("Components", Pose3d.struct).publish();
    private final Pose3d[] m_componentPoseArray = new Pose3d[7];

    /* maple-sim game piece poses (sim only) — drag this topic onto the 3D Field and set its type
     * to "Game Piece" to render fuel flying/resting on the field. */
    private final NetworkTable fieldSimTable = inst.getTable("FieldSimulation");
    private final StructArrayPublisher<Pose3d> gamePiecePoses = fieldSimTable.getStructArrayTopic("Fuel", Pose3d.struct).publish();

    /* maple-sim's true simulated robot pose (sim only) — distinct from /DriveState/Pose, which is
     * the odometry ESTIMATE and can drift from wheel slip. Useful as a second "Ghost" robot in
     * AdvantageScope to see when/how far the two diverge. */
    private final StructPublisher<Pose2d> groundTruthPose = fieldSimTable.getStructTopic("RobotGroundTruthPose", Pose2d.struct).publish();
    // Wheel radius (2 in) — height of each wheel's center above the floor. Placeholder if your
    // actual wheel radius differs from TunerConstants' kWheelRadius.
    private static final double kWheelCenterHeightMeters = 0.0508;
    // Corrects for the wheel model's authored "forward" direction not matching WPILib's
    // zero-angle convention. Flip the sign if the wheels end up turned the wrong way.
    private static final double kWheelYawOffsetRadians = Math.PI / 2;

    // Turret rotation axis's offset from robot center (X forward, Y left, Z up) — placeholder,
    // reused from ShooterConstants' ball-launch offsets since no dedicated turret-axis
    // measurement exists yet. Replace once you have the real pivot location from CAD.
    private static final Translation3d kTurretPivotOffsetMeters = new Translation3d(-0.114, 0.0, 0.3);
    // Same mesh-alignment correction as the wheels, applied to the turret (and rack, since it
    // rides on the turret and shares the same mesh-forward convention). Positive yaw rotates
    // toward +Y (left) per WPILib's convention, so an extra 45° was added here to correct it
    // further left on top of the base 90°.
    private static final double kTurretYawOffsetRadians = Math.PI / 2 + Math.PI / 4;
    // The rack's mount point relative to the turret's own rotation axis, before the turret
    // spins it — placeholder. This gets rotated by the turret's actual angle each loop, since
    // the rack physically sweeps around with the turret.
    private static final Translation3d kRackMountOffsetMeters = new Translation3d(0.1, 0.0, 0.15);

    // Ground intake pivot axis's offset from robot center. Originally (11.75in forward, 0, 11.24in
    // up) from CAD measurements, but that rendered ~11in too high and ~11in too far left — close
    // enough in magnitude to both original values that the model's native position likely already
    // accounts for most of the height, and the lateral value landed on the wrong axis. Adjusted
    // as a best-effort correction; still an estimate, expect to keep tuning this by eye.
    private static final Translation3d kIntakePivotOffsetMeters = new Translation3d(
        Units.Inches.of(11.75).in(Units.Meters), Units.Inches.of(-11).in(Units.Meters), 0.0);
    // The imported CAD model was authored showing the intake already in its DOWN/deployed
    // position (PIVOT_DOWN_ROTATIONS), not the UP/stowed rest position (0 rotations) that
    // intakePivotAngleDeg is measured from — so this constant offset rotates the model's
    // authored pose back to matching 0° = up.
    private static final double kIntakeAngleOffsetDegrees =
        -Constants.GroundIntakeConstants.PIVOT_DOWN_ROTATIONS * 360.0;
    // The whole intake assembly's authored "forward" faces sideways instead of forward — a fixed
    // yaw correction, separate from the dynamic elevation pivot above. Flip the sign if it ends
    // up facing backward instead.
    private static final double kIntakeYawOffsetRadians = Math.PI / 2;

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

    // Pre-allocated Pose2d arrays (indexed by count 0-4) to avoid per-loop allocation
    private final Pose2d[][] m_visionPoseArrays = {
        new Pose2d[0], new Pose2d[1], new Pose2d[2], new Pose2d[3], new Pose2d[4]
    };

    // Cached SignalLogger key strings per camera name to avoid per-loop string allocation
    // Value: [poseKey, tagCountKey, avgDistanceKey]
    private final HashMap<String, String[]> m_signalLoggerKeys = new HashMap<>();

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
     * Publish the current mechanism angles as 3D component poses for AdvantageScope's 3D Field.
     * Call from the main robot thread only (e.g. {@code Robot.robotPeriodic()}) — {@link #m_componentPoseArray}
     * is not synchronized, and {@code moduleStates} should come from {@code drivetrain.getState()} rather
     * than the {@link #telemeterize} callback, which runs on CTRE's separate odometry thread.
     *
     * @param turretAngleDeg      current turret rotator angle in degrees (see {@code ShooterSubsystem.getTurretRotatorAngleDeg})
     * @param rackAngleDeg        current shooter rack angle in degrees (see {@code ShooterSubsystem.getShooterRackAngleDeg})
     * @param intakePivotAngleDeg current ground intake pivot angle in degrees (see {@code GroundIntakeSubsystem.getPivotAngleDeg})
     * @param moduleStates        current swerve module states, in FrontLeft/FrontRight/BackLeft/BackRight order
     *                            (see {@code CommandSwerveDrivetrain.getState().ModuleStates})
     * @param moduleLocations     each module's fixed (X, Y) offset from robot center, same order
     *                            (see {@code CommandSwerveDrivetrain.getModuleLocations()})
     */
    public void updateMechanismPoses(
            double turretAngleDeg, double rackAngleDeg, double intakePivotAngleDeg,
            SwerveModuleState[] moduleStates, Translation2d[] moduleLocations) {
        double turretAngleRad = Math.toRadians(turretAngleDeg);
        double turretYawForRender = turretAngleRad + kTurretYawOffsetRadians;

        // Turret translation is baked into this Pose3d rather than the model asset's
        // zeroedPosition, for the same reason as the wheels below: splitting a rotating part's
        // position (config.json) from its rotation (this published pose) makes AdvantageScope
        // pivot it around the robot's origin instead of its own offset.
        m_componentPoseArray[0] = new Pose3d(kTurretPivotOffsetMeters, new Rotation3d(0, 0, turretYawForRender));

        // The rack rides on the turret, so both its rotation AND its position must account for
        // the turret's yaw — AdvantageScope does not chain component transforms together on its
        // own (each is independent relative to the base model). The mount offset is rotated by
        // the turret's actual (uncorrected) angle, since that's the real physical sweep; the
        // render-only yaw offset is applied just to the final rotation, matching the turret mesh.
        Translation3d rackPosition = kTurretPivotOffsetMeters.plus(
            kRackMountOffsetMeters.rotateBy(new Rotation3d(0, 0, turretAngleRad)));
        // Elevation pivots the rack about its local X axis (roll) — the mesh's own "tilt" axis
        // turned out to be authored along X rather than Y (pitch) in WPILib's convention.
        m_componentPoseArray[1] = new Pose3d(
            rackPosition, new Rotation3d(-Math.toRadians(rackAngleDeg), 0, turretYawForRender));

        // Elevation pivots about local X (roll), not Y (pitch) — same authored-axis mismatch as
        // the rack. kIntakeAngleOffsetDegrees corrects for the CAD model's rest pose being
        // authored at the DOWN position instead of 0deg/up. Sign flipped from the first attempt —
        // the other direction swung it past "up" into upside-down/underground instead.
        m_componentPoseArray[2] = new Pose3d(
            kIntakePivotOffsetMeters,
            new Rotation3d(
                Math.toRadians(intakePivotAngleDeg + kIntakeAngleOffsetDegrees), 0, kIntakeYawOffsetRadians));
        // Each wheel's translation is baked into this same Pose3d, not into the model asset's
        // zeroedPosition — AdvantageScope applies a component's zeroedPosition and its logged
        // rotation in an order that pivots around the robot's origin, not the component's own
        // offset, so a rotating part with a config-side-only offset swings in an arc instead of
        // spinning in place. Keeping translation and rotation together in one published pose
        // avoids that ambiguity entirely (config.json's zeroedPosition for these must stay [0,0,0]).
        for (int i = 0; i < 4; i++) {
            Translation3d wheelOffset = new Translation3d(
                moduleLocations[i].getX(), moduleLocations[i].getY(), kWheelCenterHeightMeters);
            // The wheel model's own "forward" doesn't line up with WPILib's zero-angle
            // convention, so a fixed offset is added to every wheel's yaw to correct it.
            m_componentPoseArray[3 + i] = new Pose3d(
                wheelOffset, new Rotation3d(0, 0, moduleStates[i].angle.getRadians() + kWheelYawOffsetRadians));
        }
        componentPoses.set(m_componentPoseArray);
        SignalLogger.writeStructArray("Mechanism3d/Components", Pose3d.struct, m_componentPoseArray);
    }

    /**
     * Publishes every simulated "Fuel" game piece's pose (in flight and resting on the field) so
     * AdvantageScope can render them. No-op on a real robot — maple-sim's arena is sim-only.
     */
    public void updateGamePieces() {
        if (RobotBase.isSimulation()) {
            gamePiecePoses.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    /**
     * Publishes maple-sim's true simulated robot pose, distinct from the odometry estimate
     * published elsewhere as /DriveState/Pose. No-op on a real robot.
     *
     * @param groundTruthPose the robot's true simulated pose (see
     *     {@code CommandSwerveDrivetrain.getSimulatedGroundTruthPose()})
     */
    public void updateGroundTruthPose(Pose2d groundTruthPose) {
        this.groundTruthPose.set(groundTruthPose);
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
        int count = estimates.size();
        Pose2d[] poses = m_visionPoseArrays[Math.min(count, 4)];

        for (int i = 0; i < count; i++) {
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

        // Log individual camera estimates using cached key strings to avoid per-loop String allocation
        for (VisionPoseEstimate estimate : estimates) {
            String[] keys = m_signalLoggerKeys.computeIfAbsent(estimate.cameraName(), name -> {
                String prefix = "Vision/" + name + "/";
                return new String[]{prefix + "Pose", prefix + "TagCount", prefix + "AvgDistance"};
            });
            SignalLogger.writeStruct(keys[0], Pose2d.struct, estimate.pose());
            SignalLogger.writeInteger(keys[1], estimate.tagCount(), "tags");
            SignalLogger.writeDouble(keys[2], estimate.avgTagDistance(), "meters");
        }
    }
}
