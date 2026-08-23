package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.utils.AimSolver;
import frc.robot.utils.AimSolver.AimSolution;
import frc.robot.utils.ShooterTuning;
import frc.robot.utils.TunableDouble;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterSubsystem implements Subsystem {

    private final TalonFX m_turretRotatorMotor;
    private final TalonFX m_shooterRackMotor;
    private final TalonFX m_shooterFlywheelMotor;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VelocityEstimator m_velocityEstimator;
    private final ShooterTuning m_tuning = new ShooterTuning();
    private boolean m_turretZeroed = false;
    private double m_lastCommandedTurretAngle = 0.0;
    private AimSolution m_lastSolution = null;

    // ---- Simulation ----
    // The turret and rack skip physics simulation entirely and just snap their simulated
    // position straight to whatever was last commanded (see updateTurretSim/updateRackSim) —
    // simulating their real motor dynamics turned out to need real tuning (moment of inertia,
    // PID gains) that isn't available yet, and wasn't the point of this sim: the goal is
    // testing the aiming math and 3D visualization, not sim-only PID convergence. The flywheel
    // keeps a real physics model since velocity control doesn't have the same instability risk.
    private static final double kSimPeriodSeconds = 0.02;
    private static final DCMotor kFlywheelGearbox = DCMotor.getKrakenX60(1);

    private final FlywheelSim m_flywheelSim;
    private double m_lastCommandedTurretRotorPosition = 0.0;
    private double m_lastCommandedRackRotorPosition = 0.0;

    // ---- Live PID tuning via SmartDashboard (turret only for now) ----
    // Slot0Configs below are normally baked in once at startup, so testing a new gain would
    // otherwise mean editing Constants, recompiling, and redeploying every time. These follow the
    // same TuningMode switch as every other tunable: dashboard values only apply while it is on.
    private final TunableDouble m_turretKP =
        new TunableDouble("Tuning/Turret/kP", Constants.ShooterConstants.ROTATOR_KP);
    private final TunableDouble m_turretKI =
        new TunableDouble("Tuning/Turret/kI", Constants.ShooterConstants.ROTATOR_KI);
    private final TunableDouble m_turretKD =
        new TunableDouble("Tuning/Turret/kD", Constants.ShooterConstants.ROTATOR_KD);
    private final TunableDouble m_turretKV =
        new TunableDouble("Tuning/Turret/kV", Constants.ShooterConstants.ROTATOR_KV);

    // Last gains actually pushed to the motor, so the config is only re-applied when something
    // changes rather than every loop.
    private double m_appliedTurretKP = Constants.ShooterConstants.ROTATOR_KP;
    private double m_appliedTurretKI = Constants.ShooterConstants.ROTATOR_KI;
    private double m_appliedTurretKD = Constants.ShooterConstants.ROTATOR_KD;
    private double m_appliedTurretKV = Constants.ShooterConstants.ROTATOR_KV;

    private static final Slot0Configs turretRotatorGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.ROTATOR_KP)
        .withKI(Constants.ShooterConstants.ROTATOR_KI)
        .withKD(Constants.ShooterConstants.ROTATOR_KD)
        .withKV(Constants.ShooterConstants.ROTATOR_KV);

    private static final Slot0Configs rackGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.RACK_KP)
        .withKI(Constants.ShooterConstants.RACK_KI)
        .withKD(Constants.ShooterConstants.RACK_KD);

    private static final Slot0Configs flywheelGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.FLYWHEEL_KP)
        .withKI(Constants.ShooterConstants.FLYWHEEL_KI)
        .withKD(Constants.ShooterConstants.FLYWHEEL_KD)
        .withKV(Constants.ShooterConstants.FLYWHEEL_KV);

    // Turret rotator soft limits in motor rotations (0 motor rotations = 0° turret angle)
    private static final double TURRET_MIN_ROTATIONS =
        Constants.ShooterConstants.TURRET_ROTATOR_MIN_ANGLE
        / 360.0 / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;
    private static final double TURRET_MAX_ROTATIONS =
        Constants.ShooterConstants.TURRET_ROTATOR_MAX_ANGLE
        / 360.0 / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;

    private static final SoftwareLimitSwitchConfigs turretSoftLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Math.max(TURRET_MIN_ROTATIONS, TURRET_MAX_ROTATIONS))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Math.min(TURRET_MIN_ROTATIONS, TURRET_MAX_ROTATIONS));

    // Rack soft limits in motor rotations (0 motor rotations = RACK_MIN_ANGLE)
    private static final double RACK_LIMIT_ROTATIONS =
        (Constants.ShooterConstants.RACK_MAX_ANGLE - Constants.ShooterConstants.RACK_MIN_ANGLE)
        / 360.0 / Constants.ShooterConstants.RACK_GEAR_RATIO;

    private static final SoftwareLimitSwitchConfigs rackSoftLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Math.max(0, RACK_LIMIT_ROTATIONS))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Math.min(0, RACK_LIMIT_ROTATIONS));

    private static final CurrentLimitsConfigs flywheelCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80);

    private static final CurrentLimitsConfigs rackCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(25)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(35);

    private static final CurrentLimitsConfigs turretCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(35)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(45);

    // Lookup tables: distance (meters) → value. Populate with empirical test shots.
    private static final InterpolatingDoubleTreeMap rackAngleTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPSTable = new InterpolatingDoubleTreeMap();

    // Flight time table: distance (meters) → measured time-of-flight (seconds).
    // Measure by recording frame timestamps from ball leave to scoring event, or
    // use slow-motion video. Replace placeholder values with measured data.
    private static final InterpolatingDoubleTreeMap flightTimeTable = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill in from test shots — put(distance_meters, rack_angle_deg)
        rackAngleTable.put(2.0, 15.0);
        rackAngleTable.put(2.5, 21.0);
        rackAngleTable.put(3.0, 24.0);
        rackAngleTable.put(3.5, 24.0);
        rackAngleTable.put(4.0, 26.0);
        rackAngleTable.put(4.5,26.0);
        rackAngleTable.put(5.0, 28.0);
        // Above 5 m: 1-meter intervals. Angle approaches hardware max (42°) by 8 m.
        // launch_angle = 90° - rack_angle; higher rack = flatter shot, lower rack = steeper lob.
        rackAngleTable.put(6.0,  33.0);
        rackAngleTable.put(6.75, 36.0);
        // 8 m+: hold at hardware max (42°) for the flattest possible shot
        rackAngleTable.put(8.0,  42.0);
        rackAngleTable.put(9.0,  42.0);
        rackAngleTable.put(10.0, 42.0);
        rackAngleTable.put(11.0, 42.0);
        rackAngleTable.put(12.0, 42.0);
        rackAngleTable.put(13.0, 42.0);
        rackAngleTable.put(14.0, 42.0);
        rackAngleTable.put(15.0, 42.0);

        // TODO: Fill in from test shots — put(distance_meters, flywheel_motor_RPS)
        flywheelRPSTable.put(1.3589, 30.75);
        flywheelRPSTable.put(1.5, 31.0);
        flywheelRPSTable.put(2.0, 32.0);
        flywheelRPSTable.put(2.5, 33.0);
        flywheelRPSTable.put(3.0, 33.5);
        flywheelRPSTable.put(3.5, 35.0);
        flywheelRPSTable.put(4.0, 36.25);
        flywheelRPSTable.put(4.5, 37.75);
        flywheelRPSTable.put(5.0, 39.0);
        flywheelRPSTable.put(6.0,  41.0);
        // 8 m+: physics-based at rack=42° (launch≈43°), empirical ~0.205 m/s per RPS.
        // t = sqrt(2*(d*tan43° - 1.1938)/9.81), v0 = d/(cos43°*t), RPS = v0/0.205
        flywheelRPSTable.put(8.0,  47.0);
        flywheelRPSTable.put(9.0,  49.5);
        flywheelRPSTable.put(10.0, 52.0);
        flywheelRPSTable.put(11.0, 56.0);
        flywheelRPSTable.put(12.0, 59.0);
        flywheelRPSTable.put(13.0, 63.0);
        flywheelRPSTable.put(14.0, 66.0);
        flywheelRPSTable.put(15.0, 69.0);
        // One entry per rack-angle/flywheel-RPS breakpoint. Interpolated from prior
        // empirical data; refine with measured values from slow-motion video or logging.
        flightTimeTable.put(1.3589, 0.89);
        flightTimeTable.put(1.5,    0.95);
        flightTimeTable.put(2.0,    1.08);
        flightTimeTable.put(2.5,    1.17);
        flightTimeTable.put(3.0,    1.24);
        flightTimeTable.put(3.5,    1.24);
        flightTimeTable.put(4.0,    1.19);
        flightTimeTable.put(4.5,    1.18);
        flightTimeTable.put(5.0,    1.17);
        flightTimeTable.put(6.0,    1.15);
        flightTimeTable.put(6.75,   1.17);
        // 8 m+: flat trajectory (rack=42°, launch≈43°).
        // t = sqrt(2*(d*tan43° - 1.1938)/9.81)
        flightTimeTable.put(8.0,    1.13);
        flightTimeTable.put(9.0,    1.21);
        flightTimeTable.put(10.0,   1.29);
        flightTimeTable.put(11.0,   1.36);
        flightTimeTable.put(12.0,   1.43);
        flightTimeTable.put(13.0,   1.49);
        flightTimeTable.put(14.0,   1.56);
        flightTimeTable.put(15.0,   1.62);

    }

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, VelocityEstimator velocityEstimator) {
        m_drivetrain = drivetrain;
        m_velocityEstimator = velocityEstimator;

        m_turretRotatorMotor = new TalonFX(Constants.ShooterConstants.TURRET_ROTATOR_ID);
        m_shooterRackMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_RACK_ID);
        m_shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_ID);

        m_turretRotatorMotor.getConfigurator().apply(turretRotatorGains);
        m_turretRotatorMotor.getConfigurator().apply(turretSoftLimits);
        m_turretRotatorMotor.getConfigurator().apply(turretCurrentLimits);
        // Position is read back for mechanism telemetry/3D visualization, so request it explicitly.
        m_turretRotatorMotor.getPosition().setUpdateFrequency(50);
        m_turretRotatorMotor.optimizeBusUtilization();

        m_shooterRackMotor.getConfigurator().apply(rackGains);
        m_shooterRackMotor.getConfigurator().apply(rackSoftLimits);
        m_shooterRackMotor.getConfigurator().apply(rackCurrentLimits);
        m_shooterRackMotor.getPosition().setUpdateFrequency(50);
        m_shooterRackMotor.optimizeBusUtilization();

        m_shooterFlywheelMotor.getConfigurator().apply(flywheelGains);
        m_shooterFlywheelMotor.getConfigurator().apply(flywheelCurrentLimits);
        // Velocity is read back to compute simulated projectile launch speed — request it explicitly.
        m_shooterFlywheelMotor.getVelocity().setUpdateFrequency(50);
        m_shooterFlywheelMotor.optimizeBusUtilization();

        if (RobotBase.isSimulation()) {
            m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    kFlywheelGearbox, 0.001, Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO),
                kFlywheelGearbox
            );
        } else {
            m_flywheelSim = null;
        }
    }

    /**
     * Computes the ball's launch position in field coordinates,
     * accounting for the robot's heading and the forward/lateral/height offset from center.
     */
    private Translation3d calculateLaunchPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        double rx = m_tuning.launchForwardOffsetMeters();
        double ry = m_tuning.launchLeftOffsetMeters();
        return new Translation3d(
            robotPose.getX() + rx * Math.cos(heading) - ry * Math.sin(heading),
            robotPose.getY() + rx * Math.sin(heading) + ry * Math.cos(heading),
            m_tuning.launchHeightMeters());
    }

    /**
     * Field-relative velocity of the ball's launch point.
     *
     * <p>This is not the same as the robot's velocity. The launch point sits off the robot's center,
     * so chassis rotation swings it through an arc, giving it ground speed the chassis itself does
     * not have. Aiming against the chassis velocity instead would put every shot fired while turning
     * off to one side.
     */
    private Translation2d calculateLaunchPointVelocity(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = m_velocityEstimator.getFieldRelativeSpeeds();
        double omega = fieldSpeeds.omegaRadiansPerSecond;
        double rx = m_tuning.launchForwardOffsetMeters();
        double ry = m_tuning.launchLeftOffsetMeters();
        double heading = robotPose.getRotation().getRadians();
        return new Translation2d(
            fieldSpeeds.vxMetersPerSecond
                + (-rx * Math.sin(heading) - ry * Math.cos(heading)) * omega,
            fieldSpeeds.vyMetersPerSecond
                + ( rx * Math.cos(heading) - ry * Math.sin(heading)) * omega);
    }

    /**
     * Computes a virtual target position that compensates for launch-point motion and the feeder's
     * push during projectile flight. Uses iterative refinement with flight time from the empirical
     * lookup table.
     *
     * <p>Used only by the lookup-table aiming path ({@link #calculateShooterActions}). The physics
     * path corrects for the same two effects by vector subtraction instead — see {@link AimSolver}.
     *
     * @param targetPosition  field-relative 3D position of the actual target
     * @param launchPosition  field-relative 3D position of the ball at launch
     * @param launchPointVel  field-relative velocity of the launch point
     * @param robotHeadingRad robot heading in radians (field-relative)
     * @return adjusted 3D aim point that accounts for drift during flight
     */
    private Translation3d calculateVirtualTargetPosition(
            Translation3d targetPosition,
            Translation3d launchPosition,
            Translation2d launchPointVel,
            double robotHeadingRad) {
        var tuning = m_tuning.snapshot();
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);

        double motionDriftX = launchPointVel.getX() * tuning.shootOnTheMoveGain();
        double motionDriftY = launchPointVel.getY() * tuning.shootOnTheMoveGain();

        Translation3d virtualTarget = targetPosition;
        for (int i = 0; i < 10; i++) {
            double dx = virtualTarget.getX() - launchPosition.getX();
            double dy = virtualTarget.getY() - launchPosition.getY();
            double flightTime = flightTimeTable.get(Math.hypot(dx, dy));

            // The feeder's push depends on where the turret ends up pointing, so it is recomputed
            // from the current aim each pass rather than hoisted out of the loop.
            double turretAngleRad = Math.atan2(dy, dx) - robotHeadingRad;
            double push = AimSolver.feederPushForwardMps(turretAngleRad, tuning);

            virtualTarget = new Translation3d(
                targetPosition.getX() - (motionDriftX + push * cosH) * flightTime,
                targetPosition.getY() - (motionDriftY + push * sinH) * flightTime,
                targetPosition.getZ());
        }
        return virtualTarget;
    }

    /**
     * Returns the horizontal distance from the launch position to a field target.
     */
    private double getDistanceToTarget(Translation3d targetPosition, Translation3d launchPosition) {
        return Math.hypot(
            targetPosition.getX() - launchPosition.getX(),
            targetPosition.getY() - launchPosition.getY());
    }

    /**
     * Computes and applies the turret rotation, rack angle, and flywheel velocity
     * needed to hit a 3D field target, using empirical lookup tables and
     * compensating for robot motion during flight.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public void calculateShooterActions(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Translation3d launchPosition = calculateLaunchPosition(robotPose);
        Translation2d launchPointVel = calculateLaunchPointVelocity(robotPose);

        double distance = getDistanceToTarget(targetPosition, launchPosition);
        double rackAngleDeg = rackAngleTable.get(distance);
        double flywheelMotorRPS = flywheelRPSTable.get(distance);

        Translation3d virtualTarget = calculateVirtualTargetPosition(
            targetPosition, launchPosition, launchPointVel, robotPose.getRotation().getRadians());

        // Turret: field-relative angle to virtual target, converted to robot-relative
        double dx = virtualTarget.getX() - launchPosition.getX();
        double dy = virtualTarget.getY() - launchPosition.getY();
        double turretAngleDeg = Math.toDegrees(Math.atan2(dy, dx))
            - robotPose.getRotation().getDegrees();
        double horizontalDist = Math.hypot(dx, dy);

        // Re-lookup shooter parameters from virtual target distance
        rackAngleDeg = rackAngleTable.get(horizontalDist);
        flywheelMotorRPS = flywheelRPSTable.get(horizontalDist);

        SmartDashboard.putNumber("Shooter/TurretAngleDeg", turretAngleDeg);
        SmartDashboard.putNumber("Shooter/ShooterX", launchPosition.getX());
        SmartDashboard.putNumber("Shooter/ShooterY", launchPosition.getY());
        SmartDashboard.putNumber("Shooter/TargetX", virtualTarget.getX());
        SmartDashboard.putNumber("Shooter/TargetY", virtualTarget.getY());
        SmartDashboard.putNumber("Shooter/HorizontalDist", horizontalDist);
        SmartDashboard.putNumber("Shooter/Distance", distance);
        SmartDashboard.putNumber("Shooter/RackAngleDeg", rackAngleDeg);
        SmartDashboard.putNumber("Shooter/FlywheelMotorRPS", flywheelMotorRPS);

        setShooterRackAngle(rackAngleDeg);
        setTurretRotatorAngle(turretAngleDeg);
        setShooterFlywheelVelocity(flywheelMotorRPS);
    }

    /**
     * Solves for a firing solution against a field target without commanding anything.
     *
     * <p>Separate from {@link #calculatePhysicsShooterActions} so a solution can be inspected — is
     * the shot even achievable from here? — without moving a motor.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public AimSolution solveAim(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Translation3d launchPosition = calculateLaunchPosition(robotPose);
        Translation2d launchPointVel = calculateLaunchPointVelocity(robotPose);

        return AimSolver.solve(
            targetPosition,
            launchPosition,
            robotPose.getRotation().getRadians(),
            launchPointVel.getX(),
            launchPointVel.getY(),
            m_tuning.snapshot());
    }

    /**
     * Computes turret angle, rack angle, and flywheel speed entirely from projectile physics (no
     * lookup tables) and commands the mechanism to match. See {@link AimSolver} for the math.
     *
     * <p>The robot velocity used for shoot-on-the-move comes from {@link VelocityEstimator}, which
     * leads with the Pigeon 2's accelerometer rather than wheel odometry — the ball leaves the
     * shooter in the moment the robot changes direction, which is exactly when wheel odometry is
     * least trustworthy.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public void calculatePhysicsShooterActions(Translation3d targetPosition) {
        AimSolution solution = solveAim(targetPosition);
        m_lastSolution = solution;
        publishAimTelemetry(solution);

        setShooterRackAngle(solution.rackAngleDeg());
        setTurretRotatorAngle(solution.turretAngleDeg());
        setShooterFlywheelVelocity(solution.flywheelRps());
    }

    /** Most recent firing solution, or null if none has been computed yet. */
    public AimSolution getLastSolution() {
        return m_lastSolution;
    }

    /**
     * Publishes the solution alongside what the mechanism actually did with it. The error rows are
     * the ones that matter when a shot misses: a solution that was correct but never reached (rack
     * still travelling, flywheel not spun up) looks nothing like a solution that was wrong.
     */
    private void publishAimTelemetry(AimSolution s) {
        SmartDashboard.putNumber("Shooter/Physics/TurretAngleDeg", s.turretAngleDeg());
        SmartDashboard.putNumber("Shooter/Physics/RackAngleDeg", s.rackAngleDeg());
        SmartDashboard.putNumber("Shooter/Physics/FlywheelMotorRPS", s.flywheelRps());
        SmartDashboard.putNumber("Shooter/Physics/LaunchSpeedMPS", s.launchSpeedMps());
        SmartDashboard.putNumber("Shooter/Physics/LaunchAngleDeg", s.launchAngleDeg());
        SmartDashboard.putNumber("Shooter/Physics/HorizontalDist", s.horizontalDistM());
        SmartDashboard.putNumber("Shooter/Physics/FlightTimeSec", s.flightTimeS());

        SmartDashboard.putNumber("Shooter/Physics/TurretErrorDeg",
            s.turretAngleDeg() - getTurretRotatorAngleDeg());
        SmartDashboard.putNumber("Shooter/Physics/RackErrorDeg",
            s.rackAngleDeg() - getShooterRackAngleDeg());
        SmartDashboard.putNumber("Shooter/Physics/FlywheelErrorRPS",
            s.flywheelRps() - getShooterFlywheelVelocityRps());

        SmartDashboard.putBoolean("Shooter/Physics/RackClamped", s.rackClamped());
        SmartDashboard.putBoolean("Shooter/Physics/SpeedClamped", s.speedClamped());
        SmartDashboard.putBoolean("Shooter/Physics/Feasible", s.feasible());
        SmartDashboard.putBoolean("Shooter/Physics/Achievable", s.achievable());
    }

    /**
     * Converts a turret angle in degrees to motor rotations.
     * Unwraps the angle relative to the last commanded position so the turret tracks
     * smoothly past the ±180° atan2 boundary. When the unwrapped angle exceeds a
     * physical limit, it wraps 360° to the other side of the range.
     */
    public double angleToTurretPosition(double angleDeg) {
        final double min = Constants.ShooterConstants.TURRET_ROTATOR_MIN_ANGLE;
        final double max = Constants.ShooterConstants.TURRET_ROTATOR_MAX_ANGLE;

        // Unwrap: find the equivalent angle closest to the last commanded angle
        double delta = ((angleDeg - m_lastCommandedTurretAngle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        double target = m_lastCommandedTurretAngle + delta;

        // If the unwrapped angle exceeds a limit, swap to the other side
        if (target > max) {
            target -= 360.0;
        } else if (target < min) {
            target += 360.0;
        }

        // Safety clamp (shouldn't activate with >360° range)
        target = Math.max(min, Math.min(max, target));

        m_lastCommandedTurretAngle = target;
        return target / 360.0 / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;
    }

    /**
     * Converts a rack angle in degrees to motor rotations.
     * Clamps the angle to [RACK_MIN_ANGLE, RACK_MAX_ANGLE].
     */
    public static double angleToRackPosition(double angleDeg) {
        double clamped = Math.max(Constants.ShooterConstants.RACK_MIN_ANGLE,
            Math.min(Constants.ShooterConstants.RACK_MAX_ANGLE, angleDeg));
        return (clamped - Constants.ShooterConstants.RACK_MIN_ANGLE)
            / 360.0 / Constants.ShooterConstants.RACK_GEAR_RATIO;
    }

    /**
     * Rotates the turret to face a field-relative 3D target.
     * Does not command the rack or flywheel.
     *
     * @param targetPosition field-relative 3D position of the target
     */
    public void aimTurretAt(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Translation3d launchPosition = calculateLaunchPosition(robotPose);
        double dx = targetPosition.getX() - launchPosition.getX();
        double dy = targetPosition.getY() - launchPosition.getY();
        double turretAngleDeg = Math.toDegrees(Math.atan2(dy, dx))
            - robotPose.getRotation().getDegrees();
        setTurretRotatorAngle(turretAngleDeg);
    }

    /** Returns the current turret rotator angle in degrees (same convention as {@link #setTurretRotatorAngle}). */
    public double getTurretRotatorAngleDeg() {
        return m_turretRotatorMotor.getPosition().getValueAsDouble()
            * Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO * 360.0;
    }

    /** Returns the current shooter rack angle in degrees (same convention as {@link #setShooterRackAngle}). */
    public double getShooterRackAngleDeg() {
        return m_shooterRackMotor.getPosition().getValueAsDouble()
            * Constants.ShooterConstants.RACK_GEAR_RATIO * 360.0
            + Constants.ShooterConstants.RACK_MIN_ANGLE;
    }

    public void setTurretRotatorPosition(double position) {
        m_lastCommandedTurretRotorPosition = position;
        m_turretRotatorMotor.setControl(new PositionVoltage(position));
    }

    /**
     * Sets turret rotator position from a target angle in degrees, with omega feedforward.
     * When the robot is rotating, the turret must counter-rotate to stay field-locked.
     * The velocity feedforward (kV × motorVelRps) pre-applies voltage to overcome friction
     * and inertia before the PID error has time to build up.
     */
    public void setTurretRotatorAngle(double angleDeg) {
        double position = angleToTurretPosition(angleDeg);
        m_lastCommandedTurretRotorPosition = position;
        // Counter-rotation: turret must spin at -omega to maintain field-relative aim.
        // Convert rad/s → turret rot/s → motor rot/s (gear ratio is negative, so signs cancel).
        double omega = m_velocityEstimator.getYawRateRadPerSec();
        double motorVelRps = -omega / (2.0 * Math.PI) / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;
        m_turretRotatorMotor.setControl(new PositionVoltage(position).withVelocity(motorVelRps));
    }

    public void setShooterRackPosition(double position) {
        m_lastCommandedRackRotorPosition = position;
        m_shooterRackMotor.setControl(new PositionVoltage(position));
    }

    /** Sets rack position from a target angle in degrees. */
    public void setShooterRackAngle(double angleDeg) {
        setShooterRackPosition(angleToRackPosition(angleDeg));
    }

    public void setShooterFlywheelVelocity(double velocity) {
        double clamped = Math.max(-Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC,
            Math.min(Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC, velocity));
        m_shooterFlywheelMotor.setControl(new VelocityVoltage(clamped));
    }

    /** Returns the current flywheel speed in motor rotations per second (FLYWHEEL_GEAR_RATIO is 1, direct drive). */
    public double getShooterFlywheelVelocityRps() {
        return m_shooterFlywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Launches a simulated fuel projectile from the shooter's current aim (turret angle, rack
     * angle, flywheel speed) using maple-sim's built-in 2026 physics — including shoot-on-the-move
     * compensation from the drivetrain's current field-relative velocity. No-op on a real robot.
     *
     * <p>Note: this uses BALL_LAUNCH_FRONT_OFFSET_METERS/LATERAL_OFFSET_METERS rotated by the
     * turret's full field-relative facing (robot heading + turret angle), which is slightly more
     * accurate than calculateLaunchPosition()'s robot-heading-only rotation — a small (~0.11 m)
     * difference in where the projectile visually spawns, not in aim direction.
     */
    public void launchProjectile() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        // Use maple-sim's true simulated pose, not the odometry estimate (getState().Pose) —
        // odometry can drift (e.g. from wheel slip), and launching from where the robot actually
        // is matters more here than being consistent with what the robot's own sensors believe.
        Pose2d robotPose = m_drivetrain.getSimulatedGroundTruthPose().orElse(m_drivetrain.getState().Pose);
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, robotPose.getRotation());

        Rotation2d shooterFacing = robotPose.getRotation().plus(Rotation2d.fromDegrees(getTurretRotatorAngleDeg()));
        double launchSpeedMPS = Math.PI * m_tuning.snapshot().flywheelDiameterMeters()
            * getShooterFlywheelVelocityRps();
        double shooterAngleDeg = 90.0 - getShooterRackAngleDeg();

        SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new Translation2d(
                m_tuning.launchForwardOffsetMeters(),
                m_tuning.launchLeftOffsetMeters()),
            fieldRelativeSpeeds,
            shooterFacing,
            Units.Meters.of(m_tuning.launchHeightMeters()),
            Units.MetersPerSecond.of(launchSpeedMPS),
            Units.Degrees.of(shooterAngleDeg)
        ));
    }

    /** Seeds the turret encoder on first enable to account for the 15° rightward offset at boot. */
    public void zeroTurretEncoderOnce() {
        if (!m_turretZeroed) {
            // Turret physically points 15° to the right (−15°) when motor reads 0.
            m_turretRotatorMotor.setPosition(
                -0 / 360.0 / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO);
            m_turretZeroed = true;
        }
    }

    public void stop() {
        m_turretRotatorMotor.setControl(new StaticBrake());
        m_shooterRackMotor.setControl(new NeutralOut());
        m_shooterFlywheelMotor.setControl(new NeutralOut());
    }

    public Command disabledCommand() {
        return run(() -> {
            m_turretRotatorMotor.setControl(new NeutralOut());
            m_shooterRackMotor.setControl(new NeutralOut());
            m_shooterFlywheelMotor.setControl(new NeutralOut());
        }).ignoringDisable(true);
    }

    public Command maintainStateCommand() {
        return run(() -> {
            setTurretRotatorPosition(0);
            setShooterRackPosition(0);
            setShooterFlywheelVelocity(0);
        });
    }

    @Override
    public void periodic() {
        // Re-applies on any change, which includes tuning mode being switched off — the gains snap
        // back to the compiled constants the same way every other tunable does.
        double kP = m_turretKP.get();
        double kI = m_turretKI.get();
        double kD = m_turretKD.get();
        double kV = m_turretKV.get();
        if (kP != m_appliedTurretKP || kI != m_appliedTurretKI
                || kD != m_appliedTurretKD || kV != m_appliedTurretKV) {
            m_appliedTurretKP = kP;
            m_appliedTurretKI = kI;
            m_appliedTurretKD = kD;
            m_appliedTurretKV = kV;
            m_turretRotatorMotor.getConfigurator().apply(
                new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV));
        }
    }

    @Override
    public void simulationPeriodic() {
        updateTurretSim();
        updateRackSim();
        updateFlywheelSim();
    }

    /**
     * Snaps the simulated turret straight to its last commanded position instead of running a
     * physics model through it — see the note on the simulation fields above for why.
     */
    private void updateTurretSim() {
        m_turretRotatorMotor.getSimState().setRawRotorPosition(m_lastCommandedTurretRotorPosition);
    }

    /**
     * Snaps the simulated rack straight to its last commanded position instead of running a
     * physics model through it — see the note on the simulation fields above for why.
     */
    private void updateRackSim() {
        m_shooterRackMotor.getSimState().setRawRotorPosition(m_lastCommandedRackRotorPosition);
    }

    /** Steps the flywheel's physics model and feeds its velocity back into the TalonFX simulation state. */
    private void updateFlywheelSim() {
        TalonFXSimState simState = m_shooterFlywheelMotor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_flywheelSim.setInputVoltage(simState.getMotorVoltage());
        m_flywheelSim.update(kSimPeriodSeconds);

        double rotorRotationsPerSec = m_flywheelSim.getAngularVelocityRadPerSec()
            / (2 * Math.PI) * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
        simState.setRotorVelocity(rotorRotationsPerSec);
    }
}
