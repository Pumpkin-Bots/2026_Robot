package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class ShooterSubsystem implements Subsystem {

    private final TalonFX m_turretRotatorMotor;
    private final TalonFX m_shooterRackMotor;
    private final TalonFX m_shooterFlywheelMotor;
    private final CommandSwerveDrivetrain m_drivetrain;
    private boolean m_turretZeroed = false;
    private double m_lastCommandedTurretAngle = 0.0;

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
        .withKD(Constants.ShooterConstants.FLYWHEEL_KD);

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
        .withSupplyCurrentLimit(40);

    // Lookup tables: distance (meters) → value. Populate with empirical test shots.
    private static final InterpolatingDoubleTreeMap rackAngleTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPSTable = new InterpolatingDoubleTreeMap();

    // Flight time table: distance (meters) → measured time-of-flight (seconds).
    // Measure by recording frame timestamps from ball leave to scoring event, or
    // use slow-motion video. Replace placeholder values with measured data.
    private static final InterpolatingDoubleTreeMap flightTimeTable = new InterpolatingDoubleTreeMap();

    static {
        // TODO: Fill in from test shots — put(distance_meters, rack_angle_deg)
        rackAngleTable.put(1.3589, 15.0);
        rackAngleTable.put(1.7018, 15.0);
        rackAngleTable.put(2.3495, 17.0);
        rackAngleTable.put(3.0099, 19.0);
        rackAngleTable.put(3.7211, 23.0);
        rackAngleTable.put(4.0767, 27.0);
        rackAngleTable.put(4.4, 29.0);
        // Above 5 m: 1-meter intervals. Angle approaches hardware max (42°) by 8 m.
        // launch_angle = 90° - rack_angle; higher rack = flatter shot, lower rack = steeper lob.
        rackAngleTable.put(6.0, 38.0);
        rackAngleTable.put(7.0, 41.0);
        rackAngleTable.put(8.0, 42.0);
        rackAngleTable.put(9.0, 42.0);
        rackAngleTable.put(10.0, 42.0);
        rackAngleTable.put(11.0, 42.0);
        rackAngleTable.put(12.0, 42.0);
        rackAngleTable.put(13.0, 42.0);
        rackAngleTable.put(14.0, 42.0);
        rackAngleTable.put(15.0, 42.0);


        // TODO: Fill in from test shots — put(distance_meters, flywheel_motor_RPS)
        flywheelRPSTable.put(1.3589, 27.0);
        flywheelRPSTable.put(1.7018, 26.0);
        flywheelRPSTable.put(2.10, 29.0);
        flywheelRPSTable.put(2.3495, 32.0);
        flywheelRPSTable.put(3.0, 33.75);
        flywheelRPSTable.put(3.5, 33.75);
        flywheelRPSTable.put(4.4, 33.75);
        flywheelRPSTable.put(5.0, 38.0);
        // Above 5 m: physics-derived at ~80% flywheel efficiency.
        // v0_req = d / (sin(rack) * t); RPS = v0_req / (0.80 * pi * 0.0762)
        flywheelRPSTable.put(6.0, 44.0);
        flywheelRPSTable.put(7.0, 47.0);
        flywheelRPSTable.put(8.0, 50.0);
        flywheelRPSTable.put(9.0, 52.5);
        flywheelRPSTable.put(10.0, 55.0);
        flywheelRPSTable.put(11.0, 57.5);
        flywheelRPSTable.put(12.0, 59.5);
        flywheelRPSTable.put(13.0, 62.0);
        flywheelRPSTable.put(14.0, 64.0);
        flywheelRPSTable.put(15.0, 66.0);

        // Physics-based flight times: t = sqrt(2 * (d*tan(launch) - Δh) / g)
        // where launch = 90° - rack_angle (rack measured from vertical; rack=0° = horizontal).
        // Δh = target_height - launch_height = 1.6764m (5.5 ft) - 0.4826m = 1.1938m.
        // All shots arc high and descend into the target from above.
        // TODO: Refine with measured values from slow-motion video or timestamp logging.
        flightTimeTable.put(1.3589, 0.89);
        flightTimeTable.put(1.7018, 1.03);
        flightTimeTable.put(2.3495, 1.15);
        flightTimeTable.put(3.0099, 1.24);
        flightTimeTable.put(3.7211, 1.24);
        flightTimeTable.put(4.0767, 1.18);
        flightTimeTable.put(5.0,    1.17);
        flightTimeTable.put(6.0,    1.15);
        flightTimeTable.put(7.0,    1.18);
        flightTimeTable.put(8.0,    1.25);
        flightTimeTable.put(9.0,    1.34);
        flightTimeTable.put(10.0,   1.42);
        flightTimeTable.put(11.0,   1.50);
        flightTimeTable.put(12.0,   1.57);
        flightTimeTable.put(13.0,   1.64);
        flightTimeTable.put(14.0,   1.71);
        flightTimeTable.put(15.0,   1.78);

    }

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        m_turretRotatorMotor = new TalonFX(Constants.ShooterConstants.TURRET_ROTATOR_ID);
        m_shooterRackMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_RACK_ID);
        m_shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_ID);

        m_turretRotatorMotor.getConfigurator().apply(turretRotatorGains);
        m_turretRotatorMotor.getConfigurator().apply(turretSoftLimits);
        m_shooterRackMotor.getConfigurator().apply(rackGains);
        m_shooterRackMotor.getConfigurator().apply(rackSoftLimits);
        m_shooterFlywheelMotor.getConfigurator().apply(flywheelGains);
        m_shooterFlywheelMotor.getConfigurator().apply(flywheelCurrentLimits);
    }

    /**
     * Computes the ball's launch position in field coordinates,
     * accounting for the robot's heading and the forward/lateral/height offset from center.
     */
    private Translation3d calculateLaunchPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        double rx = Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS;
        double ry = Constants.ShooterConstants.BALL_LAUNCH_LATERAL_OFFSET_METERS;
        return new Translation3d(
            robotPose.getX() + rx * Math.cos(heading) - ry * Math.sin(heading),
            robotPose.getY() + rx * Math.sin(heading) + ry * Math.cos(heading),
            Constants.ShooterConstants.BALL_LAUNCH_HEIGHT_METERS);
    }

    /**
     * Computes a virtual target position that compensates for robot motion and
     * turret-indexer spin during projectile flight time. Uses iterative refinement
     * with flight time from the empirical lookup table.
     *
     * <p>Because the turret is off-center, robot rotation adds a linear velocity
     * component at the launch point. The full launch-point velocity is:
     * <pre>
     *   v_launch_x = v_robot_x + (-rx·sin(θ) - ry·cos(θ))·ω
     *   v_launch_y = v_robot_y + ( rx·cos(θ) - ry·sin(θ))·ω
     * </pre>
     * where rx/ry are the forward/lateral offsets of the turret from robot center,
     * θ is the robot heading, and ω is the robot's angular velocity.
     *
     * <p>The indexer imparts spin on the ball that creates an effective extra velocity
     * component. The along-barrel component scales as cos(turretAngle) and the
     * perpendicular component (left/right from the turret's perspective) scales as
     * sin(turretAngle). Both are rotated into field coordinates and applied as
     * additional velocity offsets so the aim point corrects for spin drift.
     *
     * @param targetPosition      field-relative 3D position of the actual target
     * @param launchPosition      field-relative 3D position of the ball at launch
     * @param fieldRelativeSpeeds robot velocity in field-relative coordinates (vx, vy, omega)
     * @param robotHeadingRad     robot heading in radians (field-relative)
     * @return adjusted 3D aim point that accounts for robot drift and spin during flight
     */
    private Translation3d calculateVirtualTargetPosition(
            Translation3d targetPosition,
            Translation3d launchPosition,
            ChassisSpeeds fieldRelativeSpeeds,
            double robotHeadingRad) {
        // Compute the launch point's actual field-relative velocity.
        // Robot rotation adds a linear velocity at the turret because it is off-center.
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double rx = Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS;
        double ry = Constants.ShooterConstants.BALL_LAUNCH_LATERAL_OFFSET_METERS;
        double launchVelX = fieldRelativeSpeeds.vxMetersPerSecond
            + (-rx * Math.sin(robotHeadingRad) - ry * Math.cos(robotHeadingRad)) * omega;
        double launchVelY = fieldRelativeSpeeds.vyMetersPerSecond
            + ( rx * Math.cos(robotHeadingRad) - ry * Math.sin(robotHeadingRad)) * omega;

        Translation3d virtualTarget = targetPosition;
        for (int i = 0; i < 10; i++) {
            double dx = virtualTarget.getX() - launchPosition.getX();
            double dy = virtualTarget.getY() - launchPosition.getY();
            double horizontalDist = Math.hypot(dx, dy);
            double flightTime = flightTimeTable.get(horizontalDist);

            // Barrel direction in field frame and turret angle relative to robot
            double barrelAngleRad = Math.atan2(dy, dx);
            double turretAngleRad = barrelAngleRad - robotHeadingRad;

            // Spin drift distances along and perpendicular to the shot-path line:
            //   spinAlongBarrel > 0  →  ball lands further from turret than expected
            //   spinLeftOfBarrel > 0  →  ball drifts left of the shot path
            double spinAlongBarrel  = Constants.TurretConstants.INDEXER_SPIN_FORWARD_BACK_MAX_MS
                * Math.cos(turretAngleRad) * flightTime;
            double spinLeftOfBarrel = Constants.TurretConstants.INDEXER_SPIN_LEFT_RIGHT_MAX_MS
                * Math.sin(turretAngleRad) * flightTime;

            // Unit vectors in field frame: along barrel (toward target) and left of barrel
            double barrelX =  Math.cos(barrelAngleRad);
            double barrelY =  Math.sin(barrelAngleRad);
            double leftX   = -Math.sin(barrelAngleRad);
            double leftY   =  Math.cos(barrelAngleRad);

            // Shift virtual target opposite to launch-point velocity and spin drift
            virtualTarget = new Translation3d(
                targetPosition.getX()
                    - launchVelX * flightTime
                    - spinAlongBarrel  * barrelX
                    - spinLeftOfBarrel * leftX,
                targetPosition.getY()
                    - launchVelY * flightTime
                    - spinAlongBarrel  * barrelY
                    - spinLeftOfBarrel * leftY,
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
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, robotPose.getRotation());
        Translation3d launchPosition = calculateLaunchPosition(robotPose);

        double distance = getDistanceToTarget(targetPosition, launchPosition);
        double rackAngleDeg = rackAngleTable.get(distance);
        double flywheelMotorRPS = flywheelRPSTable.get(distance);

        Translation3d virtualTarget = calculateVirtualTargetPosition(
            targetPosition, launchPosition, fieldRelativeSpeeds, robotPose.getRotation().getRadians());

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

    public void setTurretRotatorPosition(double position) {
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
        // Counter-rotation: turret must spin at -omega to maintain field-relative aim.
        // Convert rad/s → turret rot/s → motor rot/s (gear ratio is negative, so signs cancel).
        double omega = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
        double motorVelRps = -omega / (2.0 * Math.PI) / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;
        m_turretRotatorMotor.setControl(new PositionVoltage(position).withVelocity(motorVelRps));
    }

    public void setShooterRackPosition(double position) {
        m_shooterRackMotor.setControl(new PositionVoltage(position));
    }

    /** Sets rack position from a target angle in degrees. */
    public void setShooterRackAngle(double angleDeg) {
        setShooterRackPosition(angleToRackPosition(angleDeg));
    }

    public void setShooterFlywheelVelocity(double velocity) {
        double clamped = Math.max(-Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC,
            Math.min(Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC, velocity));
        m_shooterFlywheelMotor.setControl(new VelocityDutyCycle(clamped));
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
}
