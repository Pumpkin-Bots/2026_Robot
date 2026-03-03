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
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
        .withKD(Constants.ShooterConstants.ROTATOR_KD);

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

    private static final double GRAVITY = 9.80665; // m/s²

    // Lookup tables: distance (meters) → value. Populate with empirical test shots.
    private static final InterpolatingDoubleTreeMap rackAngleTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPSTable = new InterpolatingDoubleTreeMap();
    static {
        // TODO: Fill in from test shots — put(distance_meters, rack_angle_deg)
        rackAngleTable.put(1.3589, 15.0);
        rackAngleTable.put(1.7018, 15.0);
        rackAngleTable.put(2.3495, 17.0);
        rackAngleTable.put(3.0099, 19.0);
        rackAngleTable.put(3.7211, 23.0);
        rackAngleTable.put(4.0767, 27.0);
        rackAngleTable.put(5.9182, 35.0);
        rackAngleTable.put(5.9182, 37.0);


        // TODO: Fill in from test shots — put(distance_meters, flywheel_motor_RPS)
        flywheelRPSTable.put(1.3589, 27.0);
        flywheelRPSTable.put(1.7018, 28.0);
        flywheelRPSTable.put(2.3495, 30.0);
        flywheelRPSTable.put(3.0099, 32.5);
        flywheelRPSTable.put(3.7211, 33.5);
        flywheelRPSTable.put(4.0767, 34.5);
        flywheelRPSTable.put(5.9182, 40.5);
        flywheelRPSTable.put(6.2484, 42.0);

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
     * accounting for the robot's heading and the forward/height offset from center.
     */
    private Translation3d calculateLaunchPosition(Pose2d robotPose) {
        double heading = robotPose.getRotation().getRadians();
        return new Translation3d(
            robotPose.getX() + Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS * Math.cos(heading),
            robotPose.getY() + Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS * Math.sin(heading),
            Constants.ShooterConstants.BALL_LAUNCH_HEIGHT_METERS);
    }

    /**
     * Computes and applies the turret rotation, rack angle, and flywheel velocity
     * needed to hit a 3D field target, compensating for robot motion and gravity.
     *
     * Flight time is derived from the vertical projectile equation using the known
     * height difference, which guarantees the ball is descending when it hits the
     * target (larger root of the quadratic). Table lookups are re-queried each
     * iteration using the virtual target distance — the distance the ball travels
     * in the muzzle direction — so rack angle and flywheel speed stay consistent
     * with the actual compensated shot. Converges in 3 passes since robot speed
     * is much smaller than ball speed.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public void calculateShooterActions(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, robotPose.getRotation());
        Translation3d launchPosition = calculateLaunchPosition(robotPose);

        double dx = targetPosition.getX() - launchPosition.getX();
        double dy = targetPosition.getY() - launchPosition.getY();
        double dz = targetPosition.getZ() - launchPosition.getZ();
        double vx = fieldRelativeSpeeds.vxMetersPerSecond;
        double vy = fieldRelativeSpeeds.vyMetersPerSecond;

        // Start with real horizontal distance; refine toward virtual target distance.
        double lookupDist = Math.hypot(dx, dy);
        double rackAngleDeg = 0;
        double flywheelMotorRPS = 0;
        double flightTime = 0;
        double aimX = 0;
        double aimY = 0;

        for (int i = 0; i < 3; i++) {
            rackAngleDeg     = rackAngleTable.get(lookupDist);
            flywheelMotorRPS = flywheelRPSTable.get(lookupDist);

            double muzzleSpeed = flywheelMotorRPS
                * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO
                * Math.PI * Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS;
            double sv = muzzleSpeed * Math.sin(Math.toRadians(rackAngleDeg));

            // Vertical projectile equation: dz = sv·t - ½g·t²
            // Rearranged: (g/2)t² - sv·t + dz = 0
            // t = (sv ± √(sv² - 2g·dz)) / g
            // Larger root → ball is descending when it reaches the target.
            double discriminant = sv * sv - 2.0 * GRAVITY * dz;
            if (discriminant < 0) {
                // Rack angle too shallow to reach target height at this distance.
                SmartDashboard.putBoolean("Shooter/CanReachTarget", false);
                return;
            }
            flightTime = (sv + Math.sqrt(discriminant)) / GRAVITY;

            // Required muzzle velocity in the field frame to reach the real goal in flightTime.
            // Gain scales the velocity offset to compensate for unmodeled effects.
            double gain = Constants.ShooterConstants.MOTION_COMPENSATION_GAIN;
            aimX = dx / flightTime - vx * gain;
            aimY = dy / flightTime - vy * gain;

            // Virtual target distance: how far the ball travels in the muzzle direction.
            // When robot is stationary this equals the real horizontal distance exactly.
            lookupDist = Math.hypot(aimX, aimY) * flightTime;
        }

        double turretAngleDeg = Math.toDegrees(Math.atan2(aimY, aimX))
            - robotPose.getRotation().getDegrees();

        SmartDashboard.putBoolean("Shooter/CanReachTarget", true);
        SmartDashboard.putNumber("Shooter/TurretAngleDeg", turretAngleDeg);
        SmartDashboard.putNumber("Shooter/ShooterX", launchPosition.getX());
        SmartDashboard.putNumber("Shooter/ShooterY", launchPosition.getY());
        SmartDashboard.putNumber("Shooter/TargetX", targetPosition.getX());
        SmartDashboard.putNumber("Shooter/TargetY", targetPosition.getY());
        SmartDashboard.putNumber("Shooter/HorizontalDist", Math.hypot(dx, dy));
        SmartDashboard.putNumber("Shooter/LookupDist", lookupDist);
        SmartDashboard.putNumber("Shooter/FlightTime", flightTime);
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

    /** Sets turret rotator position from a target angle in degrees. */
    public void setTurretRotatorAngle(double angleDeg) {
        setTurretRotatorPosition(angleToTurretPosition(angleDeg));
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
