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
     * Computes a virtual target position that compensates for robot motion during
     * projectile flight time. Uses 5-step iterative refinement, solving for the
     * actual required muzzle velocity at each step so flight time is self-consistent
     * with the arc (rack angle) rather than assuming maximum speed.
     *
     * @param targetPosition      field-relative 3D position of the actual target
     * @param launchPosition      field-relative 3D position of the ball at launch
     * @param fieldRelativeSpeeds robot velocity in field-relative coordinates
     * @param rackAngleDeg        elevation angle in degrees
     * @return adjusted 3D aim point that accounts for robot drift during flight
     */
    private Translation3d calculateVirtualTargetPosition(
            Translation3d targetPosition,
            Translation3d launchPosition,
            ChassisSpeeds fieldRelativeSpeeds,
            double rackAngleDeg) {
        double avgDiameter = Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS;
        // elevation = 90° - rackAngle: rack angle is measured from vertical, not horizontal
        double elevAngleRad = Math.toRadians(90.0 - rackAngleDeg);
        double cosElev = Math.cos(elevAngleRad);
        double tanElev = Math.tan(elevAngleRad);
        double dz = targetPosition.getZ() - launchPosition.getZ();

        // Seed with max muzzle speed for the first flight-time estimate
        double maxMuzzleSpeed = Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC
            * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO
            * Math.PI * avgDiameter;
        double horizontalSpeed = maxMuzzleSpeed * cosElev;

        Translation3d virtualTarget = targetPosition;
        for (int i = 0; i < 5; i++) {
            double dx = virtualTarget.getX() - launchPosition.getX();
            double dy = virtualTarget.getY() - launchPosition.getY();
            double horizontalDist = Math.hypot(dx, dy);
            double flightTime = horizontalDist / horizontalSpeed;

            virtualTarget = new Translation3d(
                targetPosition.getX() - fieldRelativeSpeeds.vxMetersPerSecond * flightTime,
                targetPosition.getY() - fieldRelativeSpeeds.vyMetersPerSecond * flightTime,
                targetPosition.getZ());

            // Recompute muzzle speed from projectile physics at this distance
            // so the next iteration's flight time uses the actual required speed.
            double denominator = 2.0 * cosElev * cosElev * (horizontalDist * tanElev - dz);
            if (denominator > 0) {
                double muzzleVelocity = Math.sqrt(9.80665 * horizontalDist * horizontalDist / denominator);
                horizontalSpeed = Math.min(muzzleVelocity, maxMuzzleSpeed) * cosElev;
            }
        }
        return virtualTarget;
    }

    /**
     * Computes the rack angle in degrees for a given target distance without
     * commanding any motor. Use this value to pass to {@link #calculateShooterActions}.
     *
     * @param targetPosition field-relative 3D position of the actual target
     * @return interpolated rack angle in degrees
     */
    public double computeRackAngleDeg(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        double distance = Math.hypot(
            targetPosition.getX() - robotPose.getX(),
            targetPosition.getY() - robotPose.getY());

        double t = (distance - Constants.ShooterConstants.RACK_MIN_DISTANCE_METERS)
            / (Constants.ShooterConstants.RACK_MAX_DISTANCE_METERS
               - Constants.ShooterConstants.RACK_MIN_DISTANCE_METERS);
        t = Math.max(0.0, Math.min(1.0, t));

        return Constants.ShooterConstants.RACK_MIN_ANGLE
            + t * (Constants.ShooterConstants.RACK_MAX_ANGLE - Constants.ShooterConstants.RACK_MIN_ANGLE);
    }

    /**
     * Commands the rack motor to the distance-interpolated angle.
     * Call {@link #computeRackAngleDeg} first to get the angle for
     * passing to {@link #calculateShooterActions}.
     *
     * @param targetPosition field-relative 3D position of the actual target
     */
    public void applyRackAngle(Translation3d targetPosition) {
        setShooterRackAngle(computeRackAngleDeg(targetPosition));
    }

    /**
     * Computes and applies the turret rotation and flywheel velocity needed to hit
     * a 3D field target from the ball's launch position, compensating for robot
     * motion during flight.
     *
     * <p>The rack angle is a parameter supplied by an independent function (e.g. a
     * lookup table or optimizer). That function is also responsible for commanding
     * the rack motor — this method does not touch it.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     * @param rackAngleDeg   elevation angle in degrees, supplied by an external function
     */
    public void calculateShooterActions(Translation3d targetPosition, double rackAngleDeg) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, robotPose.getRotation());
        Translation3d launchPosition = calculateLaunchPosition(robotPose);

        // physicsRackAngleDeg is used only for trajectory calculations (virtual target
        // compensation and flywheel velocity). The rack motor is commanded at rackAngleDeg
        // so the physical angle is unaffected by the trim.
        double physicsRackAngleDeg = rackAngleDeg + Constants.ShooterConstants.RACK_ANGLE_TRIM_DEG;

        Translation3d virtualTarget = calculateVirtualTargetPosition(
            targetPosition, launchPosition, fieldRelativeSpeeds, physicsRackAngleDeg);

        // Turret: field-relative angle to virtual target, converted to robot-relative
        double dx = virtualTarget.getX() - launchPosition.getX();
        double dy = virtualTarget.getY() - launchPosition.getY();
        double turretAngleDeg = Math.toDegrees(Math.atan2(dy, dx))
            - robotPose.getRotation().getDegrees();

        // Flywheel: solve projectile physics for required muzzle velocity
        // v = sqrt( g * d² / (2 * cos²θ * (d·tanθ − dz)) )
        // elevation = 90° - rackAngle: rack angle is measured from vertical, not horizontal
        double horizontalDist = Math.hypot(dx, dy);
        double dz = virtualTarget.getZ() - launchPosition.getZ();
        double elevAngleRad = Math.toRadians(90.0 - physicsRackAngleDeg);
        double cosElev = Math.cos(elevAngleRad);
        double denominator = 2.0 * cosElev * cosElev
            * (horizontalDist * Math.tan(elevAngleRad) - dz);

        double flywheelMotorRPS;
        if (denominator <= 0) {
            // Trajectory is physically infeasible at this rack angle — use max speed
            flywheelMotorRPS = Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC;
        } else {
            double avgDiameter = Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS;
            double muzzleVelocity = Math.sqrt(
                9.80665 * horizontalDist * horizontalDist / denominator);
            double baseVelocityRPS = Constants.ShooterConstants.kBaseVelocity
                / (Math.PI * avgDiameter * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO);
            flywheelMotorRPS = muzzleVelocity
                / (Math.PI * avgDiameter * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO)
                - baseVelocityRPS;
            flywheelMotorRPS = Math.min(
                flywheelMotorRPS, Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC);
        }

        SmartDashboard.putNumber("Shooter/TurretAngleDeg", turretAngleDeg);
        SmartDashboard.putNumber("Shooter/ShooterX", launchPosition.getX());
        SmartDashboard.putNumber("Shooter/ShooterY", launchPosition.getY());
        SmartDashboard.putNumber("Shooter/TargetX", virtualTarget.getX());
        SmartDashboard.putNumber("Shooter/TargetY", virtualTarget.getY());
        SmartDashboard.putNumber("Shooter/HorizontalDist", horizontalDist);
        SmartDashboard.putNumber("Shooter/RackAngleDeg", rackAngleDeg);
        SmartDashboard.putNumber("Shooter/PhysicsRackAngleDeg", physicsRackAngleDeg);
        SmartDashboard.putNumber("Shooter/FlywheelMotorRPS", flywheelMotorRPS);
        SmartDashboard.putNumber("Shooter/MuzzleVelocity",
            flywheelMotorRPS <= 0 ? 0
                : (flywheelMotorRPS + Constants.ShooterConstants.kBaseVelocity
                    / (Math.PI * Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS
                        * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO))
                    * Math.PI * Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS
                    * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO);

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

    /**
     * Back-calculates the effective flywheel diameter from a test shot where the
     * ball landed on the floor at a known horizontal distance from the launch point.
     *
     * <p>Procedure:
     * <ol>
     *   <li>Park the robot at a fixed, measured position.</li>
     *   <li>Command a known rack angle and a known motor RPS (use
     *       {@link #setShooterRackAngle} and {@link #setShooterFlywheelVelocity}).</li>
     *   <li>Fire the ball and measure the horizontal floor distance from the launch
     *       point to where it landed.</li>
     *   <li>Pass those values here along with {@code BALL_LAUNCH_HEIGHT_METERS}.</li>
     *   <li>Copy the returned value into {@code FLYWHEEL_EFFECTIVE_DIAMETER_METERS}
     *       and set {@code kBaseVelocity} to {@code 0.0}.</li>
     * </ol>
     *
     * @param rackAngleDeg      rack angle used during the test shot (degrees from vertical)
     * @param motorRPS          motor RPS commanded during the test shot
     * @param launchHeightMeters height of the ball at launch above the floor (meters)
     * @param landingDistMeters measured horizontal distance from launch point to
     *                          where the ball hit the floor (meters)
     * @return effective flywheel diameter in meters, or {@code -1} if the inputs are
     *         physically inconsistent
     */
    public static double calculateEffectiveDiameter(
            double rackAngleDeg,
            double motorRPS,
            double launchHeightMeters,
            double landingDistMeters) {
        double elevAngleRad = Math.toRadians(90.0 - rackAngleDeg);
        double cosElev = Math.cos(elevAngleRad);
        // dz is negative because the ball lands below the launch point
        double dz = -launchHeightMeters;
        double denominator = 2.0 * cosElev * cosElev
            * (landingDistMeters * Math.tan(elevAngleRad) - dz);
        if (denominator <= 0 || motorRPS <= 0) {
            return -1;
        }
        double actualMuzzleVelocity = Math.sqrt(
            9.80665 * landingDistMeters * landingDistMeters / denominator);
        return actualMuzzleVelocity
            / (Math.PI * Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO * motorRPS);
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
