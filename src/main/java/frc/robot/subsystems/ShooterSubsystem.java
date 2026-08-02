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

import edu.wpi.first.math.MathUtil;
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

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterSubsystem implements Subsystem {

    private final TalonFX m_turretRotatorMotor;
    private final TalonFX m_shooterRackMotor;
    private final TalonFX m_shooterFlywheelMotor;
    private final CommandSwerveDrivetrain m_drivetrain;
    private boolean m_turretZeroed = false;
    private double m_lastCommandedTurretAngle = 0.0;

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
    // otherwise mean editing Constants, recompiling, and redeploying every time. Reading these
    // back each loop lets you drag a "Turret/kP" etc. slider on Shuffleboard/Glass/Elastic and
    // see the response change live instead.
    private double m_tunedTurretKP = Constants.ShooterConstants.ROTATOR_KP;
    private double m_tunedTurretKI = Constants.ShooterConstants.ROTATOR_KI;
    private double m_tunedTurretKD = Constants.ShooterConstants.ROTATOR_KD;
    private double m_tunedTurretKV = Constants.ShooterConstants.ROTATOR_KV;

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

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        m_turretRotatorMotor = new TalonFX(Constants.ShooterConstants.TURRET_ROTATOR_ID);
        m_shooterRackMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_RACK_ID);
        m_shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_ID);

        m_turretRotatorMotor.getConfigurator().apply(turretRotatorGains);
        m_turretRotatorMotor.getConfigurator().apply(turretSoftLimits);
        m_turretRotatorMotor.getConfigurator().apply(turretCurrentLimits);
        // Position is read back for mechanism telemetry/3D visualization, so request it explicitly.
        m_turretRotatorMotor.getPosition().setUpdateFrequency(50);
        m_turretRotatorMotor.optimizeBusUtilization();

        SmartDashboard.putNumber("Turret/kP", m_tunedTurretKP);
        SmartDashboard.putNumber("Turret/kI", m_tunedTurretKI);
        SmartDashboard.putNumber("Turret/kD", m_tunedTurretKD);
        SmartDashboard.putNumber("Turret/kV", m_tunedTurretKV);

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

    // Matches GamePieceProjectile.GRAVITY in maple-sim (org.ironmaple.simulation.gamepieces),
    // which uses 11 m/s^2 instead of real gravity (9.81) to compensate for the simulation
    // ignoring air drag. Using the same constant means this math's predicted trajectory matches
    // what the simulated projectile actually does. Once you have real-world test data, this is
    // one of the first constants to recalibrate — real air drag behaves differently from a
    // flat gravity fudge factor, especially at higher speeds/longer shots.
    private static final double PHYSICS_GRAVITY_MPS2 = 11.0;

    // How much steeper than the pure minimum-energy angle to aim. The minimum-energy angle hits
    // the target exactly at the apex of its arc (zero vertical velocity) — a knife's-edge case
    // that, especially at close range, can end up arriving on the way UP instead of down. Biasing
    // steeper guarantees the ball is still descending when it reaches the target, so it drops
    // into the hub rather than skimming the rim on the way up.
    private static final double DESCENT_MARGIN_DEG = 12.0;

    /**
     * Computes turret angle, rack angle, and flywheel speed entirely from projectile physics
     * (no lookup tables), including proper shoot-on-the-move compensation via vector subtraction:
     *
     * <ol>
     *   <li>Pick an elevation angle steeper than the minimum-energy ballistic angle (see
     *       {@link #DESCENT_MARGIN_DEG}) so the shot arrives descending, clamped to what the
     *       rack can achieve.
     *   <li>Solve for the exact launch speed that hits the target at that angle, from the
     *       no-air-resistance projectile range equation.
     *   <li>Build the resulting stationary-shot velocity as a 3D field-relative vector.
     *   <li>Subtract the launch point's current field-relative velocity (chassis translation
     *       plus the extra ground speed the off-center launch point picks up from chassis
     *       rotation) from that vector. What's left is the velocity the shooter itself must
     *       impart, relative to the moving robot, so that the ball's actual field-relative
     *       velocity matches the stationary solution and it hits the same target.
     *   <li>Convert that resulting vector back into turret angle, rack angle, and flywheel RPS.
     * </ol>
     *
     * <p>This assumes ideal conditions matching maple-sim's own physics (no air drag, no wheel
     * slip on the flywheel/ball interface, instantaneous flywheel response) — expect to need a
     * fudge factor or two once you have real test-shot data to compare against.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public void calculatePhysicsShooterActions(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, robotPose.getRotation());
        Translation3d launchPosition = calculateLaunchPosition(robotPose);

        double dx = targetPosition.getX() - launchPosition.getX();
        double dy = targetPosition.getY() - launchPosition.getY();
        double dz = targetPosition.getZ() - launchPosition.getZ();
        double horizontalDist = Math.hypot(dx, dy);
        double bearingToTargetRad = Math.atan2(dy, dx);

        // Minimum-energy ballistic angle for a target at (horizontalDist, dz) relative to the
        // launch point, biased steeper by DESCENT_MARGIN_DEG so the ball is guaranteed to be
        // descending on arrival, then clamped to the rack's achievable range (RACK_MIN/MAX_ANGLE,
        // converted via launch_angle = 90 - rack_angle).
        double minEnergyLaunchAngleRad = Math.PI / 4 + 0.5 * Math.atan2(dz, horizontalDist);
        double desiredLaunchAngleRad = minEnergyLaunchAngleRad + Math.toRadians(DESCENT_MARGIN_DEG);
        double flattestLaunchAngleRad = Math.toRadians(90.0 - Constants.ShooterConstants.RACK_MAX_ANGLE);
        double steepestLaunchAngleRad = Math.toRadians(90.0 - Constants.ShooterConstants.RACK_MIN_ANGLE);
        double launchAngleRad = MathUtil.clamp(desiredLaunchAngleRad, flattestLaunchAngleRad, steepestLaunchAngleRad);

        // Required speed to hit the target at this angle, from the no-drag projectile range
        // equation: dz = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta)), solved for v.
        double denominator = horizontalDist * Math.tan(launchAngleRad) - dz;
        if (denominator <= 0) {
            // Target unreachable at this angle (too steep a climb for the distance) — fall back
            // to the steepest achievable angle and accept an imperfect shot rather than NaN out.
            launchAngleRad = steepestLaunchAngleRad;
            denominator = Math.max(1e-6, horizontalDist * Math.tan(launchAngleRad) - dz);
        }
        double cosAngle = Math.cos(launchAngleRad);
        double stationarySpeedMPS = Math.sqrt(
            PHYSICS_GRAVITY_MPS2 * horizontalDist * horizontalDist / (2 * cosAngle * cosAngle * denominator));

        // Stationary-shot velocity as a field-relative 3D vector.
        double stationaryHorizontalSpeed = stationarySpeedMPS * cosAngle;
        Translation3d stationaryVelocity = new Translation3d(
            stationaryHorizontalSpeed * Math.cos(bearingToTargetRad),
            stationaryHorizontalSpeed * Math.sin(bearingToTargetRad),
            stationarySpeedMPS * Math.sin(launchAngleRad));

        // The launch point's own field-relative velocity: chassis translation, plus the extra
        // ground speed it picks up from chassis rotation since it's offset from robot center
        // (same lever-arm calculation as calculateVirtualTargetPosition above).
        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        double rx = Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS;
        double ry = Constants.ShooterConstants.BALL_LAUNCH_LATERAL_OFFSET_METERS;
        double heading = robotPose.getRotation().getRadians();
        double launchPointVelX = fieldRelativeSpeeds.vxMetersPerSecond
            + (-rx * Math.sin(heading) - ry * Math.cos(heading)) * omega;
        double launchPointVelY = fieldRelativeSpeeds.vyMetersPerSecond
            + ( rx * Math.cos(heading) - ry * Math.sin(heading)) * omega;

        // Subtract the launch point's velocity from the stationary solution — what remains is
        // what the shooter must impart, relative to the moving robot, to reproduce that exact
        // field-relative velocity.
        Translation3d shooterRelativeVelocity = new Translation3d(
            stationaryVelocity.getX() - launchPointVelX,
            stationaryVelocity.getY() - launchPointVelY,
            stationaryVelocity.getZ());

        double actualSpeedMPS = shooterRelativeVelocity.getNorm();
        double actualHorizontalSpeed = Math.hypot(shooterRelativeVelocity.getX(), shooterRelativeVelocity.getY());
        double actualLaunchAngleRad = Math.atan2(shooterRelativeVelocity.getZ(), actualHorizontalSpeed);
        double actualBearingRad = Math.atan2(shooterRelativeVelocity.getY(), shooterRelativeVelocity.getX());

        double turretAngleDeg = Math.toDegrees(actualBearingRad) - robotPose.getRotation().getDegrees();
        double rackAngleDeg = 90.0 - Math.toDegrees(actualLaunchAngleRad);
        double flywheelMotorRPS = actualSpeedMPS / (Math.PI * Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS);

        SmartDashboard.putNumber("Shooter/Physics/TurretAngleDeg", turretAngleDeg);
        SmartDashboard.putNumber("Shooter/Physics/RackAngleDeg", rackAngleDeg);
        SmartDashboard.putNumber("Shooter/Physics/FlywheelMotorRPS", flywheelMotorRPS);
        SmartDashboard.putNumber("Shooter/Physics/LaunchSpeedMPS", actualSpeedMPS);
        SmartDashboard.putNumber("Shooter/Physics/HorizontalDist", horizontalDist);

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
        double omega = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
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
        double launchSpeedMPS = Math.PI * Constants.ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS
            * getShooterFlywheelVelocityRps();
        double shooterAngleDeg = 90.0 - getShooterRackAngleDeg();

        SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new Translation2d(
                Constants.ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS,
                Constants.ShooterConstants.BALL_LAUNCH_LATERAL_OFFSET_METERS),
            fieldRelativeSpeeds,
            shooterFacing,
            Units.Meters.of(Constants.ShooterConstants.BALL_LAUNCH_HEIGHT_METERS),
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
        double kP = SmartDashboard.getNumber("Turret/kP", m_tunedTurretKP);
        double kI = SmartDashboard.getNumber("Turret/kI", m_tunedTurretKI);
        double kD = SmartDashboard.getNumber("Turret/kD", m_tunedTurretKD);
        double kV = SmartDashboard.getNumber("Turret/kV", m_tunedTurretKV);
        if (kP != m_tunedTurretKP || kI != m_tunedTurretKI || kD != m_tunedTurretKD || kV != m_tunedTurretKV) {
            m_tunedTurretKP = kP;
            m_tunedTurretKI = kI;
            m_tunedTurretKD = kD;
            m_tunedTurretKV = kV;
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
