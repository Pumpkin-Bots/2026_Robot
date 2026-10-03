package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.utils.AimSolver;
import frc.robot.utils.AimSolver.AimSolution;
import frc.robot.utils.AimSolver.ArcPolicy;
import frc.robot.utils.FlywheelDroopCompensator;
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
    // Read for pitch and roll only. Yaw comes from the pose estimator instead, which is the same
    // gyro yaw after vision has had its say about it.
    private final Pigeon2 m_pigeon;
    private final ShooterTuning m_tuning = new ShooterTuning();
    private boolean m_turretZeroed = false;
    private double m_lastCommandedTurretAngle = 0.0;
    // Set by angleToTurretPosition() when it swaps the target 360° to the other side of the
    // range, and cleared by consumeTurretWrap(). This is the "turret spins all the way around to
    // reset itself" event, and it is a discrete event rather than a threshold on tracking error
    // precisely so a turret that is merely lagging can never be mistaken for one that is unwrapping.
    private boolean m_turretWrapped = false;
    private double m_commandedFlywheelRps = 0.0;
    private AimSolution m_lastSolution = null;

    // ---- Flywheel ----
    // The speed actually handed to the velocity loop: the aim solution's number plus whatever the
    // droop compensator says it takes for the ball to leave at that number. Distinct from
    // m_commandedFlywheelRps, which stays the aim figure so callers asking "is the shooter parked"
    // and the telemetry comparing solution against reality both keep meaning what they meant.
    private double m_flywheelSetpointRps = 0.0;
    private final FlywheelDroopCompensator m_droopCompensator = new FlywheelDroopCompensator();

    /**
     * Rate the flywheel's velocity is published and sampled at, Hz. Set by the width of the thing
     * being measured: a ball's contact with the wheel lasts on the order of 10-25 ms, so anything
     * near the 50 Hz main loop rate cannot see the dip at all. See the constructor.
     */
    private static final double kFlywheelSampleHz = 200.0;

    /** Registered once so the fast sampler can refresh them without going through the motor object. */
    private final StatusSignal<AngularVelocity> m_flywheelVelocity;

    /**
     * Applied output as a fraction of supply voltage. The droop compensator freezes its learning
     * while this is pinned: a wheel already at full voltage cannot be made to spin faster by asking
     * for more, so a deficit measured there would wind the command up against a shot that simply is
     * not achievable at this gearing.
     */
    private final StatusSignal<Double> m_flywheelDutyCycle;

    /**
     * Whether the flywheel motor holds a Phoenix Pro license, published as
     * {@code Shooter/Flywheel/FocActive}.
     *
     * <p>FOC is requested on every control in this subsystem, but requesting it is not the same as
     * getting it: an unlicensed TalonFX falls back to trapezoidal commutation and raises
     * {@code Fault_UnlicensedFeatureInUse} rather than refusing the request. Without this readout
     * "we enabled FOC" and "FOC is running" are indistinguishable from the driver station, which is
     * how a licensing problem gets misdiagnosed as a tuning problem.
     */
    private final StatusSignal<Boolean> m_flywheelProLicensed;
    // Reused rather than reallocated per loop; the acceleration field is refilled each call.
    //
    // withEnableFOC(true) is stated rather than left to the default — see the note on
    // flywheelGains about what an "FOC conversion" must not touch. Phoenix 6 already defaults
    // EnableFOC to true on every *Voltage and *DutyCycle request, so this is documentation of
    // intent, not a behaviour change: the wheel spins exactly as fast as it did before.
    private final VelocityVoltage m_flywheelRequest =
        new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    // What a flywheel command of zero actually sends. Reused for the same no-allocation reason as
    // the requests above, and carries no FOC flag because there is no commutation to do when no
    // current is being asked for. See setShooterFlywheelVelocity.
    private final CoastOut m_flywheelCoastRequest = new CoastOut();

    // Turret and rack position requests, reused for the same reasons as the flywheel's: FOC stated
    // explicitly, and no per-loop allocation. Both velocity and feedForward are refilled on every
    // call — including with zeros on the paths that do not use them — because a reused request keeps
    // whatever was last written to it, and a stale feedforward would quietly push the mechanism.
    private final PositionVoltage m_turretRequest =
        new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private final PositionVoltage m_rackRequest =
        new PositionVoltage(0).withSlot(0).withEnableFOC(true);

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
    private final TunableDouble m_turretKS =
        new TunableDouble("Tuning/Turret/kS", Constants.ShooterConstants.ROTATOR_KS);
    // Unlike the gains above, this never touches a Slot0Configs — it is read straight into the
    // control request every loop, so there is no configurator round-trip and no re-apply guard.
    private final TunableDouble m_turretKff =
        new TunableDouble("Tuning/Turret/kFF", Constants.ShooterConstants.ROTATOR_KFF);

    // Last gains actually pushed to the motor, so the config is only re-applied when something
    // changes rather than every loop.
    private double m_appliedTurretKP = Constants.ShooterConstants.ROTATOR_KP;
    private double m_appliedTurretKI = Constants.ShooterConstants.ROTATOR_KI;
    private double m_appliedTurretKD = Constants.ShooterConstants.ROTATOR_KD;
    private double m_appliedTurretKV = Constants.ShooterConstants.ROTATOR_KV;
    private double m_appliedTurretKS = Constants.ShooterConstants.ROTATOR_KS;

    // ---- Live PID tuning, rack ----
    // The rack carries the launch angle, so a loop that undershoots its commanded angle is an aiming
    // error that looks exactly like a bad physics calibration — and chasing the calibration when the
    // mechanism is the problem wastes a whole test session. Watch "Shooter/Physics/RackErrorDeg"
    // while tuning these: it should settle to near zero well before a shot is taken.
    private final TunableDouble m_rackKP =
        new TunableDouble("Tuning/Rack/kP", Constants.ShooterConstants.RACK_KP);
    private final TunableDouble m_rackKI =
        new TunableDouble("Tuning/Rack/kI", Constants.ShooterConstants.RACK_KI);
    private final TunableDouble m_rackKD =
        new TunableDouble("Tuning/Rack/kD", Constants.ShooterConstants.RACK_KD);

    private double m_appliedRackKP = Constants.ShooterConstants.RACK_KP;
    private double m_appliedRackKI = Constants.ShooterConstants.RACK_KI;
    private double m_appliedRackKD = Constants.ShooterConstants.RACK_KD;

    // ---- Live PID tuning, flywheel ----
    // Same mechanism as the turret's above. These matter more than they look: how quickly the
    // velocity loop puts back the speed a ball took out of the wheel is half of what decides
    // whether the NEXT shot leaves at the right speed, and it is not something that can be judged
    // without watching "Shooter/Flywheel/LastRecoverySec" change as the gains do.
    //
    // kA is absent on purpose — it comes from the wheel's inertia (see flywheelKaFromMoi), so the
    // thing to edit is FLYWHEEL_MOI_KG_M2, not a gain.
    private final TunableDouble m_flywheelKP =
        new TunableDouble("Tuning/Flywheel/kP", Constants.ShooterConstants.FLYWHEEL_KP);
    private final TunableDouble m_flywheelKI =
        new TunableDouble("Tuning/Flywheel/kI", Constants.ShooterConstants.FLYWHEEL_KI);
    private final TunableDouble m_flywheelKD =
        new TunableDouble("Tuning/Flywheel/kD", Constants.ShooterConstants.FLYWHEEL_KD);
    private final TunableDouble m_flywheelKV =
        new TunableDouble("Tuning/Flywheel/kV", Constants.ShooterConstants.FLYWHEEL_KV);
    private final TunableDouble m_flywheelKS =
        new TunableDouble("Tuning/Flywheel/kS", Constants.ShooterConstants.FLYWHEEL_KS);

    private double m_appliedFlywheelKP = Constants.ShooterConstants.FLYWHEEL_KP;
    private double m_appliedFlywheelKI = Constants.ShooterConstants.FLYWHEEL_KI;
    private double m_appliedFlywheelKD = Constants.ShooterConstants.FLYWHEEL_KD;
    private double m_appliedFlywheelKV = Constants.ShooterConstants.FLYWHEEL_KV;
    private double m_appliedFlywheelKS = Constants.ShooterConstants.FLYWHEEL_KS;

    private static final Slot0Configs turretRotatorGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.ROTATOR_KP)
        .withKI(Constants.ShooterConstants.ROTATOR_KI)
        .withKD(Constants.ShooterConstants.ROTATOR_KD)
        .withKV(Constants.ShooterConstants.ROTATOR_KV)
        .withKS(Constants.ShooterConstants.ROTATOR_KS);

    private static final Slot0Configs rackGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.RACK_KP)
        .withKI(Constants.ShooterConstants.RACK_KI)
        .withKD(Constants.ShooterConstants.RACK_KD);

    /**
     * Acceleration feedforward for the flywheel, in volts per (rotor rotation/s²), derived from the
     * wheel's moment of inertia instead of guessed at.
     *
     * <p>Spinning the rotor up at {@code α} rad/s² takes {@code τ = J·α} of torque, which takes
     * {@code τ/Kt} of current, which takes {@code τ·R/Kt} of voltage across the winding. Converting
     * to the rotor rotations/s² Phoenix works in puts a {@code 2π} on the front:
     *
     * <pre>kA = 2π · J_rotor · R / Kt</pre>
     *
     * <p>{@code J_rotor} is the inertia as the rotor feels it — the wheel's own inertia divided by
     * the square of the reduction, which is why the 28:18 gearing makes this nearly two and a half
     * times smaller than the direct-drive figure would have been.
     *
     * <p>This is what makes {@code FLYWHEEL_MOI_KG_M2} worth measuring rather than leaving at its
     * placeholder: it is the same number that decides how far the wheel sags per ball, so getting
     * it right improves both the recovery and the compensation for what was lost.
     *
     * <p>kA only earns its keep because {@link #setShooterFlywheelVelocity} hands the request the
     * setpoint's own rate of change — a Phoenix velocity request multiplies kA by the acceleration
     * it is given, and the default is zero.
     *
     * @param rotorMoiKgM2 inertia reflected to the rotor, i.e. {@code FLYWHEEL_ROTOR_MOI_KG_M2}
     */
    private static double flywheelKaFromMoi(double rotorMoiKgM2) {
        return 2.0 * Math.PI * rotorMoiKgM2 * kFlywheelGearbox.rOhms / kFlywheelGearbox.KtNMPerAmp;
    }

    /** Robot loop period, used to turn a setpoint step into the rate of change kA acts on. */
    private static final double kLoopPeriodSeconds = 0.020;

    /**
     * The most acceleration the flywheel motor can actually produce, in rotor rotations/s², from
     * stall torque against the wheel's inertia. The setpoint's rate of change is clamped to it, so
     * a step command asks kA for the volts to accelerate as hard as the motor can and not for the
     * fictional volts to do it in one 20 ms loop.
     */
    private static final double kFlywheelMaxAccelRps2 =
        kFlywheelGearbox.stallTorqueNewtonMeters
        / (2.0 * Math.PI * Constants.ShooterConstants.FLYWHEEL_ROTOR_MOI_KG_M2);

    /**
     * Flywheel velocity gains, <b>in volts</b>.
     *
     * <p>This is the single thing an "enable FOC everywhere" change must not disturb, and it is
     * worth being explicit about why. FOC is a commutation mode: it is a flag on the control
     * request, orthogonal to the units the request is expressed in, and Phoenix 6 turns it on by
     * default for every {@code *Voltage} and {@code *DutyCycle} request. Nothing about enabling it
     * changes what these numbers mean, and nothing about it can make a shot leave slower — FOC
     * raises peak torque, it does not lower it.
     *
     * <p>What <i>does</i> make the shot leave far too slow is reaching for FOC by swapping the
     * request to the {@code *TorqueCurrentFOC} family, because that family's output — and therefore
     * every gain in this slot — is reinterpreted from volts to <b>amps</b>. kV 0.125 stops meaning
     * "0.125 V per rotation/s" and starts meaning "0.125 A per rotation/s", so the feedforward
     * holding a 40 RPS shot drops from 5 volts to 5 amps, kP 0.4 goes from 0.4 V to 0.4 A per RPS
     * of error, and the wheel creeps along at a fraction of its commanded speed. The symptom is a
     * shot that looks weak everywhere, with {@code Shooter/Flywheel/VelocityRps} sitting well under
     * {@code SetpointRps} and the duty cycle nowhere near saturated.
     *
     * <p>So: FOC is requested explicitly on every request in this subsystem, and every request
     * stays in the voltage family. If someone does want torque-current control later, these gains
     * have to be re-characterised in amps from scratch — they do not carry over.
     */
    private static final Slot0Configs flywheelGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.FLYWHEEL_KP)
        .withKI(Constants.ShooterConstants.FLYWHEEL_KI)
        .withKD(Constants.ShooterConstants.FLYWHEEL_KD)
        .withKV(Constants.ShooterConstants.FLYWHEEL_KV)
        .withKS(Constants.ShooterConstants.FLYWHEEL_KS)
        .withKA(flywheelKaFromMoi(Constants.ShooterConstants.FLYWHEEL_ROTOR_MOI_KG_M2));

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

    /**
     * Flywheel current limits. The stator limit is the torque the wheel is allowed to make, and it
     * is what decides how fast the velocity loop claws back the speed a ball took out — so this one
     * is deliberately the loosest in the robot at 120 A.
     *
     * <p>Note the supply limit still sits at 40 A, and supply current is roughly stator current
     * times duty cycle. Recovery after a ball happens at high duty cycle, so above about 33% output
     * it is the <i>supply</i> limit that actually binds, not the 120 A. Raising supply is a battery
     * decision rather than a motor one, so it is left alone here; if recovery still looks slow with
     * 120 A stator, watch {@code Shooter/Flywheel/DutyCycleAvg} and raise supply next, not stator.
     */
    private static final CurrentLimitsConfigs flywheelCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(120);

    private static final CurrentLimitsConfigs rackCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(25)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(35);

    private static final CurrentLimitsConfigs turretCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

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
        rackAngleTable.put(8.0,  39.0);
        rackAngleTable.put(9.0,  39.0);
        rackAngleTable.put(10.0, 39.0);
        rackAngleTable.put(11.0, 39.0);
        rackAngleTable.put(12.0, 39.0);
        rackAngleTable.put(13.0, 39.0);
        rackAngleTable.put(14.0, 39.0);
        rackAngleTable.put(15.0, 39.0);

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
        m_pigeon = drivetrain.getPigeon2();

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
        // 200 Hz, not the 50 Hz everything else here uses. A ball is in contact with the wheel for
        // something like 10-25 ms, so the dip it produces is at most one sample wide at 50 Hz: the
        // detector would miss most troughs outright and catch a random point on the curve for the
        // rest, which is worse than missing them — it would learn a confidently wrong number. At
        // 200 Hz a contact spans 2-5 samples, and the Hoot log captures the dip's actual shape,
        // which is how the contact time itself gets measured.
        m_flywheelVelocity = m_shooterFlywheelMotor.getVelocity();
        m_flywheelDutyCycle = m_shooterFlywheelMotor.getDutyCycle();
        BaseStatusSignal.setUpdateFrequencyForAll(
            kFlywheelSampleHz, m_flywheelVelocity, m_flywheelDutyCycle);
        // Registered before optimizeBusUtilization() silences it, at a rate suited to a fact that
        // only changes when someone licenses the device.
        m_flywheelProLicensed = m_shooterFlywheelMotor.getIsProLicensed();
        m_flywheelProLicensed.setUpdateFrequency(4);
        m_shooterFlywheelMotor.optimizeBusUtilization();

        // Tilt compensation reads the Pigeon through getRotation3d(), which is fed by the four
        // quaternion signals — not the yaw signal the drivetrain already asks for. Request them
        // explicitly at the loop rate: it keeps the tilt reading fresh, and it means a future
        // optimizeBusUtilization() on the drivetrain, which silences everything unrequested, cannot
        // quietly turn tilt compensation into a no-op that still looks like it is running.
        BaseStatusSignal.setUpdateFrequencyForAll(50,
            m_pigeon.getQuatW(), m_pigeon.getQuatX(), m_pigeon.getQuatY(), m_pigeon.getQuatZ());

        if (RobotBase.isSimulation()) {
            m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    kFlywheelGearbox,
                    Constants.ShooterConstants.FLYWHEEL_MOI_KG_M2,
                    Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO),
                kFlywheelGearbox
            );
        } else {
            m_flywheelSim = null;
        }
    }

    /**
     * The robot's full orientation relative to the field: heading from the pose estimator, pitch and
     * roll from the gyro.
     *
     * <p>The pitch and roll are the whole point — they are what let {@link AimSolver} work out where
     * the turret and rack have to point on a robot that is not sitting flat. Three things happen to
     * them on the way out, all of them guarding against a correction that is worse than no
     * correction:
     *
     * <ul>
     *   <li>The mounting offsets come off first, so a Pigeon bolted down slightly out of plane does
     *       not spend the match aiming the shot at an imaginary tilt.
     *   <li>The result is clamped, so a gyro fault moves the turret by a bounded amount.
     *   <li>The tuning gain scales it, so the whole correction can be taken back out from the
     *       dashboard without a redeploy if it turns out to be aiming the wrong way.
     * </ul>
     *
     * <p>Read through {@code getRotation3d()} rather than {@code getPitch()}/{@code getRoll()}
     * because that is the reading already expressed in WPILib's axis convention, which is the one
     * the solver's rotation needs. In simulation it reports level, which is also true — maple-sim's
     * field is flat — so the whole correction quietly becomes the identity there.
     */
    private Rotation3d robotOrientation(Pose2d robotPose) {
        Rotation3d gyro = m_pigeon.getRotation3d();
        double gain = m_tuning.tiltCompensationGain();
        double rollDeg = compensatedTiltDeg(
            Math.toDegrees(gyro.getX()), m_tuning.tiltRollOffsetDeg(), gain);
        double pitchDeg = compensatedTiltDeg(
            Math.toDegrees(gyro.getY()), m_tuning.tiltPitchOffsetDeg(), gain);
        return new Rotation3d(
            Math.toRadians(rollDeg),
            Math.toRadians(pitchDeg),
            robotPose.getRotation().getRadians());
    }

    /** One tilt axis: calibration offset removed, authority applied, and bounded. */
    private static double compensatedTiltDeg(double measuredDeg, double offsetDeg, double gain) {
        if (!Double.isFinite(measuredDeg)) {
            return 0.0;
        }
        return MathUtil.clamp(
            (measuredDeg - offsetDeg) * gain,
            -Constants.ShooterConstants.TILT_MAX_COMPENSATED_DEG,
            Constants.ShooterConstants.TILT_MAX_COMPENSATED_DEG);
    }

    /**
     * Computes the ball's launch position in field coordinates, accounting for the robot's
     * orientation and the forward/lateral/height offset from center.
     *
     * <p>Tilt moves the launch point as well as turning the barrel: the shooter sits about half a
     * meter up, so that offset swings through an arc as the chassis tips, carrying the muzzle
     * several centimeters sideways and slightly down. Rotating the whole offset — height included —
     * by the robot's orientation is the same arithmetic as before once the robot is level.
     */
    private Translation3d calculateLaunchPosition(Pose2d robotPose, Rotation3d orientation) {
        Translation3d offset = new Translation3d(
            m_tuning.launchForwardOffsetMeters(),
            m_tuning.launchLeftOffsetMeters(),
            m_tuning.launchHeightMeters()).rotateBy(orientation);
        return new Translation3d(
            robotPose.getX() + offset.getX(),
            robotPose.getY() + offset.getY(),
            offset.getZ());
    }

    /**
     * Field-relative 3D position the ball leaves from, using the drivetrain's current pose estimate.
     *
     * <p>Zone decisions (shoot / shuttle / storage) key off this rather than the robot's centre: the
     * launch point hangs off centre, so on a robot straddling a boundary the two genuinely disagree,
     * and what matters is where the ball comes out.
     */
    public Translation3d getLaunchPosition() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        return calculateLaunchPosition(robotPose, robotOrientation(robotPose));
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
        // Not tilt-compensated: the tables are indexed on distance alone and have no way to express
        // a rack angle measured from a tipped-over chassis. The physics path below is the one that
        // handles a robot on the depot; this one aims as if the floor were flat.
        Translation3d launchPosition = calculateLaunchPosition(robotPose, new Rotation3d(
            0.0, 0.0, robotPose.getRotation().getRadians()));
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
        return solveAim(targetPosition, ArcPolicy.DESCENT_MARGIN);
    }

    /**
     * Solves for a firing solution against a field target without commanding anything.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     * @param policy         how the launch angle is chosen — see {@link ArcPolicy}
     */
    public AimSolution solveAim(Translation3d targetPosition, ArcPolicy policy) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Rotation3d orientation = robotOrientation(robotPose);
        Translation3d launchPosition = calculateLaunchPosition(robotPose, orientation);
        Translation2d launchPointVel = calculateLaunchPointVelocity(robotPose);

        return AimSolver.solve(
            targetPosition,
            launchPosition,
            orientation,
            launchPointVel.getX(),
            launchPointVel.getY(),
            m_tuning.snapshot(),
            policy);
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
        commandSolution(solveAim(targetPosition, ArcPolicy.DESCENT_MARGIN));
    }

    /**
     * Same solve and the same commands, but with the arc pinned flat ({@link ArcPolicy#FLATTEST})
     * instead of biased steep — the shuttle case.
     *
     * <p>A shuttle pass is thrown to the carpet, not into a goal, so the descent margin that keeps
     * a hub shot from skimming the rim is buying nothing: it only trades flywheel speed and hang
     * time for an arc shape that does not matter once the target is the floor. Holding the rack at
     * its high stop asks the least of the flywheel at long range and puts the ball down sooner.
     *
     * @param targetPosition field-relative 3D position of the target (x, y, z in meters)
     */
    public void calculatePhysicsShuttleActions(Translation3d targetPosition) {
        commandSolution(solveAim(targetPosition, ArcPolicy.FLATTEST));
    }

    /** Records, publishes, and commands a solution. */
    private void commandSolution(AimSolution solution) {
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
        // Against the solution's own number, not the compensated setpoint — this row answers "did
        // the ball get the speed the solve asked for", which is the question a missed shot raises.
        SmartDashboard.putNumber("Shooter/Physics/FlywheelErrorRPS",
            s.flywheelRps() - getShooterFlywheelVelocityRps());
        SmartDashboard.putBoolean("Shooter/Physics/FlywheelReady", isFlywheelReady());

        // How the motion correction got divided up. With the rack carrying it, the shift row moves
        // with the driver's sticks and the flywheel command stays on the nominal — if instead the
        // two RPS rows are pulling apart, the arc has run out of travel and the flywheel is back to
        // chasing the robot's velocity.
        SmartDashboard.putNumber("Shooter/Physics/MotionArcShiftDeg", s.motionArcShiftDeg());
        SmartDashboard.putNumber("Shooter/Physics/NominalFlywheelRPS", s.nominalFlywheelRps());
        SmartDashboard.putBoolean("Shooter/Physics/MotionRackSaturated", s.motionRackSaturated());

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

        // If the unwrapped angle exceeds a limit, swap to the other side. That swap is the turret
        // committing to a full sweep the long way round, so flag it — nothing points at the target
        // until the sweep finishes. See consumeTurretWrap().
        if (target > max) {
            target -= 360.0;
            m_turretWrapped = true;
        } else if (target < min) {
            target += 360.0;
            m_turretWrapped = true;
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
     * <p>Bearing only, and deliberately not tilt-compensated: nothing is being fired here. This is
     * the turret keeping itself roughly pre-aimed while in storage so that leaving a trench does not
     * cost a turret swing before the first shot, and the shot itself is solved properly by
     * {@link #solveAim} the moment one is actually taken.
     *
     * @param targetPosition field-relative 3D position of the target
     */
    public void aimTurretAt(Translation3d targetPosition) {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Translation3d launchPosition = getLaunchPosition();
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

    /**
     * How far the turret still has to travel to reach its commanded angle, in degrees (commanded
     * minus measured). Unwrapped, so a turret partway through a 360° reset reads a few hundred
     * degrees rather than folding back into ±180°.
     */
    public double getTurretErrorDeg() {
        return m_lastCommandedTurretAngle - getTurretRotatorAngleDeg();
    }

    /**
     * True once if the turret has wrapped 360° to the other side of its range since this was last
     * called, then false again until the next wrap — see {@link #angleToTurretPosition}.
     *
     * <p>Reading it clears it, so exactly one caller can act on each wrap. That caller is
     * {@code ShooterMode}, which holds the shot until the sweep has finished.
     */
    public boolean consumeTurretWrap() {
        boolean wrapped = m_turretWrapped;
        m_turretWrapped = false;
        return wrapped;
    }

    /** Returns the current shooter rack angle in degrees (same convention as {@link #setShooterRackAngle}). */
    public double getShooterRackAngleDeg() {
        return m_shooterRackMotor.getPosition().getValueAsDouble()
            * Constants.ShooterConstants.RACK_GEAR_RATIO * 360.0
            + Constants.ShooterConstants.RACK_MIN_ANGLE;
    }

    public void setTurretRotatorPosition(double position) {
        m_lastCommandedTurretRotorPosition = position;
        m_turretRotatorMotor.setControl(m_turretRequest.withPosition(position)
            .withVelocity(0.0)
            .withFeedForward(0.0));
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
        double turretRateRps = -omega / (2.0 * Math.PI);
        double motorVelRps = turretRateRps / Constants.ShooterConstants.TURRET_ROTATOR_GEAR_RATIO;
        m_turretRotatorMotor.setControl(m_turretRequest.withPosition(position)
            .withVelocity(motorVelRps)
            .withFeedForward(turretFeedForwardVolts(turretRateRps, motorVelRps)));
    }

    /**
     * One-directional static feedforward, in volts, for a commanded turret rate.
     *
     * <p>The wiring chain only spools against the turret one way round, so only that direction is
     * assisted — the other returns zero rather than being pushed by an equal and opposite amount.
     * That asymmetry is why this is an arbitrary feedforward on the request rather than Slot0
     * {@code kS}, which always applies its magnitude in both directions.
     *
     * @param turretRateRps commanded turret rate, turret rotations per second (positive = CCW)
     * @param motorVelRps   the same rate in motor rotations per second, whose sign the output takes
     */
    private double turretFeedForwardVolts(double turretRateRps, double motorVelRps) {
        double kFF = m_turretKff.get();

        // Near zero the direction is meaningless and would dither loop to loop, so hold the term
        // off rather than leaning on a turret that is trying to hold still.
        if (Math.abs(turretRateRps) * 360.0
                < Constants.ShooterConstants.ROTATOR_KFF_DEADBAND_DEG_PER_SEC) {
            return 0.0;
        }

        // Sign of kFF selects which direction gets the assist; magnitude is the voltage.
        if (kFF == 0.0 || (kFF > 0.0) != (turretRateRps > 0.0)) {
            return 0.0;
        }
        return Math.abs(kFF) * Math.signum(motorVelRps);
    }

    public void setShooterRackPosition(double position) {
        m_lastCommandedRackRotorPosition = position;
        m_shooterRackMotor.setControl(m_rackRequest.withPosition(position)
            .withVelocity(0.0)
            .withFeedForward(0.0));
    }

    /** Sets rack position from a target angle in degrees. */
    public void setShooterRackAngle(double angleDeg) {
        setShooterRackPosition(angleToRackPosition(angleDeg));
    }

    /**
     * Commands the flywheel, biased by whatever the droop compensator says it costs to actually
     * deliver this speed to a ball.
     *
     * <p>The aim solution's number is the speed the ball has to <i>leave</i> at, and a wheel held
     * exactly there does not produce it: the ball takes energy out of the wheel on its way through,
     * so it separates from a wheel that has already slowed. The bias added here is measured from
     * the robot's own shots — see {@link FlywheelDroopCompensator} — and is zero until there is
     * something to measure on a real field, so a mis-tuned compensator can never make the shot
     * worse than an uncompensated one by more than {@code FLYWHEEL_MAX_COMPENSATION_RPS}.
     *
     * <p>The request also carries the setpoint's own rate of change as its acceleration, which is
     * what gives the inertia-derived kA something to act on: a solution walking the speed up as the
     * robot backs away from the hub gets the volts for it immediately, rather than after kP has
     * noticed the wheel falling behind.
     *
     * @param velocity the speed the aim solution asked for, in motor RPS
     */
    public void setShooterFlywheelVelocity(double velocity) {
        double clamped = Math.max(-Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC,
            Math.min(Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC, velocity));
        m_commandedFlywheelRps = clamped;

        // Asking for zero means "we are not shooting" — trench, defense, storage — and the honest
        // way to say that to a flywheel is to stop driving it, not to hold it at zero. Holding zero
        // is a real command: the velocity loop sees a wheel spinning well above its setpoint and
        // drags it down with reverse voltage, which dumps the wheel's whole stored energy into the
        // motor and the battery over a second or two, then keeps kS fighting it to stay stopped.
        // Coasting instead costs nothing, and the wheel is still turning when the next shot comes
        // up, so the spin-up starts partway there. There is no accuracy to lose: a parked wheel was
        // never going to deliver a ball anyway.
        //
        // CoastOut rather than NeutralOut because this is a statement about the mechanism, not a
        // deference to configuration — the flywheel must freewheel even if the motor's neutral mode
        // is later set to brake for some other reason.
        //
        // A negative request lands here too. Nothing in the project asks the flywheel to run
        // backwards, and a mechanism that genuinely needed to would want its own path rather than
        // this one's droop compensation and acceleration feed-forward.
        if (clamped <= 0.0) {
            m_flywheelSetpointRps = 0.0;
            m_shooterFlywheelMotor.setControl(m_flywheelCoastRequest);
            return;
        }

        double setpoint = Math.min(Constants.ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC,
            clamped + m_droopCompensator.compensationRps(clamped));

        double accelRps2 = MathUtil.clamp(
            (setpoint - m_flywheelSetpointRps) / kLoopPeriodSeconds,
            -kFlywheelMaxAccelRps2, kFlywheelMaxAccelRps2);
        m_flywheelSetpointRps = setpoint;
        m_shooterFlywheelMotor.setControl(
            m_flywheelRequest.withVelocity(setpoint).withAcceleration(accelRps2));
    }

    /**
     * Flywheel speed most recently commanded, in motor RPS. Distinct from
     * {@link #getShooterFlywheelVelocityRps()}, which is what the wheel is actually doing — this is
     * what it was asked for, so a caller can tell "the shooter is parked" from "the shooter is
     * spinning down".
     *
     * <p>This is the <i>aim solution's</i> number, before droop compensation. See
     * {@link #getFlywheelSetpointRps()} for the one the velocity loop is chasing.
     */
    public double getCommandedFlywheelRps() {
        return m_commandedFlywheelRps;
    }

    /** The speed the velocity loop is actually holding to, in motor RPS — aim target plus droop bias. */
    public double getFlywheelSetpointRps() {
        return m_flywheelSetpointRps;
    }

    /**
     * Current flywheel speed in MOTOR rotations per second, straight off the rotor.
     *
     * <p>Motor units, not wheel units — no {@code SensorToMechanismRatio} is configured on this
     * motor, and every commanded speed in the project (the lookup tables, the solver's output,
     * {@code FLYWHEEL_MAX_REV_PER_SEC}, kV) is in the same frame, so this compares directly against
     * all of them. The gearing is accounted for once, in
     * {@code FLYWHEEL_EFFECTIVE_DIAMETER_METERS}, where motor speed becomes ball speed.
     *
     * <p>See {@link #getFlywheelWheelRps()} for what the wheel itself is doing.
     */
    public double getShooterFlywheelVelocityRps() {
        return m_shooterFlywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Speed of the LARGE (4 in) flywheel itself, in wheel rotations per second. Telemetry only —
     * nothing commands in this frame — but it is the number to compare against a tachometer or a
     * slow-motion video when checking that the gearing constants match the real gearbox.
     */
    public double getFlywheelWheelRps() {
        return getShooterFlywheelVelocityRps() / Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    /**
     * True when the flywheel is within {@code FLYWHEEL_AT_SPEED_TOLERANCE_RPS} of the speed it is
     * being held to, and so is ready for a ball. False whenever the wheel is parked.
     *
     * <p>Note what this is measured against: the compensated setpoint, not the aim target. A wheel
     * sitting at the aim target is not ready — that is precisely the state that produces a shot
     * which leaves slow.
     */
    public boolean isFlywheelReady() {
        return m_commandedFlywheelRps > 0.0 && m_droopCompensator.isAtSpeed();
    }

    /** Droop compensator, for telemetry and for commands that want to hold fire until it is ready. */
    public FlywheelDroopCompensator getDroopCompensator() {
        return m_droopCompensator;
    }

    /**
     * Writes the learned flywheel droop curve to the roboRIO if anything new has been measured.
     * Called from {@code Robot.disabledInit()} — the robot gets power-cycled between matches, and
     * being disabled is the one moment a flash write is guaranteed to cost nothing.
     */
    public void saveFlywheelLearning() {
        m_droopCompensator.saveIfDirty();
    }

    /**
     * Feeds one fresh velocity sample to the droop detector. Scheduled at
     * {@link #kFlywheelSampleHz} from {@code Robot}, NOT from {@code periodic()} — a ball's whole
     * contact with the wheel fits inside a single 50 Hz loop, so a detector running at loop rate is
     * measuring an event it cannot see.
     *
     * <p>Runs on the main robot thread (WPILib's {@code addPeriodic} callbacks are interleaved with
     * the main loop, not threaded), so there is no synchronisation to think about against the
     * commands that write {@code m_flywheelSetpointRps}.
     *
     * <p>The setpoint it compares against is whatever the last command loop commanded, which is the
     * right pairing: the wheel is chasing that number for the whole 20 ms until the next one.
     */
    public void sampleFlywheelDroop() {
        BaseStatusSignal.refreshAll(m_flywheelVelocity, m_flywheelDutyCycle);
        m_droopCompensator.update(
            m_flywheelSetpointRps,
            m_flywheelVelocity.getValueAsDouble(),
            m_flywheelDutyCycle.getValueAsDouble());
    }

    /** The rate {@link #sampleFlywheelDroop()} expects to be called at, in seconds per call. */
    public static double flywheelSamplePeriodSeconds() {
        return 1.0 / kFlywheelSampleHz;
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

    /**
     * Publishes what the gyro says about how level the robot is.
     *
     * <p>Raw, before the offsets and the gain, because these rows are what the offsets get read off
     * in the first place — park on flat carpet and whatever they say is the mounting error. They are
     * published every loop rather than only while shooting, including while disabled, so that
     * calibration doesn't require holding a button.
     *
     * <p>{@code AppliedRollDeg}/{@code AppliedPitchDeg} are the same two numbers after all of that,
     * i.e. the tilt the aim is actually being corrected for. Equal pairs mean the calibration is
     * doing nothing; a large disagreement means it is doing a lot, and is worth understanding before
     * trusting a shot.
     */
    private void publishTiltTelemetry() {
        Rotation3d gyro = m_pigeon.getRotation3d();
        double rollDeg = Math.toDegrees(gyro.getX());
        double pitchDeg = Math.toDegrees(gyro.getY());
        double gain = m_tuning.tiltCompensationGain();
        double appliedRoll = compensatedTiltDeg(rollDeg, m_tuning.tiltRollOffsetDeg(), gain);
        double appliedPitch = compensatedTiltDeg(pitchDeg, m_tuning.tiltPitchOffsetDeg(), gain);

        SmartDashboard.putNumber("Shooter/Tilt/RollDeg", rollDeg);
        SmartDashboard.putNumber("Shooter/Tilt/PitchDeg", pitchDeg);
        SmartDashboard.putNumber("Shooter/Tilt/AppliedRollDeg", appliedRoll);
        SmartDashboard.putNumber("Shooter/Tilt/AppliedPitchDeg", appliedPitch);
        // Total angle between the chassis's up axis and the field's, which is the single number that
        // says how far from flat the robot is regardless of which way it is facing.
        SmartDashboard.putNumber("Shooter/Tilt/TotalDeg", Math.toDegrees(Math.acos(MathUtil.clamp(
            Math.cos(Math.toRadians(appliedRoll)) * Math.cos(Math.toRadians(appliedPitch)),
            -1.0, 1.0))));
    }

    @Override
    public void periodic() {
        publishTiltTelemetry();

        SmartDashboard.putBoolean(
            "Shooter/Flywheel/FocActive", m_flywheelProLicensed.refresh().getValue());

        applyTurretGains();
        applyRackGains();
        applyFlywheelGains();
    }

    /** The same re-apply-on-change as the turret's, for the rack's position loop. */
    private void applyRackGains() {
        double kP = m_rackKP.get();
        double kI = m_rackKI.get();
        double kD = m_rackKD.get();
        if (kP != m_appliedRackKP || kI != m_appliedRackKI || kD != m_appliedRackKD) {
            m_appliedRackKP = kP;
            m_appliedRackKI = kI;
            m_appliedRackKD = kD;
            m_shooterRackMotor.getConfigurator().apply(
                new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
        }
    }

    /**
     * Re-applies the turret's gains on any change, which includes tuning mode being switched off —
     * the gains snap back to the compiled constants the same way every other tunable does.
     */
    private void applyTurretGains() {
        double kP = m_turretKP.get();
        double kI = m_turretKI.get();
        double kD = m_turretKD.get();
        double kV = m_turretKV.get();
        double kS = m_turretKS.get();
        if (kP != m_appliedTurretKP || kI != m_appliedTurretKI
                || kD != m_appliedTurretKD || kV != m_appliedTurretKV
                || kS != m_appliedTurretKS) {
            m_appliedTurretKP = kP;
            m_appliedTurretKI = kI;
            m_appliedTurretKD = kD;
            m_appliedTurretKV = kV;
            m_appliedTurretKS = kS;
            m_turretRotatorMotor.getConfigurator().apply(
                new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV).withKS(kS));
        }
    }

    /** The same re-apply-on-change for the flywheel. kA is not tunable — it comes from the inertia. */
    private void applyFlywheelGains() {
        double kP = m_flywheelKP.get();
        double kI = m_flywheelKI.get();
        double kD = m_flywheelKD.get();
        double kV = m_flywheelKV.get();
        double kS = m_flywheelKS.get();
        if (kP != m_appliedFlywheelKP || kI != m_appliedFlywheelKI
                || kD != m_appliedFlywheelKD || kV != m_appliedFlywheelKV
                || kS != m_appliedFlywheelKS) {
            m_appliedFlywheelKP = kP;
            m_appliedFlywheelKI = kI;
            m_appliedFlywheelKD = kD;
            m_appliedFlywheelKV = kV;
            m_appliedFlywheelKS = kS;
            m_shooterFlywheelMotor.getConfigurator().apply(
                new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV).withKS(kS)
                    .withKA(flywheelKaFromMoi(
                        Constants.ShooterConstants.FLYWHEEL_ROTOR_MOI_KG_M2)));
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
