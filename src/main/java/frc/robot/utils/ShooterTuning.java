package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.utils.AimSolver.AimTuning;

/**
 * Every aiming calibration value, exposed live on SmartDashboard under {@code Tuning/Shooter/}.
 *
 * <p>This exists because none of these numbers will be right the first time. Each one can be typed
 * into a dashboard between shots instead of costing an edit-compile-deploy cycle, and the compiled
 * defaults in {@link ShooterConstants} document what shipped. Once a value is proven, copy it back
 * into Constants so it survives a reboot.
 *
 * <p>Suggested order to tune, one at a time, robot STATIONARY until step 6:
 * <ol>
 *   <li>{@code LaunchHeightM}, {@code LaunchForwardOffsetM}, {@code LaunchLeftOffsetM} — measure
 *       these, do not guess. Everything downstream is built on them.
 *   <li>{@code TurretOffsetDeg} — park at a fixed distance and correct any consistent left/right
 *       bias until shots are centered.
 *   <li>{@code SpeedScalar} — if every shot is short by a similar fraction, raise it.
 *   <li>{@code SpeedPerMeterMps} — if close shots are good but long shots fall short, raise this.
 *       That signature is air drag, which scales with range.
 *   <li>{@code RackOffsetDeg} / {@code DescentMarginDeg} — adjust arc shape. Raise the descent
 *       margin if shots skim the rim on the way up.
 *   <li>{@code FeederForwardPushMps} / {@code FeederBackwardPushAt90Mps} — with the rest dialed in,
 *       park the turret at 0 deg to isolate the forward term, then at 90 deg for the rearward one.
 *   <li>Only now start driving, and tune {@code ShootOnTheMoveGain}. Set it to 0 first to confirm
 *       stationary aim is still good, then bring it to 1.
 *   <li>{@code TiltRollOffsetDeg} / {@code TiltPitchOffsetDeg} — with the robot parked on flat
 *       carpet, read {@code Shooter/Tilt/RollDeg} and {@code Shooter/Tilt/PitchDeg} and enter what
 *       they say. They should be zero and generally won't be, and whatever they read on the flat is
 *       error the tilt compensation would otherwise apply all match. Then drive one wheel up onto
 *       the depot and confirm the two numbers move the way the robot physically did before
 *       trusting a shot from there — {@code TiltCompensationGain} is the switch that takes the
 *       whole correction back out if they don't.
 * </ol>
 */
public class ShooterTuning {

    private final TunableDouble m_gravity;
    private final TunableDouble m_descentMarginDeg;
    private final TunableDouble m_rackOffsetDeg;
    private final TunableDouble m_turretOffsetDeg;
    private final TunableDouble m_flywheelDiameterM;
    private final TunableDouble m_speedScalar;
    private final TunableDouble m_speedPerMeterMps;
    private final TunableDouble m_flywheelRpsOffset;
    private final TunableDouble m_shootOnTheMoveGain;
    private final TunableDouble m_feederForwardPushMps;
    private final TunableDouble m_feederBackwardPushAt90Mps;

    private final TunableDouble m_launchForwardOffsetM;
    private final TunableDouble m_launchLeftOffsetM;
    private final TunableDouble m_launchHeightM;

    private final TunableDouble m_tiltCompensationGain;
    private final TunableDouble m_tiltRollOffsetDeg;
    private final TunableDouble m_tiltPitchOffsetDeg;

    public ShooterTuning() {
        // maple-sim's projectiles fall under a flat 11 m/s^2 rather than 9.81, as its own stand-in
        // for air drag. Matching it in simulation is what makes the predicted trajectory agree with
        // the ball the sim actually draws; on the real robot, real gravity is the honest starting
        // point and drag gets handled by the speed calibration terms instead.
        m_gravity = new TunableDouble("Tuning/Shooter/GravityMps2",
            RobotBase.isSimulation()
                ? ShooterConstants.PHYSICS_GRAVITY_SIM_MPS2
                : ShooterConstants.PHYSICS_GRAVITY_REAL_MPS2);

        m_descentMarginDeg = new TunableDouble(
            "Tuning/Shooter/DescentMarginDeg", ShooterConstants.DESCENT_MARGIN_DEG);
        m_rackOffsetDeg = new TunableDouble(
            "Tuning/Shooter/RackOffsetDeg", ShooterConstants.RACK_ANGLE_OFFSET_DEG);
        m_turretOffsetDeg = new TunableDouble(
            "Tuning/Shooter/TurretOffsetDeg", ShooterConstants.TURRET_ANGLE_OFFSET_DEG);
        m_flywheelDiameterM = new TunableDouble(
            "Tuning/Shooter/FlywheelDiameterM", ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS);
        m_speedScalar = new TunableDouble(
            "Tuning/Shooter/SpeedScalar", ShooterConstants.SPEED_SCALAR_DEFAULT);
        m_speedPerMeterMps = new TunableDouble(
            "Tuning/Shooter/SpeedPerMeterMps", ShooterConstants.SPEED_PER_METER_DEFAULT);
        m_flywheelRpsOffset = new TunableDouble(
            "Tuning/Shooter/FlywheelRpsOffset", ShooterConstants.FLYWHEEL_RPS_OFFSET_DEFAULT);
        m_shootOnTheMoveGain = new TunableDouble(
            "Tuning/Shooter/ShootOnTheMoveGain", ShooterConstants.SHOOT_ON_THE_MOVE_GAIN);
        m_feederForwardPushMps = new TunableDouble(
            "Tuning/Shooter/FeederForwardPushMps", TurretConstants.FEEDER_FORWARD_PUSH_MPS);
        m_feederBackwardPushAt90Mps = new TunableDouble(
            "Tuning/Shooter/FeederBackwardPushAt90Mps", TurretConstants.FEEDER_BACKWARD_PUSH_AT_90_MPS);

        m_launchForwardOffsetM = new TunableDouble(
            "Tuning/Shooter/LaunchForwardOffsetM", ShooterConstants.BALL_LAUNCH_FRONT_OFFSET_METERS);
        m_launchLeftOffsetM = new TunableDouble(
            "Tuning/Shooter/LaunchLeftOffsetM", ShooterConstants.BALL_LAUNCH_LATERAL_OFFSET_METERS);
        m_launchHeightM = new TunableDouble(
            "Tuning/Shooter/LaunchHeightM", ShooterConstants.BALL_LAUNCH_HEIGHT_METERS);

        m_tiltCompensationGain = new TunableDouble(
            "Tuning/Shooter/TiltCompensationGain", ShooterConstants.TILT_COMPENSATION_GAIN);
        m_tiltRollOffsetDeg = new TunableDouble(
            "Tuning/Shooter/TiltRollOffsetDeg", ShooterConstants.TILT_ROLL_OFFSET_DEG);
        m_tiltPitchOffsetDeg = new TunableDouble(
            "Tuning/Shooter/TiltPitchOffsetDeg", ShooterConstants.TILT_PITCH_OFFSET_DEG);
    }

    /** Reads every dashboard value once, so a single solve can never see a half-changed set. */
    public AimTuning snapshot() {
        return new AimTuning(
            m_gravity.get(),
            m_descentMarginDeg.get(),
            ShooterConstants.RACK_MIN_ANGLE,
            ShooterConstants.RACK_MAX_ANGLE,
            m_rackOffsetDeg.get(),
            m_turretOffsetDeg.get(),
            m_flywheelDiameterM.get(),
            ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC,
            m_speedScalar.get(),
            m_speedPerMeterMps.get(),
            m_flywheelRpsOffset.get(),
            m_shootOnTheMoveGain.get(),
            m_feederForwardPushMps.get(),
            m_feederBackwardPushAt90Mps.get());
    }

    /** Forward offset of the ball's launch point from robot center, meters (positive = front). */
    public double launchForwardOffsetMeters() {
        return m_launchForwardOffsetM.get();
    }

    /** Lateral offset of the ball's launch point from robot center, meters (positive = left). */
    public double launchLeftOffsetMeters() {
        return m_launchLeftOffsetM.get();
    }

    /** Height of the ball's launch point above the floor, meters. */
    public double launchHeightMeters() {
        return m_launchHeightM.get();
    }

    /** Tilt-compensation authority, 0 to 1. 0 aims as if the robot were always level. */
    public double tiltCompensationGain() {
        return m_tiltCompensationGain.get();
    }

    /** Gyro roll reading on a robot known to be level, degrees. Subtracted before compensating. */
    public double tiltRollOffsetDeg() {
        return m_tiltRollOffsetDeg.get();
    }

    /** Gyro pitch reading on a robot known to be level, degrees. Subtracted before compensating. */
    public double tiltPitchOffsetDeg() {
        return m_tiltPitchOffsetDeg.get();
    }
}
