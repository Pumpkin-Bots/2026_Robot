package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants.VelocityEstimatorConstants;
import frc.robot.utils.TunableDouble;

/**
 * Estimates the robot's field-relative velocity by fusing three sources with complementary
 * strengths, for use by shoot-on-the-move aiming.
 *
 * <ul>
 *   <li><b>Pigeon 2 accelerometer (primary, fast).</b> Integrated forward every loop. It reacts
 *       instantly to a direction change, and unlike wheel odometry it still tells the truth while
 *       the wheels are slipping. Its weakness is that integrating a slightly-biased acceleration
 *       accumulates velocity error, which is why it alone cannot be trusted down a long straight.
 *   <li><b>Wheel odometry (correction, medium).</b> Pulled in as a first-order correction with time
 *       constant {@code WHEEL_TRUST_TAU_SECONDS}. This is the term that bounds IMU drift. It also
 *       decides, on its own, when the robot is standing still — see below.
 *   <li><b>Vision-fused pose (correction, slow).</b> Differentiated over a window to produce a
 *       noisy but <i>unbiased</i> velocity. Applied very weakly, it catches systematic wheel error
 *       (wrong wheel radius, carpet scrub) that wheel odometry cannot detect about itself.
 * </ul>
 *
 * <p>Net effect: the fast, high-frequency content of the estimate comes from the gyro and the slow,
 * low-frequency content from the wheels and cameras. Turn {@code WHEEL_TRUST_TAU_SECONDS} up to
 * lean further on the gyro, down to fall back toward plain wheel odometry.
 *
 * <p><b>The standstill is a special case, and an important one.</b> A complementary filter like this
 * one has a steady-state velocity error of exactly (accelerometer error) × {@code
 * WHEEL_TRUST_TAU_SECONDS} — that is its DC gain from one to the other. So every source of
 * acceleration error, whether a sensor offset or a moment of gravity leaking into a horizontal axis
 * as the chassis dips under braking, shows up as a phantom velocity that scales with the time
 * constant and takes about as long to bleed off. A shooter aiming off a phantom velocity leads a
 * target the robot is not moving toward, and that is most visible exactly where it is least
 * excusable: parked, lined up, about to shoot.
 *
 * <p>Two things address it, both keyed off wheel odometry noticing the robot has stopped. The
 * estimate is snapped to zero rather than allowed to decay there, so aim is correct the instant the
 * robot settles; and the standstill is used as the one clean opportunity to read the accelerometer's
 * bias directly, since a stopped robot's true acceleration is known to be zero. The bias is never
 * learned while driving, because nothing observed while driving distinguishes a real offset from a
 * transient.
 *
 * <p><b>Simulation note:</b> Phoenix's {@code Pigeon2SimState} cannot inject accelerometer readings,
 * so the IMU signals read zero under {@code simulateJava}. In simulation the acceleration input is
 * instead differentiated from the simulated wheel speeds, which keeps the entire filter and every
 * consumer downstream on exactly the same code path as the real robot.
 */
public class VelocityEstimator extends SubsystemBase {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pigeon2 m_pigeon;

    private final StatusSignal<LinearAcceleration> m_accelX;
    private final StatusSignal<LinearAcceleration> m_accelY;
    private final StatusSignal<Double> m_gravityX;
    private final StatusSignal<Double> m_gravityY;
    private final StatusSignal<AngularVelocity> m_yawRate;

    // Fused field-relative velocity — the output of this class.
    private double m_velXMps = 0.0;
    private double m_velYMps = 0.0;

    // Learned accelerometer bias, held in the ROBOT frame because that is where the physical
    // sensor offset actually lives. Keeping it robot-frame means it stays valid as the robot spins.
    private double m_biasXMps2 = 0.0;
    private double m_biasYMps2 = 0.0;

    private double m_lastUpdateTime = 0.0;

    // Standstill detection. The dwell is how long the wheels have continuously read stopped, and
    // m_atRest is that dwell having lasted long enough to believe.
    private double m_restDwellSeconds = 0.0;
    private boolean m_atRest = false;

    // Robot-frame acceleration actually fed to the integrator, kept for telemetry.
    private double m_accelRobotX = 0.0;
    private double m_accelRobotY = 0.0;

    // Previous field-relative wheel velocity, used only to synthesize acceleration in simulation.
    private double m_lastSimWheelVelX = 0.0;
    private double m_lastSimWheelVelY = 0.0;

    private final TunableDouble m_wheelTrustTau = new TunableDouble(
        "Tuning/Velocity/WheelTrustTauSec", VelocityEstimatorConstants.WHEEL_TRUST_TAU_SECONDS);
    private final TunableDouble m_accelBiasGain = new TunableDouble(
        "Tuning/Velocity/AccelBiasGain", VelocityEstimatorConstants.ACCEL_BIAS_GAIN);
    private final TunableDouble m_visionTrustTau = new TunableDouble(
        "Tuning/Velocity/VisionTrustTauSec", VelocityEstimatorConstants.VISION_TRUST_TAU_SECONDS);
    private final TunableDouble m_imuMountYawDeg = new TunableDouble(
        "Tuning/Velocity/ImuMountYawOffsetDeg", VelocityEstimatorConstants.IMU_MOUNT_YAW_OFFSET_DEG);
    private final TunableDouble m_pigeonOffsetForward = new TunableDouble(
        "Tuning/Velocity/PigeonOffsetForwardM", VelocityEstimatorConstants.PIGEON_OFFSET_FORWARD_METERS);
    private final TunableDouble m_pigeonOffsetLeft = new TunableDouble(
        "Tuning/Velocity/PigeonOffsetLeftM", VelocityEstimatorConstants.PIGEON_OFFSET_LEFT_METERS);

    // Yaw acceleration, differentiated from yaw rate and low-passed. Only used to remove the
    // tangential component the Pigeon picks up from being mounted off-center.
    private double m_yawAccelRadPerSec2 = 0.0;
    private double m_lastYawRateRadPerSec = 0.0;

    public VelocityEstimator(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_pigeon = drivetrain.getPigeon2();

        m_accelX = m_pigeon.getAccelerationX();
        m_accelY = m_pigeon.getAccelerationY();
        m_gravityX = m_pigeon.getGravityVectorX();
        m_gravityY = m_pigeon.getGravityVectorY();
        m_yawRate = m_pigeon.getAngularVelocityZWorld();

        // Request these explicitly: they are not on by default, and any optimizeBusUtilization()
        // call elsewhere would otherwise silence them.
        BaseStatusSignal.setUpdateFrequencyForAll(
            100, m_accelX, m_accelY, m_gravityX, m_gravityY, m_yawRate);

        m_lastUpdateTime = Timer.getFPGATimestamp();
    }

    /** Field-relative velocity of the robot's center, in meters per second. */
    public Translation2d getFieldRelativeVelocity() {
        return new Translation2d(m_velXMps, m_velYMps);
    }

    /**
     * Field-relative chassis speeds built from the fused estimate. Drop-in replacement for
     * {@code ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, heading)}.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return new ChassisSpeeds(m_velXMps, m_velYMps, getYawRateRadPerSec());
    }

    /**
     * Robot yaw rate in radians per second. Read straight off the Pigeon on a real robot, since the
     * gyro measures rotation far more directly than back-solving it from four module states.
     */
    public double getYawRateRadPerSec() {
        if (RobotBase.isSimulation()) {
            return m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
        }
        return m_yawRate.getValue().in(RadiansPerSecond);
    }

    /** True while the wheels say the robot is standing still. */
    public boolean isAtRest() {
        return m_atRest;
    }

    /**
     * Snaps the estimate to wheel odometry and clears the filter's transient state.
     *
     * <p>Deliberately leaves the learned accelerometer bias alone. That number is a property of the
     * chip, not of the filter's history — it is the same offset before and after a disable — and
     * throwing it away every time the robot is disabled means starting each enable having to learn
     * it over again from scratch.
     */
    public void reset() {
        ChassisSpeeds wheels = fieldRelativeWheelSpeeds();
        m_velXMps = wheels.vxMetersPerSecond;
        m_velYMps = wheels.vyMetersPerSecond;
        m_yawAccelRadPerSec2 = 0.0;
        m_lastYawRateRadPerSec = getYawRateRadPerSec();
        m_lastSimWheelVelX = wheels.vxMetersPerSecond;
        m_lastSimWheelVelY = wheels.vyMetersPerSecond;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_lastUpdateTime;
        m_lastUpdateTime = now;

        // A stale loop (first run, code pause, breakpoint) would integrate a huge bogus step.
        if (dt <= 0.0 || dt > 0.25) {
            reset();
            publishTelemetry(fieldRelativeWheelSpeeds(), 0.0, 0.0);
            return;
        }

        ChassisSpeeds wheelSpeeds = fieldRelativeWheelSpeeds();
        double headingRad = m_drivetrain.getState().Pose.getRotation().getRadians();

        // Acceleration is read before any of the early returns below, because a robot standing
        // still is precisely when the accelerometer is worth reading: whatever it says then is the
        // sensor's own offset.
        updateRobotFrameAcceleration(dt, wheelSpeeds, headingRad);
        updateRestDetector(wheelSpeeds, dt);

        // Pinned to wheel odometry while disabled. Going through reset() rather than just
        // overwriting the velocity also keeps the derivative history current, so the first enabled
        // loop differentiates against the present state instead of against whatever was true before
        // the disable. The standstill is put to work: a robot sitting on the field waiting for a
        // match is the longest clean look at the accelerometer's bias it will ever get, so by the
        // time it is enabled the offset is already learned.
        if (DriverStation.isDisabled()) {
            reset();
            updateBiasAtRest(dt);
            publishTelemetry(wheelSpeeds, 0.0, 0.0);
            return;
        }

        // --- Zero-velocity update ---
        // Integrating acceleration cannot discover on its own that the robot has stopped; it can
        // only be told. Without this, every scrap of error picked up during the stop — wheel slip,
        // the nose dipping under braking and leaking a little gravity into the accelerometer's
        // horizontal axes — sits in the estimate as a phantom velocity and takes
        // WHEEL_TRUST_TAU_SECONDS to bleed off, during which the shooter is still leading a target
        // the robot is no longer moving toward. The wheels know better here, and at a standstill
        // they are not merely better but exactly right, so take their word for it outright rather
        // than mixing it in at a gain.
        if (m_atRest) {
            m_velXMps = 0.0;
            m_velYMps = 0.0;
            updateBiasAtRest(dt);
            publishTelemetry(wheelSpeeds, 0.0, 0.0);
            return;
        }

        // --- Predict: integrate bias-corrected acceleration, rotated into the field frame ---
        double correctedX = m_accelRobotX - m_biasXMps2;
        double correctedY = m_accelRobotY - m_biasYMps2;
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        m_velXMps += (correctedX * cosH - correctedY * sinH) * dt;
        m_velYMps += (correctedX * sinH + correctedY * cosH) * dt;

        // --- Correct against wheel odometry ---
        double residualX = wheelSpeeds.vxMetersPerSecond - m_velXMps;
        double residualY = wheelSpeeds.vyMetersPerSecond - m_velYMps;
        double wheelGain = firstOrderGain(m_wheelTrustTau.get(), dt);
        m_velXMps += wheelGain * residualX;
        m_velYMps += wheelGain * residualY;

        // Note what is deliberately NOT here: learning the accelerometer bias from that residual.
        // While the robot is driving, the residual is dominated by things that are not bias — wheel
        // slip, and weight transfer tilting the chip so a little gravity leaks into its horizontal
        // axes — and both of those are at their worst during a hard stop. Integrating them is
        // windup, and because this filter's steady-state velocity error is the acceleration error
        // times the wheel-trust time constant, a bias wound up during a stop comes straight back out
        // as a phantom velocity that outlives the stop. The bias is learned at a standstill instead,
        // where the observation is clean — see updateBiasAtRest.

        // --- Correct against the vision-fused pose ---
        double visionVelX = 0.0;
        double visionVelY = 0.0;
        Translation2d visionVelocity = sampleVisionVelocity(now);
        if (visionVelocity != null) {
            visionVelX = visionVelocity.getX();
            visionVelY = visionVelocity.getY();
            double visionGain = firstOrderGain(m_visionTrustTau.get(), dt);
            m_velXMps += visionGain * (visionVelX - m_velXMps);
            m_velYMps += visionGain * (visionVelY - m_velYMps);
        }

        clampToPlausibleSpeed();
        publishTelemetry(wheelSpeeds, visionVelX, visionVelY);
    }

    /**
     * Fills {@link #m_accelRobotX}/{@link #m_accelRobotY} with robot-frame linear acceleration,
     * gravity removed.
     */
    private void updateRobotFrameAcceleration(double dt, ChassisSpeeds wheelSpeeds, double headingRad) {
        if (RobotBase.isSimulation()) {
            // Pigeon2SimState cannot inject acceleration, so synthesize it from the simulated wheel
            // speeds. Differentiating field-relative velocity and rotating the result into the robot
            // frame is what a perfect accelerometer bolted to a spinning chassis would report.
            double fieldAccelX = (wheelSpeeds.vxMetersPerSecond - m_lastSimWheelVelX) / dt;
            double fieldAccelY = (wheelSpeeds.vyMetersPerSecond - m_lastSimWheelVelY) / dt;
            m_lastSimWheelVelX = wheelSpeeds.vxMetersPerSecond;
            m_lastSimWheelVelY = wheelSpeeds.vyMetersPerSecond;
            m_accelRobotX =  fieldAccelX * Math.cos(headingRad) + fieldAccelY * Math.sin(headingRad);
            m_accelRobotY = -fieldAccelX * Math.sin(headingRad) + fieldAccelY * Math.cos(headingRad);
            return;
        }

        BaseStatusSignal.refreshAll(m_accelX, m_accelY, m_gravityX, m_gravityY, m_yawRate);

        // The Pigeon reports total specific force, which includes a full 1 g even sitting still.
        // The gravity vector signal is that 1 g expressed as a unit vector in sensor axes, so
        // subtracting it leaves only acceleration the robot is actually undergoing — and it stays
        // correct if the robot is tilted on a ramp or up on two wheels.
        double g = VelocityEstimatorConstants.GRAVITY_MPS2;
        double sensorX = m_accelX.getValue().in(MetersPerSecondPerSecond) - g * m_gravityX.getValue();
        double sensorY = m_accelY.getValue().in(MetersPerSecondPerSecond) - g * m_gravityY.getValue();

        double mountYawRad = Math.toRadians(m_imuMountYawDeg.get());
        double cosM = Math.cos(mountYawRad);
        double sinM = Math.sin(mountYawRad);
        double pigeonAccelX = sensorX * cosM - sensorY * sinM;
        double pigeonAccelY = sensorX * sinM + sensorY * cosM;

        // That is the acceleration of the Pigeon itself, which is not the robot's center unless the
        // chip happens to sit there. Rigid-body kinematics relate them:
        //     a_pigeon = a_center + alpha x r + omega x (omega x r)
        // so recovering the center means subtracting the tangential (alpha x r) and centripetal
        // (-omega^2 * r) terms the Pigeon picks up purely from being swung around.
        double omega = m_yawRate.getValue().in(RadiansPerSecond);
        updateYawAcceleration(omega, dt);
        double rx = m_pigeonOffsetForward.get();
        double ry = m_pigeonOffsetLeft.get();
        double alpha = m_yawAccelRadPerSec2;
        m_accelRobotX = pigeonAccelX + alpha * ry + omega * omega * rx;
        m_accelRobotY = pigeonAccelY - alpha * rx + omega * omega * ry;
    }

    /**
     * Decides whether the robot is standing still, from wheel odometry alone.
     *
     * <p>Wheel odometry is the right sensor for this and the IMU is the wrong one: an accelerometer
     * reads the same zero whether the robot is parked or gliding at 3 m/s, whereas a swerve's wheels
     * physically cannot report zero while the chassis is still translating.
     *
     * <p>The dwell is what makes it safe. Braking hard enough to skid drives the wheels to a halt
     * while the robot is genuinely still sliding, and for that instant the wheels do lie; requiring
     * them to hold the claim for {@code ZERO_VELOCITY_DWELL_SECONDS} outlasts the skid.
     *
     * <p>Note that a robot pivoting in place passes: its center really is going nowhere, which is
     * what this estimate is about. That case is excluded from bias learning, but not from the
     * velocity update, where zero is the right answer.
     */
    private void updateRestDetector(ChassisSpeeds wheelSpeeds, double dt) {
        double wheelSpeed = Math.hypot(
            wheelSpeeds.vxMetersPerSecond, wheelSpeeds.vyMetersPerSecond);
        if (wheelSpeed > VelocityEstimatorConstants.ZERO_VELOCITY_WHEEL_SPEED_MPS) {
            m_restDwellSeconds = 0.0;
            m_atRest = false;
            return;
        }
        m_restDwellSeconds += dt;
        m_atRest = m_restDwellSeconds >= VelocityEstimatorConstants.ZERO_VELOCITY_DWELL_SECONDS;
    }

    /**
     * Learns the accelerometer's offset, and only ever from a robot that is standing still.
     *
     * <p>At a standstill the robot's true acceleration is known to be exactly zero, so whatever the
     * accelerometer reports with gravity already removed is the sensor's offset, read directly. No
     * residual, no inference, nothing to confuse a transient for a bias — which is the failure the
     * old residual-driven version had, and the reason a hard stop used to leave the estimate holding
     * a phantom velocity.
     *
     * <p>Spinning in place is excluded: the Pigeon sits off-center, so what it reports there is
     * mostly centripetal acceleration that {@link #updateRobotFrameAcceleration} has had to subtract
     * back off, and the leftovers of that subtraction are not a clean look at the chip's offset.
     */
    private void updateBiasAtRest(double dt) {
        double biasGain = m_accelBiasGain.get();
        if (biasGain <= 0.0 || !m_atRest) {
            return;
        }
        if (Math.abs(getYawRateRadPerSec())
                > VelocityEstimatorConstants.ZERO_VELOCITY_YAW_RATE_RAD_PER_SEC) {
            return;
        }

        double gain = MathUtil.clamp(biasGain * dt, 0.0, 1.0);
        m_biasXMps2 = clampBias(m_biasXMps2 + gain * (m_accelRobotX - m_biasXMps2));
        m_biasYMps2 = clampBias(m_biasYMps2 + gain * (m_accelRobotY - m_biasYMps2));
    }

    private static double clampBias(double biasMps2) {
        return MathUtil.clamp(biasMps2,
            -VelocityEstimatorConstants.ACCEL_BIAS_MAX_MPS2,
            VelocityEstimatorConstants.ACCEL_BIAS_MAX_MPS2);
    }

    /** Differentiates yaw rate into yaw acceleration, low-passed to keep the derivative usable. */
    private void updateYawAcceleration(double yawRateRadPerSec, double dt) {
        double raw = (yawRateRadPerSec - m_lastYawRateRadPerSec) / dt;
        m_lastYawRateRadPerSec = yawRateRadPerSec;
        double gain = firstOrderGain(VelocityEstimatorConstants.YAW_ACCEL_FILTER_TAU_SECONDS, dt);
        m_yawAccelRadPerSec2 += gain * (raw - m_yawAccelRadPerSec2);
    }

    /**
     * Velocity derived by differentiating the vision-fused pose across a time window, or null when
     * no usable sample exists.
     *
     * <p>A vision correction moves the pose estimate discontinuously, and differentiating a jump
     * produces a velocity spike that is entirely fictional, so any sample too far from the current
     * estimate is thrown out rather than fed in.
     */
    private Translation2d sampleVisionVelocity(double now) {
        double window = VelocityEstimatorConstants.VISION_SAMPLE_WINDOW_SECONDS;
        if (window <= 0.0) {
            return null;
        }

        var pastPose = m_drivetrain.samplePoseAt(now - window);
        if (pastPose.isEmpty()) {
            return null;
        }

        Pose2d currentPose = m_drivetrain.getState().Pose;
        double vx = (currentPose.getX() - pastPose.get().getX()) / window;
        double vy = (currentPose.getY() - pastPose.get().getY()) / window;

        double disagreement = Math.hypot(vx - m_velXMps, vy - m_velYMps);
        if (disagreement > VelocityEstimatorConstants.VISION_REJECT_THRESHOLD_MPS) {
            return null;
        }
        return new Translation2d(vx, vy);
    }

    private ChassisSpeeds fieldRelativeWheelSpeeds() {
        var state = m_drivetrain.getState();
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    /** Discrete gain of a first-order filter with the given time constant. */
    private static double firstOrderGain(double tauSeconds, double dt) {
        if (tauSeconds <= 0.0) {
            return 1.0;
        }
        return MathUtil.clamp(dt / (tauSeconds + dt), 0.0, 1.0);
    }

    private void clampToPlausibleSpeed() {
        double max = VelocityEstimatorConstants.MAX_PLAUSIBLE_SPEED_MPS;
        double speed = Math.hypot(m_velXMps, m_velYMps);
        if (speed > max) {
            m_velXMps *= max / speed;
            m_velYMps *= max / speed;
        }
    }

    /**
     * Publishes every input and intermediate of the filter, not just its output. When aiming is off
     * on the real robot, the first question is always whether the velocity estimate or the ballistics
     * is to blame — plotting Fused vs Wheel vs Vision side by side answers that immediately.
     */
    private void publishTelemetry(ChassisSpeeds wheelSpeeds, double visionVelX, double visionVelY) {
        SmartDashboard.putNumber("Velocity/FusedX", m_velXMps);
        SmartDashboard.putNumber("Velocity/FusedY", m_velYMps);
        SmartDashboard.putNumber("Velocity/FusedSpeed", Math.hypot(m_velXMps, m_velYMps));
        SmartDashboard.putNumber("Velocity/WheelX", wheelSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Velocity/WheelY", wheelSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Velocity/VisionX", visionVelX);
        SmartDashboard.putNumber("Velocity/VisionY", visionVelY);
        SmartDashboard.putNumber("Velocity/ImuAccelRobotX", m_accelRobotX);
        SmartDashboard.putNumber("Velocity/ImuAccelRobotY", m_accelRobotY);
        SmartDashboard.putNumber("Velocity/AccelBiasX", m_biasXMps2);
        SmartDashboard.putNumber("Velocity/AccelBiasY", m_biasYMps2);
        SmartDashboard.putNumber("Velocity/YawRateRadPerSec", getYawRateRadPerSec());
        SmartDashboard.putNumber("Velocity/YawAccelRadPerSec2", m_yawAccelRadPerSec2);
        SmartDashboard.putBoolean("Velocity/AtRest", m_atRest);
        // What the learned bias is costing in velocity terms. The filter's steady-state velocity
        // error is the acceleration error times the wheel-trust time constant, so this is the
        // standing error the bias alone would produce — and the number to watch if the shooter is
        // leading a target the robot is not moving toward.
        SmartDashboard.putNumber("Velocity/BiasVelocityErrorMps",
            Math.hypot(m_biasXMps2, m_biasYMps2) * m_wheelTrustTau.get());
    }
}
