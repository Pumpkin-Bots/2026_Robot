package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.Constants.ShooterConstants;

/**
 * Learns, from the robot's own shots, how far the flywheel has actually slowed at the instant each
 * ball leaves it — and biases the commanded speed up by that much, so the ball departs at the speed
 * the aim solution asked for.
 *
 * <h2>What is being corrected</h2>
 *
 * <p>The aiming math computes the speed the ball has to <i>leave</i> at, and the velocity loop holds
 * the wheel there — right up until a ball arrives. The ball and the wheel surface match speeds
 * through the contact, and the ball leaves carrying whatever the wheel is doing at the moment of
 * separation, which is several RPS below where it started. Fire continuously and the wheel never
 * fully recovers between balls, so the deficit compounds; fire all match and it compounds further as
 * the battery sags and the motor has less voltage headroom to fight back with.
 *
 * <p>So the quantity worth learning is not "how deep is the dip" in the abstract. It is
 * <b>how far below the commanded setpoint the wheel sits at the moment of separation</b> — the
 * trough. Bias the command up by exactly that, and the trough lands on the target.
 *
 * <h2>Finding each ball</h2>
 *
 * <p>Everything works on the tracking error, {@code measured − setpoint}, rather than on raw speed,
 * because the setpoint is recomputed every loop as the robot drives and raw speed is always moving
 * for reasons that have nothing to do with shooting.
 *
 * <p>Balls are found as <b>turning points</b> in that error, not as excursions from a baseline. The
 * detector holds one of two states and needs a confirmed reversal to leave either:
 *
 * <ul>
 *   <li><b>Falling</b> — tracking the running minimum. When the error climbs back
 *       {@code ShotReboundRps} above that minimum, the minimum is confirmed as a trough and
 *       <i>one ball</i> is recorded.
 *   <li><b>Rising</b> — tracking the running maximum. When the error drops {@code ShotDetectDropRps}
 *       below that maximum, a new descent has begun.
 * </ul>
 *
 * <p>The rebound is what separates one ball from two. During rapid fire the wheel climbs only part
 * of the way back between balls — a clear step up, nowhere near the original speed — and a detector
 * waiting for a return to baseline sees one long trench instead of five separate balls, times out,
 * and learns nothing at all from the busiest part of the match. Waiting for a <i>reversal</i>
 * instead means each ball is counted on its own no matter how little the wheel recovered, which is
 * exactly the case that matters most.
 *
 * <h2>What is learned</h2>
 *
 * <p>The trough depth below setpoint, binned by commanded speed. Binning is not optional: the net
 * dip is badly non-linear in speed, because the motor's ability to push back during the contact
 * collapses as it approaches free speed. Near the bottom of the range the motor replaces nearly
 * everything the ball takes; near the top it replaces almost none of it. A single learned number,
 * however it is scaled, is wrong at one end or the other.
 *
 * <p>Bins fill independently and interpolate between whichever ones have data. Outside the range
 * that has been measured, the interpolation clamps rather than extrapolating — an underestimate,
 * deliberately, because under-compensating costs a short shot while a runaway extrapolation costs a
 * wild one. Until any bin has data at all, the fallback is
 * {@link #physicsSeedFraction()}, computed from the flywheel's inertia; the first few balls
 * replace it with measurement.
 *
 * <h2>Why this is stable</h2>
 *
 * <p>The trough is measured against the <i>already-compensated</i> setpoint, which sounds circular
 * and is not: the physical drop a ball causes does not depend on how much bias has been added, so
 * the measurement stays a correct estimate of the drop whatever the compensation is currently doing.
 * There is a second-order loop — running faster leaves the motor less headroom, so the drop grows a
 * little — but that gain is well under one, and it converges. {@code MaxCompensationRps} bounds it
 * regardless.
 *
 * <p>The one case that genuinely cannot converge is a wheel already at full voltage: raising the
 * setpoint cannot make it spin faster, so learning would wind up against a shot that is simply not
 * achievable. Learning is frozen while the motor is saturated.
 *
 * <h2>Simulation</h2>
 *
 * <p>Inert in simulation. maple-sim's projectiles leave at exactly the wheel's speed — there is no
 * ball-versus-wheel interaction to lose energy to — so there is no droop to compensate, and biasing
 * the speed up there would simply make every simulated shot overshoot.
 */
public class FlywheelDroopCompensator {

    // ---- Detector internals ----
    // Not in Constants because none of these are field-tuned; they shape the detector rather than
    // the shot, and the numbers that change how the robot shoots all live in ShooterConstants.

    /** Telemetry is published at about this period however fast the detector is being sampled. */
    private static final double PUBLISH_PERIOD_SECONDS = 0.02;

    /** Width of each learning bin, in commanded motor RPS. */
    private static final double BIN_WIDTH_RPS = 10.0;

    /** Bins spanning zero up to the commanded ceiling. */
    private static final int BIN_COUNT =
        (int) Math.ceil(ShooterConstants.FLYWHEEL_MAX_REV_PER_SEC / BIN_WIDTH_RPS) + 1;

    /**
     * Filtered duty cycle past which the motor counts as genuinely out of headroom.
     *
     * <p>Deliberately compared against a filtered figure rather than the instantaneous one. The
     * motor floors itself for a few milliseconds recovering from every single ball — at high
     * commanded speeds {@code kV·rps} alone is most of the bus, so the velocity loop pins the
     * output the moment a ball knocks the wheel down. That is the loop working, not the motor
     * being out of road, and treating it as saturation would freeze learning on nearly every shot
     * at exactly the speeds where the droop is worst.
     *
     * <p>What this is trying to catch instead is a wheel flat out <i>continuously</i> — one that
     * cannot reach the speed it is already being asked for, where adding more setpoint produces no
     * more speed and only inflates what gets learned.
     */
    private static final double SATURATION_DUTY_CYCLE = 0.98;

    /**
     * Time constant of the duty-cycle low-pass, seconds. Long enough to average across several
     * balls, so per-ball recovery spikes wash out and only a sustained pin registers.
     */
    private static final double DUTY_FILTER_TAU_SECONDS = 0.5;

    /**
     * A descent lasting longer than this is not a ball. Contact is over in tens of milliseconds;
     * anything still falling after a quarter second is a jam, a stall, or the loop settling
     * somewhere new, and the trough it eventually reaches means nothing.
     */
    private static final double MAX_DESCENT_SECONDS = 0.25;

    private final TunableDouble m_compensationGain = new TunableDouble(
        "Tuning/Flywheel/CompensationGain", ShooterConstants.FLYWHEEL_COMPENSATION_GAIN);
    private final TunableDouble m_learningRate = new TunableDouble(
        "Tuning/Flywheel/LearningRate", ShooterConstants.FLYWHEEL_DROOP_LEARNING_RATE);
    private final TunableDouble m_detectDropRps = new TunableDouble(
        "Tuning/Flywheel/ShotDetectDropRps", ShooterConstants.FLYWHEEL_SHOT_DETECT_DROP_RPS);
    private final TunableDouble m_reboundRps = new TunableDouble(
        "Tuning/Flywheel/ShotReboundRps", ShooterConstants.FLYWHEEL_SHOT_REBOUND_RPS);
    private final TunableDouble m_maxCompensationRps = new TunableDouble(
        "Tuning/Flywheel/MaxCompensationRps", ShooterConstants.FLYWHEEL_MAX_COMPENSATION_RPS);
    private final TunableDouble m_atSpeedToleranceRps = new TunableDouble(
        "Tuning/Flywheel/AtSpeedToleranceRps", ShooterConstants.FLYWHEEL_AT_SPEED_TOLERANCE_RPS);
    private final TunableDouble m_minDetectRps = new TunableDouble(
        "Tuning/Flywheel/MinDetectRps", ShooterConstants.FLYWHEEL_MIN_DETECT_RPS);
    private final TunableDouble m_maxPlausibleDroopRps = new TunableDouble(
        "Tuning/Flywheel/MaxPlausibleDroopRps", ShooterConstants.FLYWHEEL_MAX_PLAUSIBLE_DROOP_RPS);
    private final TunableDouble m_maxSetpointSlewRpsPerSec = new TunableDouble(
        "Tuning/Flywheel/MaxSetpointSlewRpsPerSec",
        ShooterConstants.FLYWHEEL_MAX_SETPOINT_SLEW_RPS_PER_SEC);

    // ---- What has been learned ----
    // Trough depth below setpoint, in motor RPS, per commanded-speed bin. Only bins with at least
    // one ball in them are published into the interpolator, so an unvisited speed range falls back
    // to whichever measured bins bracket it rather than to a guess.
    private final double[] m_binDeficitRps = new double[BIN_COUNT];
    private final int[] m_binSamples = new int[BIN_COUNT];
    private final InterpolatingDoubleTreeMap m_deficitBySpeed = new InterpolatingDoubleTreeMap();
    private boolean m_haveAnyBin = false;

    /** Fallback until a single ball has been measured — see {@link #physicsSeedFraction()}. */
    private final double m_seedFraction;

    // ---- Most recent ball, for telemetry ----
    private double m_lastTroughDeficitRps = 0.0;
    private double m_lastPerBallDropRps = 0.0;
    private double m_lastContactSeconds = 0.0;
    private double m_lastBallIntervalSeconds = 0.0;
    private double m_lastBallTime = Double.NEGATIVE_INFINITY;
    private int m_shotCount = 0;
    private int m_rejectedCount = 0;

    // ---- Live state ----
    private double m_compensationRps = 0.0;
    private boolean m_atSpeed = false;
    private boolean m_saturated = false;
    private double m_filteredDuty = 0.0;

    // True once the wheel has reached speed since it was last parked. Until then nothing counts as
    // a ball, which is what keeps spin-up — a huge, ball-shaped fall in tracking error — out of the
    // learned numbers.
    private boolean m_armed = false;

    // ---- Turning-point detector ----
    // Exactly one of two states. m_extremeErrorRps is the running minimum while falling and the
    // running maximum while rising; a confirmed reversal is what switches between them.
    private boolean m_falling = false;
    private double m_extremeErrorRps = 0.0;
    private double m_extremeTime = 0.0;
    private double m_descentPeakErrorRps = 0.0;
    private double m_descentStartTime = 0.0;
    private double m_descentSetpointRps = 0.0;
    private boolean m_descentDisturbed = false;

    private double m_lastSetpointRps = 0.0;
    private double m_lastUpdateTime = -1.0;
    private double m_lastPublishTime = Double.NEGATIVE_INFINITY;

    public FlywheelDroopCompensator() {
        m_seedFraction = RobotBase.isSimulation() ? 0.0 : physicsSeedFraction();
    }

    /**
     * The droop the wheel's inertia says to expect, as a fraction of commanded speed, before a
     * single ball has been measured.
     *
     * <p>Energy accounting. A ball leaving at {@code v} carries {@code ½·m·v²}, and at the assumed
     * efficiency the wheel gives up {@code ½·m·v²/η} to produce it. Taking that out of a wheel
     * spinning at {@code ω} costs {@code Δω = ½·m·v²/(η·I·ω)}. Substituting the muzzle speed the
     * rest of this code uses, {@code v = π·d·rps}, and {@code ω = 2π·rps}, every factor of the speed
     * cancels and what is left is a constant:
     *
     * <pre>Δrps / rps = m·d² / (8·I·η)</pre>
     *
     * <p>This is a <i>cold start only</i>. It ignores the motor's push-back during the contact,
     * which is the term that makes the real droop strongly speed-dependent, so it is wrong at both
     * ends of the range by construction — it exists so the first ball of a match is roughly
     * compensated rather than not at all, and the learned bins replace it as they fill.
     *
     * <p>Both inputs are referenced to the MOTOR, because the droop this seeds is measured off the
     * motor's own velocity signal: the reflected inertia and the motor-referenced effective
     * diameter. The answer is the same either way — the gear ratio cancels out of the fraction
     * exactly as the speed does — but mixing the two frames would not.
     */
    public static double physicsSeedFraction() {
        double d = ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS;
        double denominator = 8.0
            * ShooterConstants.FLYWHEEL_ROTOR_MOI_KG_M2
            * ShooterConstants.FLYWHEEL_SHOT_ENERGY_EFFICIENCY;
        if (denominator <= 0.0) {
            return 0.0;
        }
        return ShooterConstants.FUEL_MASS_KG * d * d / denominator;
    }

    /**
     * Folds one velocity sample into the estimate.
     *
     * <p>Call this <b>far faster than the main robot loop</b> — a ball is in contact with the wheel
     * for something like 10-25 ms, so at 50 Hz the trough is at most one sample wide and usually
     * missed entirely. See {@code ShooterSubsystem.sampleFlywheelDroop()}.
     *
     * @param setpointRps what the velocity loop was actually told to hold, in motor RPS — the
     *     compensated figure, not the raw aim target, since that is what the wheel is chasing
     * @param measuredRps what the wheel is actually doing, in motor RPS
     * @param dutyCycle   the motor's applied output as a fraction of supply, for the saturation
     *     check; learning is frozen when the motor has no headroom left to give
     */
    public void update(double setpointRps, double measuredRps, double dutyCycle) {
        double now = Timer.getFPGATimestamp();
        double dt = m_lastUpdateTime < 0.0 ? 0.005 : Math.max(1e-4, now - m_lastUpdateTime);
        m_lastUpdateTime = now;

        double error = measuredRps - setpointRps;
        double setpointSlewRpsPerSec = Math.abs(setpointRps - m_lastSetpointRps) / dt;
        m_lastSetpointRps = setpointRps;

        double dutyAlpha = dt / (DUTY_FILTER_TAU_SECONDS + dt);
        m_filteredDuty += dutyAlpha * (Math.abs(dutyCycle) - m_filteredDuty);
        m_saturated = m_filteredDuty >= SATURATION_DUTY_CYCLE;

        if (setpointRps < m_minDetectRps.get()) {
            // Parked, or on the way there. Nothing measurable happens below this speed.
            m_armed = false;
            m_falling = false;
            m_atSpeed = false;
            m_extremeErrorRps = error;
            m_compensationRps = 0.0;
            publishThrottled(setpointRps, measuredRps, error, now);
            return;
        }

        m_atSpeed = Math.abs(error) <= m_atSpeedToleranceRps.get();
        if (m_atSpeed) {
            m_armed = true;
        }

        boolean setpointSteady = setpointSlewRpsPerSec <= m_maxSetpointSlewRpsPerSec.get();
        if (!setpointSteady) {
            // The setpoint jumped. The error is about to go large and negative for reasons that
            // have nothing to do with a ball, and it stays ball-shaped for the whole time the wheel
            // takes to catch up — so detection goes off until it has, rather than skipping one
            // sample and walking straight into the transient.
            m_armed = false;
            m_descentDisturbed = true;
        }

        trackTurningPoints(error, setpointRps, now);

        m_compensationRps = compensationRps(setpointRps);
        publishThrottled(setpointRps, measuredRps, error, now);
    }

    /**
     * The turning-point state machine: one ball per confirmed trough.
     *
     * <p>A trough is only confirmed once the error has climbed {@code ShotReboundRps} back off it.
     * That rebound is the wheel starting to recover, which is the physical signature of the ball
     * having let go — and requiring only a rebound, rather than a return to where the wheel started,
     * is what lets a burst of balls be counted individually instead of as one long sag.
     */
    private void trackTurningPoints(double error, double setpointRps, double now) {
        if (m_falling) {
            if (error < m_extremeErrorRps) {
                m_extremeErrorRps = error;
                // Stamped each time the trough deepens, so it holds the moment the wheel bottomed
                // out — which is the moment the ball separated.
                m_extremeTime = now;
                return;
            }
            if (error >= m_extremeErrorRps + m_reboundRps.get()) {
                recordBall(now);
                m_falling = false;
                m_extremeErrorRps = error;
                m_extremeTime = now;
            }
            return;
        }

        // Ties refresh the timestamp as well as the value, so a wheel sitting flat on its setpoint
        // keeps m_extremeTime current — that stamp is what dates the start of the next descent.
        if (error >= m_extremeErrorRps) {
            m_extremeErrorRps = error;
            m_extremeTime = now;
            return;
        }
        if (error <= m_extremeErrorRps - m_detectDropRps.get()) {
            // A new descent. Whatever the error had climbed to is this ball's starting point, and
            // the descent is dated from when the error LEFT that peak rather than from now — the
            // wheel has already been falling for however long it took to cross the threshold, and
            // counting from here would under-report the contact time by exactly that much. Contact
            // time is an input to the physics, so the bias is worth removing.
            m_falling = true;
            m_descentPeakErrorRps = m_extremeErrorRps;
            m_descentStartTime = m_extremeTime;
            m_descentSetpointRps = setpointRps;
            m_descentDisturbed = !m_armed;
            m_extremeErrorRps = error;
            m_extremeTime = now;
        }
    }

    /** Validates one confirmed trough and, if it is a real ball, learns from it. */
    private void recordBall(double now) {
        double troughDeficitRps = -m_extremeErrorRps;
        double perBallDropRps = m_descentPeakErrorRps - m_extremeErrorRps;
        double descentSeconds = m_extremeTime - m_descentStartTime;

        boolean valid = m_armed
            && !m_descentDisturbed
            && perBallDropRps >= m_detectDropRps.get()
            && perBallDropRps <= m_maxPlausibleDroopRps.get()
            && descentSeconds <= MAX_DESCENT_SECONDS
            && m_descentSetpointRps >= m_minDetectRps.get();

        if (!valid) {
            m_rejectedCount++;
            return;
        }

        m_lastTroughDeficitRps = troughDeficitRps;
        m_lastPerBallDropRps = perBallDropRps;
        m_lastContactSeconds = descentSeconds;
        m_lastBallIntervalSeconds = Double.isInfinite(m_lastBallTime) ? 0.0 : now - m_lastBallTime;
        m_lastBallTime = now;
        m_shotCount++;

        // A wheel flat out CONTINUOUSLY cannot be made to spin faster by asking for more, so
        // learning a bigger deficit there would only wind the command up against a shot that is not
        // achievable at this gearing. The ball still counts — it just doesn't move the estimate.
        //
        // Note this is the filtered duty cycle, not the instantaneous one: the motor pins itself
        // briefly recovering from every ball, and freezing on that would block learning at exactly
        // the high speeds where the droop is worst. See SATURATION_DUTY_CYCLE.
        if (m_saturated) {
            return;
        }

        // Only a genuine deficit teaches anything. A trough ABOVE the setpoint means the wheel was
        // running fast and a ball brought it back toward where it should have been, which is not a
        // reason to command less.
        learnDeficit(m_descentSetpointRps, Math.max(0.0, troughDeficitRps));
    }

    /** Folds one measured trough depth into the bin for the speed it was measured at. */
    private void learnDeficit(double setpointRps, double deficitRps) {
        int bin = binFor(setpointRps);
        if (m_binSamples[bin] == 0) {
            // Nothing to average against yet — take the measurement whole rather than creeping up
            // to it from a zero that was never an estimate of anything.
            m_binDeficitRps[bin] = deficitRps;
        } else {
            double alpha = MathUtil.clamp(m_learningRate.get(), 0.0, 1.0);
            m_binDeficitRps[bin] += alpha * (deficitRps - m_binDeficitRps[bin]);
        }
        m_binSamples[bin]++;
        m_deficitBySpeed.put(binCenterRps(bin), m_binDeficitRps[bin]);
        m_haveAnyBin = true;
    }

    private static int binFor(double setpointRps) {
        return MathUtil.clamp((int) (setpointRps / BIN_WIDTH_RPS), 0, BIN_COUNT - 1);
    }

    private static double binCenterRps(int bin) {
        return (bin + 0.5) * BIN_WIDTH_RPS;
    }

    /**
     * How much to add to a commanded flywheel speed so the ball leaves at that speed, in motor RPS.
     * Always zero or positive, and never more than {@code MaxCompensationRps}.
     *
     * @param targetRps the speed the aim solution asked for, in motor RPS
     */
    public double compensationRps(double targetRps) {
        if (targetRps < m_minDetectRps.get()) {
            return 0.0;
        }
        return MathUtil.clamp(
            m_compensationGain.get() * learnedDeficitRps(targetRps),
            0.0,
            Math.max(0.0, m_maxCompensationRps.get()));
    }

    /**
     * The learned trough deficit at a commanded speed, in motor RPS, before the gain and the clamp.
     *
     * <p>Between the lowest and highest bins that have data, this is the measured curve — whatever
     * shape it turned out to be. Nothing here assumes the deficit is proportional to speed, or
     * quadratic, or anything else; the bins are just points on a curve and they take the shape the
     * balls put there. That matters because the real shape is neither linear nor a fixed fraction:
     * the ball's drain grows linearly with speed while the motor's push-back during the contact
     * collapses toward zero at free speed, so the total bends upward, gently at first and then
     * sharply once the stator limit stops binding and the loop runs out of voltage.
     *
     * <p>Beyond the measured range it continues on the slope of the two nearest populated bins
     * rather than flattening off. That is worth doing precisely because the curve bends <i>up</i>:
     * holding the last measured value would badly under-serve the long shots that need it most. A
     * straight line drawn off the end of a curve that bends upward still lands below the truth, so
     * the extrapolation errs short, which is the recoverable direction. A slope that comes out zero
     * or negative is noise rather than physics, and is not extrapolated on at all.
     *
     * <p>With a single populated bin there is no slope to measure, so it falls back to scaling that
     * one measurement with speed — the ball-drain half of the physics, and the best a single point
     * supports. With none, the cold-start physics seed.
     */
    public double learnedDeficitRps(double targetRps) {
        if (!m_haveAnyBin) {
            return m_seedFraction * targetRps;
        }

        int lowest = edgePopulatedBin(true);
        int highest = edgePopulatedBin(false);

        if (lowest == highest) {
            double center = binCenterRps(lowest);
            return center > 0.0 ? Math.max(0.0, m_binDeficitRps[lowest] * targetRps / center) : 0.0;
        }

        double lowCenter = binCenterRps(lowest);
        double highCenter = binCenterRps(highest);
        if (targetRps >= lowCenter && targetRps <= highCenter) {
            return Math.max(0.0, m_deficitBySpeed.get(targetRps));
        }

        if (targetRps > highCenter) {
            int inner = neighbourPopulatedBin(highest, true);
            double slope = binSlope(inner, highest);
            return slope <= 0.0
                ? m_binDeficitRps[highest]
                : m_binDeficitRps[highest] + slope * (targetRps - highCenter);
        }

        int inner = neighbourPopulatedBin(lowest, false);
        double slope = binSlope(lowest, inner);
        return slope <= 0.0
            ? m_binDeficitRps[lowest]
            : Math.max(0.0, m_binDeficitRps[lowest] - slope * (lowCenter - targetRps));
    }

    /** Lowest or highest bin holding at least one measured ball. Only valid once one does. */
    private int edgePopulatedBin(boolean lowest) {
        for (int i = 0; i < BIN_COUNT; i++) {
            int bin = lowest ? i : BIN_COUNT - 1 - i;
            if (m_binSamples[bin] > 0) {
                return bin;
            }
        }
        return 0;
    }

    /** The next populated bin inward from an edge, for measuring the slope there. */
    private int neighbourPopulatedBin(int from, boolean searchDownward) {
        int step = searchDownward ? -1 : 1;
        for (int bin = from + step; bin >= 0 && bin < BIN_COUNT; bin += step) {
            if (m_binSamples[bin] > 0) {
                return bin;
            }
        }
        return from;
    }

    /** Deficit gradient between two populated bins, RPS of sag per RPS of commanded speed. */
    private double binSlope(int lowBin, int highBin) {
        double span = binCenterRps(highBin) - binCenterRps(lowBin);
        if (span <= 0.0) {
            return 0.0;
        }
        return (m_binDeficitRps[highBin] - m_binDeficitRps[lowBin]) / span;
    }

    /** True when the wheel is within {@code AtSpeedToleranceRps} of the speed it is being held to. */
    public boolean isAtSpeed() {
        return m_atSpeed;
    }

    /** True while the motor has no voltage headroom left — the shot may simply be unachievable. */
    public boolean isSaturated() {
        return m_saturated;
    }

    /** Balls the detector has accepted since boot. Compare against balls actually fired. */
    public int getShotCount() {
        return m_shotCount;
    }

    /** Trough depth below setpoint on the most recent ball, motor RPS — what is being learned. */
    public double getLastTroughDeficitRps() {
        return m_lastTroughDeficitRps;
    }

    /**
     * Speed the wheel lost during the most recent ball's contact alone, motor RPS.
     *
     * <p>Distinct from {@link #getLastTroughDeficitRps()} during rapid fire: the incremental drop
     * stays roughly constant ball to ball while the absolute deficit accumulates, and it is the
     * absolute one that sets how fast the ball leaves.
     */
    public double getLastPerBallDropRps() {
        return m_lastPerBallDropRps;
    }

    /**
     * Ball contact time from the most recent accepted ball, seconds — descent start to trough.
     * Expect something in the 10-25 ms range; far more than that means the detector is picking up
     * something that is not a ball.
     */
    public double getLastContactSeconds() {
        return m_lastContactSeconds;
    }

    /** Number of speed bins that have at least one measured ball in them. */
    public int getPopulatedBinCount() {
        int populated = 0;
        for (int samples : m_binSamples) {
            if (samples > 0) {
                populated++;
            }
        }
        return populated;
    }

    /**
     * Throws away everything learned and goes back to the physics seed. Nothing calls this
     * automatically — what the robot learns in autonomous is still true in teleop, and carrying the
     * estimate across a whole match is the entire point.
     */
    public void reset() {
        java.util.Arrays.fill(m_binDeficitRps, 0.0);
        java.util.Arrays.fill(m_binSamples, 0);
        m_deficitBySpeed.clear();
        m_haveAnyBin = false;
        m_lastTroughDeficitRps = 0.0;
        m_lastPerBallDropRps = 0.0;
        m_lastContactSeconds = 0.0;
        m_lastBallIntervalSeconds = 0.0;
        m_lastBallTime = Double.NEGATIVE_INFINITY;
        m_shotCount = 0;
        m_rejectedCount = 0;
        m_compensationRps = 0.0;
        m_armed = false;
        m_falling = false;
    }

    /**
     * Publishes at roughly 50 Hz however fast {@link #update} is being called.
     *
     * <p>The detector runs at 200 Hz because that is what it takes to see a ball; NetworkTables has
     * no such need, and pushing a dozen keys four times a loop would cost more than the measurement
     * is worth. Every per-ball row is a latched value, so a slow reader misses nothing — and the
     * dip's real shape is in the Hoot log at the full signal rate regardless.
     */
    private void publishThrottled(
            double setpointRps, double measuredRps, double errorRps, double now) {
        if (now - m_lastPublishTime < PUBLISH_PERIOD_SECONDS) {
            return;
        }
        m_lastPublishTime = now;

        SmartDashboard.putNumber("Shooter/Flywheel/SetpointRps", setpointRps);
        SmartDashboard.putNumber("Shooter/Flywheel/MeasuredRps", measuredRps);
        SmartDashboard.putNumber("Shooter/Flywheel/ErrorRps", errorRps);
        SmartDashboard.putNumber("Shooter/Flywheel/CompensationRps", m_compensationRps);

        SmartDashboard.putNumber("Shooter/Flywheel/LastTroughDeficitRps", m_lastTroughDeficitRps);
        SmartDashboard.putNumber("Shooter/Flywheel/LastPerBallDropRps", m_lastPerBallDropRps);
        SmartDashboard.putNumber("Shooter/Flywheel/LastContactSec", m_lastContactSeconds);
        SmartDashboard.putNumber("Shooter/Flywheel/LastBallIntervalSec", m_lastBallIntervalSeconds);
        SmartDashboard.putNumber("Shooter/Flywheel/BallsPerSec",
            m_lastBallIntervalSeconds > 0.0 ? 1.0 / m_lastBallIntervalSeconds : 0.0);

        SmartDashboard.putNumber("Shooter/Flywheel/LearnedDeficitHereRps",
            learnedDeficitRps(setpointRps));
        SmartDashboard.putNumber("Shooter/Flywheel/SeedDroopFraction", m_seedFraction);
        SmartDashboard.putNumber("Shooter/Flywheel/ShotCount", m_shotCount);
        SmartDashboard.putNumber("Shooter/Flywheel/RejectedCount", m_rejectedCount);
        SmartDashboard.putNumber("Shooter/Flywheel/PopulatedBins", getPopulatedBinCount());

        SmartDashboard.putNumber("Shooter/Flywheel/DutyCycleAvg", m_filteredDuty);

        SmartDashboard.putBoolean("Shooter/Flywheel/AtSpeed", m_atSpeed);
        SmartDashboard.putBoolean("Shooter/Flywheel/Falling", m_falling);
        SmartDashboard.putBoolean("Shooter/Flywheel/Saturated", m_saturated);
    }
}
