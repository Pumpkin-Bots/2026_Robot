package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.constants.Constants.ShooterConstants;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Pins down what the droop compensator has to get right to be worth having: it must count balls
 * correctly — including a burst the wheel never fully recovers from, which is the case that matters
 * most and the one a baseline-crossing detector cannot see — learn only from balls, and never be
 * able to run the flywheel away.
 *
 * <p>Time is stepped by hand through {@link SimHooks} rather than slept through, at the 5 ms the
 * real detector samples at, so a ball's descent and rebound happen over a deterministic and
 * realistic number of samples.
 */
class FlywheelDroopCompensatorTest {

    /** Matches the 200 Hz the real sampler runs at. */
    private static final double LOOP = 0.005;
    private static final double SETPOINT = 40.0;

    /** Duty cycle reported when the motor has headroom left, so learning is not frozen. */
    private static final double UNSATURATED = 0.5;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0), "HAL should initialize for the FPGA clock");
        SimHooks.pauseTiming();
    }

    @AfterAll
    static void shutdownHal() {
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    /** One sample: advance the clock, then hand the compensator a reading. */
    private static void step(FlywheelDroopCompensator c, double setpoint, double measured) {
        step(c, setpoint, measured, UNSATURATED);
    }

    private static void step(
            FlywheelDroopCompensator c, double setpoint, double measured, double duty) {
        SimHooks.stepTiming(LOOP);
        c.update(setpoint, measured, duty);
    }

    /** Holds the wheel exactly on setpoint long enough to arm the detector. */
    private static void settleAtSpeed(FlywheelDroopCompensator c) {
        settleAtSpeed(c, SETPOINT);
    }

    private static void settleAtSpeed(FlywheelDroopCompensator c, double setpoint) {
        for (int i = 0; i < 100; i++) {
            step(c, setpoint, setpoint);
        }
    }

    /**
     * One ball: the error falls from {@code fromError} to {@code toError} over a few samples, then
     * climbs back to {@code reboundError}. The rebound is what confirms the trough — see the
     * detector's docs — so it has to clear {@code FLYWHEEL_SHOT_REBOUND_RPS}.
     */
    private static void oneBall(
            FlywheelDroopCompensator c, double fromError, double toError, double reboundError) {
        oneBall(c, SETPOINT, fromError, toError, reboundError);
    }

    private static void oneBall(
            FlywheelDroopCompensator c,
            double setpoint,
            double fromError,
            double toError,
            double reboundError) {
        double depth = fromError - toError;
        // Descent, over ~4 samples (20 ms), which is the real contact timescale.
        for (int i = 1; i <= 4; i++) {
            step(c, setpoint, setpoint + fromError - depth * i / 4.0);
        }
        // Recovery toward the rebound level, over ~10 samples.
        double climb = reboundError - toError;
        for (int i = 1; i <= 10; i++) {
            step(c, setpoint, setpoint + toError + climb * i / 10.0);
        }
    }

    /** An isolated ball: down from the setpoint and all the way back to it. */
    private static void fireOneBall(FlywheelDroopCompensator c, double depthRps) {
        fireOneBall(c, SETPOINT, depthRps);
    }

    private static void fireOneBall(
            FlywheelDroopCompensator c, double setpoint, double depthRps) {
        oneBall(c, setpoint, 0.0, -depthRps, 0.0);
        for (int i = 0; i < 10; i++) {
            step(c, setpoint, setpoint);
        }
    }

    // ---- Physics seed ----

    @Test
    void physicsSeedIsTheClosedFormFromInertiaAndBallMass() {
        double expected = ShooterConstants.FUEL_MASS_KG
            * ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS
            * ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS
            / (8.0 * ShooterConstants.FLYWHEEL_ROTOR_MOI_KG_M2
                * ShooterConstants.FLYWHEEL_SHOT_ENERGY_EFFICIENCY);

        assertEquals(expected, FlywheelDroopCompensator.physicsSeedFraction(), 1e-12,
            "seed should be m*d^2 / (8*I_rotor*efficiency)");
    }

    @Test
    void physicsSeedDoesNotDependOnWhichSideOfTheGearboxItIsMeasuredFrom() {
        // Droop as a fraction is a property of the machine, not of the units it is written in —
        // the gear ratio has to cancel.
        double dAtFlywheel = (ShooterConstants.FLYWHEEL_LARGE_DIAMETER_METERS
            + ShooterConstants.FLYWHEEL_SMALL_DIAMETER_METERS
                * ShooterConstants.FLYWHEEL_SMALL_PER_LARGE_RATIO) / 2.0;
        double atFlywheel = ShooterConstants.FUEL_MASS_KG * dAtFlywheel * dAtFlywheel
            / (8.0 * ShooterConstants.FLYWHEEL_MOI_KG_M2
                * ShooterConstants.FLYWHEEL_SHOT_ENERGY_EFFICIENCY);

        assertEquals(atFlywheel, FlywheelDroopCompensator.physicsSeedFraction(), 1e-12,
            "the gear ratio must cancel out of the droop fraction");
    }

    @Test
    void gearingIsActuallyWiredIntoTheSpeedConversion() {
        // The bug this guards: FLYWHEEL_GEAR_RATIO used to reach nothing but the simulation, so
        // changing it had no effect on a single real shot.
        double ungeared = (ShooterConstants.FLYWHEEL_LARGE_DIAMETER_METERS
            + ShooterConstants.FLYWHEEL_SMALL_DIAMETER_METERS) / 2.0;

        assertTrue(ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS < ungeared,
            "a reduction must show up as less ball speed per motor rotation");
        assertEquals(
            (ShooterConstants.FLYWHEEL_LARGE_DIAMETER_METERS
                + ShooterConstants.FLYWHEEL_SMALL_DIAMETER_METERS
                    * ShooterConstants.FLYWHEEL_SMALL_PER_LARGE_RATIO)
                / 2.0 / ShooterConstants.FLYWHEEL_GEAR_RATIO,
            ShooterConstants.FLYWHEEL_EFFECTIVE_DIAMETER_METERS, 1e-12,
            "effective diameter should carry both the motor reduction and the wheel-to-wheel ratio");
    }

    // ---- Counting balls ----

    @Test
    void spinUpIsNotMistakenForAShot() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();

        // A wheel climbing from parked trails its setpoint by tens of RPS — a far bigger excursion
        // than any ball — and must not be learned from.
        for (double measured = 0.0; measured < SETPOINT; measured += 0.5) {
            step(c, SETPOINT, measured);
        }
        settleAtSpeed(c);

        assertEquals(0, c.getShotCount(), "spin-up must not count as a ball");
        assertEquals(0.0, c.compensationRps(SETPOINT), 1e-9,
            "and nothing should have been learned (the seed is inert in simulation)");
    }

    @Test
    void anIsolatedBallIsCountedAndLearned() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);
        assertEquals(0.0, c.compensationRps(SETPOINT), 1e-9, "no bias before anything is measured");

        fireOneBall(c, 4.0);

        assertEquals(1, c.getShotCount(), "the dip should have been counted as one ball");
        assertEquals(4.0, c.getLastTroughDeficitRps(), 0.5,
            "the learned quantity is how far below setpoint the wheel was at separation");
        assertEquals(4.0, c.getLastPerBallDropRps(), 0.5,
            "for an isolated ball the incremental drop and the absolute deficit agree");
        assertTrue(c.compensationRps(SETPOINT) > 0.0, "a measured sag should bias the command up");
    }

    /**
     * The case the previous baseline-crossing detector could not see at all: a burst where the wheel
     * climbs part of the way back between balls but never returns to where it started. That
     * detector waited for a return to baseline, never got one, timed out, and learned nothing from
     * the busiest part of the match.
     */
    @Test
    void aBurstIsCountedAsSeparateBallsNotOneLongSag() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // Five balls. Each takes 4 RPS out; the motor puts only 2 back before the next arrives, so
        // the absolute trough walks steadily down and never comes near the starting speed.
        double level = 0.0;
        for (int n = 0; n < 5; n++) {
            double trough = level - 4.0;
            double rebound = trough + 2.0;
            oneBall(c, level, trough, rebound);
            level = rebound;
        }

        assertEquals(5, c.getShotCount(), "every ball in the burst should be counted on its own");
        assertEquals(4.0, c.getLastPerBallDropRps(), 0.5,
            "each ball's own contribution stays about the same through the burst");
        assertEquals(12.0, c.getLastTroughDeficitRps(), 0.5,
            "while the absolute deficit accumulates — and that is what sets the ball's speed");
    }

    @Test
    void aSteadySagWithNoReboundIsNotABall() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // The wheel simply loses ground and keeps losing it — a dying battery, not a ball. With no
        // reversal there is no trough to confirm, so nothing is counted.
        for (int i = 1; i <= 200; i++) {
            step(c, SETPOINT, SETPOINT - i * 0.05);
        }

        assertEquals(0, c.getShotCount(), "a monotonic sag has no trough and so is not a ball");
    }

    @Test
    void noiseTooShallowToBeABallIsIgnored() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // Ripple well under FLYWHEEL_SHOT_DETECT_DROP_RPS. Nothing here is a ball.
        for (int i = 0; i < 200; i++) {
            step(c, SETPOINT, SETPOINT + 0.4 * Math.sin(i * 0.7));
        }

        assertEquals(0, c.getShotCount(), "velocity ripple must not register as balls");
    }

    @Test
    void aSetpointStepIsNotMistakenForABall() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // The aim solution jumping the setpoint up leaves the wheel trailing by exactly the
        // signature a ball produces. The slew guard is what tells them apart.
        double raised = SETPOINT + 8.0;
        for (int i = 0; i <= 60; i++) {
            step(c, raised, SETPOINT + 8.0 * i / 60.0);
        }

        assertEquals(0, c.getShotCount(), "a moving setpoint must not be learned from");
    }

    @Test
    void aStallIsTooDeepToBeABall() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // Something jammed the wheel: past FLYWHEEL_MAX_PLAUSIBLE_DROOP_RPS, and nothing that deep
        // should be allowed to poison the estimate.
        fireOneBall(c, ShooterConstants.FLYWHEEL_MAX_PLAUSIBLE_DROOP_RPS + 5.0);

        assertEquals(0, c.getShotCount(), "an implausibly deep sag should be discarded");
        assertEquals(0.0, c.compensationRps(SETPOINT), 1e-9, "and must not move the estimate");
    }

    // ---- What gets learned ----

    @Test
    void theLearnedDeficitTracksRepeatedBalls() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        for (int i = 0; i < 25; i++) {
            fireOneBall(c, 4.0);
        }

        assertEquals(25, c.getShotCount(), "every ball should have been counted");
        // Default gain is 1.0 — the ball leaves at the wheel's speed at separation, so the full
        // deficit is what has to be added back.
        assertEquals(4.0, c.compensationRps(SETPOINT), 0.5,
            "bias should converge on the measured trough depth");
    }

    @Test
    void deficitIsLearnedPerSpeedNotAsOneNumber() {
        // The motor's ability to push back during contact collapses near free speed, so the real
        // droop is badly non-linear — a single learned figure is wrong at one end or the other.
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();

        // A shallow dip at a low commanded speed.
        settleAtSpeed(c, 30.0);
        for (int n = 0; n < 10; n++) {
            fireOneBall(c, 30.0, 2.0);
        }

        // A much deeper one up near the ceiling.
        settleAtSpeed(c, 75.0);
        for (int n = 0; n < 10; n++) {
            fireOneBall(c, 75.0, 8.0);
        }

        assertTrue(c.compensationRps(75.0) > c.compensationRps(30.0) + 2.0,
            "the fast end should have learned a much bigger deficit than the slow end");
        assertEquals(2.0, c.compensationRps(30.0), 1.0, "slow end keeps its own shallow number");
        assertEquals(8.0, c.compensationRps(75.0), 1.0, "fast end keeps its own deep number");
    }

    @Test
    void theLearnedCurveCarriesItsSlopePastTheMeasuredRange() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();

        // Two speeds measured, the faster one sagging much harder — the real shape.
        settleAtSpeed(c, 40.0);
        for (int n = 0; n < 8; n++) {
            fireOneBall(c, 40.0, 2.0);
        }
        settleAtSpeed(c, 60.0);
        for (int n = 0; n < 8; n++) {
            fireOneBall(c, 60.0, 6.0);
        }

        // Asking beyond anything measured must keep climbing rather than flattening off at the last
        // value. The curve bends upward, so holding the top value would badly under-serve exactly
        // the long shots that need the most help.
        assertTrue(c.learnedDeficitRps(75.0) > c.learnedDeficitRps(60.0) + 1.0,
            "extrapolation should continue the measured slope, not flatten");

        // But a straight line off the end of an upward-bending curve still lands under the truth,
        // and the hard clamp is the backstop either way.
        assertTrue(c.compensationRps(75.0) <= ShooterConstants.FLYWHEEL_MAX_COMPENSATION_RPS + 1e-9,
            "and can never exceed the hard ceiling");
    }

    @Test
    void aSingleMeasuredSpeedScalesRatherThanGuessingACurve() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);
        for (int i = 0; i < 8; i++) {
            fireOneBall(c, 4.0);
        }

        // One bin means no slope to measure, so all it can honestly do is scale with speed.
        assertTrue(c.learnedDeficitRps(60.0) > c.learnedDeficitRps(40.0),
            "a single measurement should still grow with commanded speed");
    }

    @Test
    void briefSaturationRecoveringFromEachBallStillLearns() {
        // The bug this guards: the motor floors itself for a few milliseconds putting back what
        // every ball took, and at high commanded speeds that happens on EVERY shot. Treating that
        // as "out of headroom" would freeze learning exactly where the droop is worst.
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        for (int n = 0; n < 10; n++) {
            // Cruising with headroom to spare...
            for (int i = 1; i <= 4; i++) {
                step(c, SETPOINT, SETPOINT - 6.0 * i / 4.0, 0.8);
            }
            // ...then pinned briefly while it claws the speed back.
            for (int i = 1; i <= 10; i++) {
                step(c, SETPOINT, SETPOINT - 6.0 + 6.0 * i / 10.0, 1.0);
            }
            for (int i = 0; i < 10; i++) {
                step(c, SETPOINT, SETPOINT, 0.8);
            }
        }

        assertEquals(10, c.getShotCount(), "every ball should be counted");
        assertTrue(c.compensationRps(SETPOINT) > 0.0,
            "and a momentary pin during recovery must not stop the robot learning from them");
    }

    @Test
    void learningIsFrozenWhenTheMotorIsFlatOutContinuously() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // Pinned flat out with the wheel stuck below its setpoint and no balls in sight — it simply
        // cannot reach the speed it is already being asked for. Held long enough for the duty
        // filter to recognise that as sustained rather than a recovery transient.
        for (int i = 0; i < 600; i++) {
            step(c, SETPOINT, SETPOINT - 3.0, 1.0);
        }

        // Now feed it. More setpoint would produce no more speed, so none of this should be learned.
        for (int n = 0; n < 10; n++) {
            for (int i = 1; i <= 4; i++) {
                step(c, SETPOINT, SETPOINT - 3.0 - 6.0 * i / 4.0, 1.0);
            }
            for (int i = 1; i <= 10; i++) {
                step(c, SETPOINT, SETPOINT - 9.0 + 6.0 * i / 10.0, 1.0);
            }
        }

        assertTrue(c.getShotCount() > 0, "the balls still count");
        assertEquals(0.0, c.compensationRps(SETPOINT), 1e-9,
            "but a permanently maxed-out motor must not teach the robot to ask for more");
    }

    // ---- Safety ----

    @Test
    void biasIsClampedAndNeverNegative() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // Sag right up against the plausibility ceiling, over and over — the worst case the learner
        // will accept.
        for (int i = 0; i < 25; i++) {
            fireOneBall(c, ShooterConstants.FLYWHEEL_MAX_PLAUSIBLE_DROOP_RPS - 1.0);
        }

        assertTrue(c.compensationRps(SETPOINT)
                <= ShooterConstants.FLYWHEEL_MAX_COMPENSATION_RPS + 1e-9,
            "bias must never exceed FLYWHEEL_MAX_COMPENSATION_RPS");

        // A wheel running FAST of its setpoint is not a reason to command less.
        FlywheelDroopCompensator fast = new FlywheelDroopCompensator();
        settleAtSpeed(fast);
        for (int n = 0; n < 10; n++) {
            oneBall(fast, 6.0, 2.0, 6.0);
        }
        assertTrue(fast.compensationRps(SETPOINT) >= 0.0, "bias must never go negative");
    }

    @Test
    void aParkedFlywheelIsNeverCompensated() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);
        for (int i = 0; i < 10; i++) {
            fireOneBall(c, 4.0);
        }

        assertEquals(0.0, c.compensationRps(0.0), 1e-9,
            "a stopped wheel must stay stopped — storage mode depends on it");
        assertEquals(0.0, c.compensationRps(ShooterConstants.FLYWHEEL_MIN_DETECT_RPS - 1.0), 1e-9,
            "and nothing below the detection floor should be biased either");
    }

    @Test
    void resetReturnsToTheColdStartEstimate() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);
        for (int i = 0; i < 10; i++) {
            fireOneBall(c, 4.0);
        }
        assertTrue(c.compensationRps(SETPOINT) > 0.0);

        c.reset();

        assertEquals(0, c.getShotCount(), "reset should forget every ball");
        assertEquals(0, c.getPopulatedBinCount(), "and every learned bin");
    }

    @Test
    void contactTimeIsMeasuredFromTheDescent() {
        FlywheelDroopCompensator c = new FlywheelDroopCompensator();
        settleAtSpeed(c);

        // oneBall() descends over 4 samples at 5 ms each — the real 10-25 ms contact timescale.
        fireOneBall(c, 4.0);

        assertEquals(4 * LOOP, c.getLastContactSeconds(), LOOP,
            "contact time should be the descent, not the much longer recovery");
    }
}
