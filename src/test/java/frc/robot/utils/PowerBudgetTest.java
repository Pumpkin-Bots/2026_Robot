package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.constants.Constants.PowerConstants;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Pins down the priority the power budget exists to enforce: whatever else gets cut, the intake is
 * cut at least as hard as the drivetrain, and a tier never flickers.
 */
class PowerBudgetTest {

    private static final double LOOP = 0.020;

    // Every voltage below is DERIVED from the thresholds rather than written in.
    //
    // These tests are about the tier machine's behaviour — the cut ordering, the hysteresis, the
    // dwell — none of which is a claim about any particular voltage. Hard-coded readings turn a
    // threshold retune into five unrelated-looking failures, which is what happened when the bands
    // were lowered for a battery that sags further than the first guess allowed for.

    /** Healthy battery, well clear of every threshold. */
    private static final double HEALTHY_VOLTS = 12.5;

    /** Inside the REDUCED band: past its entry, not yet at critical's. */
    private static final double REDUCED_VOLTS =
        midpoint(PowerConstants.CRITICAL_ENTER_VOLTS, PowerConstants.REDUCED_ENTER_VOLTS);

    /** Clearly into CRITICAL. */
    private static final double CRITICAL_VOLTS = PowerConstants.CRITICAL_ENTER_VOLTS - 0.5;

    /**
     * Recovered past critical's exit but not past reduced's — the one reading that has to land in
     * REDUCED on the way back up rather than jumping to full authority.
     */
    private static final double CRITICAL_RECOVERY_VOLTS =
        midpoint(PowerConstants.CRITICAL_EXIT_VOLTS, PowerConstants.REDUCED_EXIT_VOLTS);

    /** Inside reduced's hysteresis band: past the entry threshold, short of the exit one. */
    private static final double REDUCED_HYSTERESIS_VOLTS =
        midpoint(PowerConstants.REDUCED_ENTER_VOLTS, PowerConstants.REDUCED_EXIT_VOLTS);

    private static double midpoint(double a, double b) {
        return (a + b) / 2.0;
    }

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0), "HAL should initialize for the FPGA clock");
        SimHooks.pauseTiming();
    }

    @AfterAll
    static void shutdownHal() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
        HAL.shutdown();
    }

    /** Enabled, healthy battery, tier settled at NORMAL — the state every test starts from. */
    @BeforeEach
    void startHealthyAndEnabled() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        holdVoltage(HEALTHY_VOLTS, 2.0);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(), "should start healthy");
    }

    /**
     * The thresholds have to be ordered for any of the rest of this to mean anything: each tier's
     * exit above its own entry (that gap <i>is</i> the hysteresis), and critical below reduced.
     * Retuning them by feel on a practice field is easy, and getting one the wrong side of another
     * turns the tier machine into a latch nobody asked for.
     */
    @Test
    void thresholdsAreOrderedSoTheTiersNest() {
        assertTrue(PowerConstants.REDUCED_EXIT_VOLTS > PowerConstants.REDUCED_ENTER_VOLTS,
            "reduced needs hysteresis: exit must be above entry");
        assertTrue(PowerConstants.CRITICAL_EXIT_VOLTS > PowerConstants.CRITICAL_ENTER_VOLTS,
            "critical needs hysteresis: exit must be above entry");
        assertTrue(PowerConstants.CRITICAL_ENTER_VOLTS < PowerConstants.REDUCED_ENTER_VOLTS,
            "critical must be entered at a lower voltage than reduced");
        assertTrue(PowerConstants.CRITICAL_EXIT_VOLTS <= PowerConstants.REDUCED_EXIT_VOLTS,
            "leaving critical must not also leave reduced");
    }

    /** Holds the battery at a voltage for a span of seconds, stepping the loop the whole way. */
    private static void holdVoltage(double volts, double seconds) {
        RoboRioSim.setVInVoltage(volts);
        for (int i = 0; i < (int) Math.round(seconds / LOOP); i++) {
            SimHooks.stepTiming(LOOP);
            PowerBudget.update();
        }
    }

    @Test
    void healthyBatteryChangesNothing() {
        assertEquals(1.0, PowerBudget.driveOutputScale(), 1e-9);
        assertEquals(1.0, PowerBudget.intakeOutputScale(), 1e-9);
        assertTrue(!PowerBudget.isLimiting());
    }

    @Test
    void intakeIsAlwaysCutAtLeastAsHardAsTheDrivetrain() {
        // The whole point of the ordering: fuel collected slowly costs cycle time, a robot that
        // cannot move costs the match, and neither costs as much as a slow shot.
        double[] sweep = {
            HEALTHY_VOLTS,
            PowerConstants.REDUCED_ENTER_VOLTS + 0.5,
            REDUCED_VOLTS,
            PowerConstants.CRITICAL_ENTER_VOLTS + 0.1,
            CRITICAL_VOLTS,
        };
        for (double volts : sweep) {
            holdVoltage(volts, 1.5);
            assertTrue(PowerBudget.intakeOutputScale() <= PowerBudget.driveOutputScale() + 1e-9,
                "intake should never be cut less than the drive, at " + volts + " V");
        }
    }

    @Test
    void sagEntersReducedThenCritical() {
        holdVoltage(REDUCED_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());
        assertTrue(PowerBudget.intakeOutputScale() < 1.0, "intake should be cut first");
        assertTrue(PowerBudget.driveOutputScale() < 1.0, "drive should be cut too");

        holdVoltage(CRITICAL_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());
        assertEquals(0.0, PowerBudget.intakeOutputScale(), 1e-9,
            "a critical battery should stop the intake outright");
        assertTrue(PowerBudget.driveOutputScale() > 0.0, "but the robot should still be drivable");
    }

    @Test
    void recoveryNeedsToClearTheEntryThreshold() {
        holdVoltage(REDUCED_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());

        // Above the entry threshold but below the exit one: that gap is the hysteresis, and sitting
        // in it must not restore full authority — cutting the intake is itself what raised the
        // voltage, so this is exactly the reading a cut robot produces.
        holdVoltage(REDUCED_HYSTERESIS_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "should not recover inside the hysteresis band");

        holdVoltage(HEALTHY_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(), "a real recovery should restore");
    }

    @Test
    void aTierIsHeldLongEnoughNotToFlicker() {
        // Step only as far as the moment the tier flips, so the dwell clock starts here rather
        // than having already run out during a long hold.
        RoboRioSim.setVInVoltage(REDUCED_VOLTS);
        for (int i = 0; i < 200 && PowerBudget.tier() == PowerBudget.Tier.NORMAL; i++) {
            SimHooks.stepTiming(LOOP);
            PowerBudget.update();
        }
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());

        // Cutting the intake is what raises the voltage, so the battery snaps back the instant the
        // protection engages. Without the dwell that reading restores the load, which sags the
        // battery, which cuts it again — and the loop runs at 50 Hz.
        holdVoltage(HEALTHY_VOLTS, PowerConstants.MIN_TIER_HOLD_SECONDS - 0.1);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "the tier should be held for its minimum dwell even once the voltage recovers");

        holdVoltage(HEALTHY_VOLTS, 0.5);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(),
            "and should restore once the dwell is up");
    }

    @Test
    void criticalStepsBackOutOneTierAtATime() {
        holdVoltage(CRITICAL_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());

        holdVoltage(CRITICAL_RECOVERY_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "leaving critical should land in reduced, not straight back to full authority");
    }

    @Test
    void disablingTheRobotClearsAnyLatchedTier() {
        holdVoltage(CRITICAL_VOLTS, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        holdVoltage(CRITICAL_VOLTS, 0.1);

        // A disabled robot draws nothing, so its voltage says nothing about what the next enable
        // can afford — and a tier latched in the last match must not follow the robot into the next.
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier());
    }

    @Test
    void theOperatorSwitchPinsEverythingAtFullAuthority() {
        PowerBudget.ENABLED.set(false);
        try {
            holdVoltage(CRITICAL_VOLTS, 1.5);
            assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier());
            assertEquals(1.0, PowerBudget.driveOutputScale(), 1e-9);
            assertEquals(1.0, PowerBudget.intakeOutputScale(), 1e-9);
        } finally {
            PowerBudget.ENABLED.set(true);
        }
    }
}
