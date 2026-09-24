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
        holdVoltage(12.5, 2.0);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(), "should start healthy");
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
        for (double volts : new double[] {12.5, 9.8, 9.0, 8.2, 7.5}) {
            holdVoltage(volts, 1.5);
            assertTrue(PowerBudget.intakeOutputScale() <= PowerBudget.driveOutputScale() + 1e-9,
                "intake should never be cut less than the drive, at " + volts + " V");
        }
    }

    @Test
    void sagEntersReducedThenCritical() {
        holdVoltage(9.0, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());
        assertTrue(PowerBudget.intakeOutputScale() < 1.0, "intake should be cut first");
        assertTrue(PowerBudget.driveOutputScale() < 1.0, "drive should be cut too");

        holdVoltage(8.0, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());
        assertEquals(0.0, PowerBudget.intakeOutputScale(), 1e-9,
            "a critical battery should stop the intake outright");
        assertTrue(PowerBudget.driveOutputScale() > 0.0, "but the robot should still be drivable");
    }

    @Test
    void recoveryNeedsToClearTheEntryThreshold() {
        holdVoltage(9.0, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());

        // Above the entry threshold but below the exit one: that gap is the hysteresis, and sitting
        // in it must not restore full authority — cutting the intake is itself what raised the
        // voltage, so this is exactly the reading a cut robot produces.
        holdVoltage(
            (PowerConstants.REDUCED_ENTER_VOLTS + PowerConstants.REDUCED_EXIT_VOLTS) / 2.0, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "should not recover inside the hysteresis band");

        holdVoltage(11.0, 1.5);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(), "a real recovery should restore");
    }

    @Test
    void aTierIsHeldLongEnoughNotToFlicker() {
        // Step only as far as the moment the tier flips, so the dwell clock starts here rather
        // than having already run out during a long hold.
        RoboRioSim.setVInVoltage(9.0);
        for (int i = 0; i < 200 && PowerBudget.tier() == PowerBudget.Tier.NORMAL; i++) {
            SimHooks.stepTiming(LOOP);
            PowerBudget.update();
        }
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier());

        // Cutting the intake is what raises the voltage, so the battery snaps back the instant the
        // protection engages. Without the dwell that reading restores the load, which sags the
        // battery, which cuts it again — and the loop runs at 50 Hz.
        holdVoltage(12.5, PowerConstants.MIN_TIER_HOLD_SECONDS - 0.1);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "the tier should be held for its minimum dwell even once the voltage recovers");

        holdVoltage(12.5, 0.5);
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier(),
            "and should restore once the dwell is up");
    }

    @Test
    void criticalStepsBackOutOneTierAtATime() {
        holdVoltage(8.0, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());

        holdVoltage(9.6, 1.5);
        assertEquals(PowerBudget.Tier.REDUCED, PowerBudget.tier(),
            "leaving critical should land in reduced, not straight back to full authority");
    }

    @Test
    void disablingTheRobotClearsAnyLatchedTier() {
        holdVoltage(8.0, 1.5);
        assertEquals(PowerBudget.Tier.CRITICAL, PowerBudget.tier());

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        holdVoltage(8.0, 0.1);

        // A disabled robot draws nothing, so its voltage says nothing about what the next enable
        // can afford — and a tier latched in the last match must not follow the robot into the next.
        assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier());
    }

    @Test
    void theOperatorSwitchPinsEverythingAtFullAuthority() {
        PowerBudget.ENABLED.set(false);
        try {
            holdVoltage(7.5, 1.5);
            assertEquals(PowerBudget.Tier.NORMAL, PowerBudget.tier());
            assertEquals(1.0, PowerBudget.driveOutputScale(), 1e-9);
            assertEquals(1.0, PowerBudget.intakeOutputScale(), 1e-9);
        } finally {
            PowerBudget.ENABLED.set(true);
        }
    }
}
