package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Pins down the contract that keeps a forgotten dashboard value from following the robot into a
 * match: with the switch off, the compiled constant wins no matter what the dashboard says.
 */
class TuningModeTest {

    private static BooleanEntry tuningSwitch() {
        return NetworkTableInstance.getDefault()
            .getBooleanTopic("/SmartDashboard/Tuning/TuningModeEnabled").getEntry(false);
    }

    private static DoubleEntry dashboardEntry(String key) {
        return NetworkTableInstance.getDefault()
            .getDoubleTopic("/SmartDashboard/" + key).getEntry(0.0);
    }

    @AfterEach
    void disableTuning() {
        tuningSwitch().set(false);
    }

    @Test
    void dashboardValueIsIgnoredWhileTuningModeIsOff() {
        TunableDouble tunable = new TunableDouble("Tuning/Test/IgnoredWhenOff", 5.0);
        dashboardEntry("Tuning/Test/IgnoredWhenOff").set(9.0);

        tuningSwitch().set(false);
        assertEquals(5.0, tunable.get(), 1e-9, "constant should win while tuning mode is off");

        tuningSwitch().set(true);
        assertEquals(9.0, tunable.get(), 1e-9, "dashboard should win while tuning mode is on");

        tuningSwitch().set(false);
        assertEquals(5.0, tunable.get(), 1e-9, "toggling off should fall back to the constant");
    }

    @Test
    void resetRepublishesCompiledDefaults() {
        TunableDouble tunable = new TunableDouble("Tuning/Test/ResetMe", 2.5);
        DoubleEntry entry = dashboardEntry("Tuning/Test/ResetMe");
        entry.set(99.0);
        tuningSwitch().set(true);
        assertEquals(99.0, tunable.get(), 1e-9);

        NetworkTableInstance.getDefault()
            .getBooleanTopic("/SmartDashboard/Tuning/ResetToDefaults").getEntry(false).set(true);
        TuningMode.periodic();

        assertEquals(2.5, tunable.get(), 1e-9, "reset should restore the compiled default");
    }
}
