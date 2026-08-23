package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Master switch deciding whether {@link TunableDouble}s read their live dashboard value or their
 * compiled-in constant.
 *
 * <p>Two toggles show up in Elastic under {@code SmartDashboard/Tuning}:
 *
 * <ul>
 *   <li><b>TuningModeEnabled</b> — off (the default) means every tunable returns the constant from
 *       {@link frc.robot.constants.Constants} and whatever is typed on the dashboard is ignored
 *       entirely. On means the dashboard values take over.
 *   <li><b>ResetToDefaults</b> — momentary. Rewrites every tunable's dashboard value back to its
 *       compiled constant, for when a tuning session has wandered somewhere bad and the original
 *       numbers are gone. Clears itself back to false.
 * </ul>
 *
 * <p>Defaulting to <i>off</i> is deliberate: a value someone fat-fingered between matches should
 * never be able to silently change how the robot shoots. Flipping the switch on is one click, and
 * the switch sitting visibly on the dashboard is the reminder that the robot is not running the
 * numbers that are in source control.
 */
public final class TuningMode {

    private static final List<TunableDouble> s_registry = new ArrayList<>();

    private static final BooleanEntry s_enabled;
    private static final BooleanEntry s_resetRequested;

    static {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        s_enabled = nt.getBooleanTopic("/SmartDashboard/Tuning/TuningModeEnabled").getEntry(false);
        s_enabled.setDefault(false);
        s_resetRequested = nt.getBooleanTopic("/SmartDashboard/Tuning/ResetToDefaults").getEntry(false);
        s_resetRequested.setDefault(false);
    }

    private TuningMode() {}

    /** True when tunables should follow the dashboard instead of their compiled constants. */
    public static boolean isEnabled() {
        return s_enabled.get(false);
    }

    static void register(TunableDouble tunable) {
        s_registry.add(tunable);
    }

    /** Services the reset button. Call once per loop from {@code Robot.robotPeriodic()}. */
    public static void periodic() {
        if (s_resetRequested.get(false)) {
            s_resetRequested.set(false);
            for (TunableDouble tunable : s_registry) {
                tunable.publishDefault();
            }
        }
    }
}
