package frc.robot.utils;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A number that can be edited live from a dashboard (Elastic / Glass / Shuffleboard) instead of
 * requiring an edit-recompile-redeploy cycle. Publishes under {@code /SmartDashboard/<key>} so it
 * shows up alongside everything else this project puts on SmartDashboard.
 *
 * <p>The dashboard value is only obeyed while {@link TuningMode} is enabled. With the switch off,
 * {@link #get()} returns the compiled-in constant and ignores the dashboard completely — so a
 * forgotten value left on a laptop cannot follow the robot into a match.
 *
 * <p>Whatever is typed on the dashboard lives only until the roboRIO reboots. Once a value is proven
 * on the real robot, copy it back into {@link frc.robot.constants.Constants} so it survives.
 *
 * <p>In Elastic: find the key under the {@code SmartDashboard} tree, drag it out, and set the widget
 * type to <b>Text Display</b> — that is the one that accepts typed input. The value is published as a
 * writable entry, so edits flow straight back to the robot.
 */
public class TunableDouble {
    private final DoubleEntry m_entry;
    private final double m_default;

    public TunableDouble(String key, double defaultValue) {
        m_default = defaultValue;
        m_entry = NetworkTableInstance.getDefault()
            .getDoubleTopic("/SmartDashboard/" + key)
            .getEntry(defaultValue);
        m_entry.setDefault(defaultValue);
        TuningMode.register(this);
    }

    /** The dashboard value while tuning mode is on, otherwise the compiled-in constant. */
    public double get() {
        return TuningMode.isEnabled() ? m_entry.get(m_default) : m_default;
    }

    public void set(double value) {
        m_entry.set(value);
    }

    /** The compiled-in value, ignoring any dashboard override. */
    public double defaultValue() {
        return m_default;
    }

    /** Overwrites the dashboard value with the compiled-in constant. */
    void publishDefault() {
        m_entry.set(m_default);
    }
}
