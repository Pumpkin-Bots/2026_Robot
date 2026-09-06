package frc.robot.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An on/off switch the drive team can flip live from a dashboard (Elastic / Glass / Shuffleboard),
 * published under {@code /SmartDashboard/<key>} alongside everything else this project puts there.
 *
 * <p>Unlike {@link TunableDouble}, this is <b>not</b> gated behind {@link TuningMode} and is read
 * straight from the dashboard at all times. The distinction is deliberate: a tunable is a number
 * being characterised, and one left wrong on a laptop must never follow the robot into a match,
 * whereas a toggle is an operator decision that has to be available <i>during</i> a match — a
 * behaviour that is misfiring needs turning off between one cycle and the next, not a code push.
 *
 * <p>The flip side is that nothing snaps these back on their own, so anything switched off during a
 * match stays off until someone switches it back or the roboRIO reboots. Give every toggle a
 * default that is the behaviour you want when nobody has touched anything.
 *
 * <p>In Elastic: find the key under the {@code SmartDashboard} tree, drag it out, and set the widget
 * type to <b>Toggle Switch</b> or <b>Toggle Button</b>. The value is published as a writable entry,
 * so clicks flow straight back to the robot.
 */
public class DashboardToggle {
    private final BooleanEntry m_entry;
    private final boolean m_default;

    public DashboardToggle(String key, boolean defaultValue) {
        m_default = defaultValue;
        m_entry = NetworkTableInstance.getDefault()
            .getBooleanTopic("/SmartDashboard/" + key)
            .getEntry(defaultValue);
        m_entry.setDefault(defaultValue);
    }

    /** The current dashboard value, or the compiled-in default if nothing has been published. */
    public boolean get() {
        return m_entry.get(m_default);
    }

    public void set(boolean value) {
        m_entry.set(value);
    }

    /** The compiled-in default, ignoring whatever the dashboard currently says. */
    public boolean defaultValue() {
        return m_default;
    }
}
