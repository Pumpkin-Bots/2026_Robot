package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants.FieldConstants;
import frc.robot.constants.Constants.ShooterConstants;

/**
 * Where on the field the shooter is, and what that means it should be doing.
 *
 * <p>Every query here takes a field-relative X/Y of the SHOOTER's launch point rather than the
 * robot's centre. The turret hangs off centre, so on a robot straddling a boundary the two can
 * disagree — and what matters for "can I take this shot" is where the ball actually leaves from.
 *
 * <p>All the geometry lives in {@link FieldConstants}; nothing is hard-coded here.
 */
public final class FieldZones {
    private FieldZones() {}

    /** True when the driver station reports red. Defaults to blue when the alliance is unknown. */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /**
     * True when the point is inside any of the four trench channels — either alliance's trench, on
     * either side of the field. Only the drivable channel under the arm counts; the solid inboard
     * part of the structure is not somewhere a robot can be.
     *
     * @param extraMarginMeters padding beyond {@code TRENCH_MARGIN_METERS}, used to add hysteresis
     *     once the robot is already treating itself as being in the trench
     */
    public static boolean isInTrench(double x, double y, double extraMarginMeters) {
        double margin = FieldConstants.TRENCH_MARGIN_METERS + extraMarginMeters;

        boolean atTrenchStation =
            Math.abs(x - FieldConstants.BLUE_TRENCH_CENTER_X_METERS)
                <= FieldConstants.TRENCH_DEPTH_METERS / 2.0 + margin
            || Math.abs(x - FieldConstants.RED_TRENCH_CENTER_X_METERS)
                <= FieldConstants.TRENCH_DEPTH_METERS / 2.0 + margin;

        double channel = FieldConstants.TRENCH_CHANNEL_DEPTH_METERS + margin;
        boolean againstGuardrail =
            y <= channel || y >= FieldConstants.FIELD_WIDTH_METERS - channel;

        return atTrenchStation && againstGuardrail;
    }

    /**
     * True when the point is under either alliance's tower overhang. The towers are built into the
     * alliance walls, so this is the strip of carpet right against each end of the field.
     *
     * @param extraMarginMeters padding beyond {@code TOWER_MARGIN_METERS}, for hysteresis
     */
    public static boolean isUnderTower(double x, double y, double extraMarginMeters) {
        double margin = FieldConstants.TOWER_MARGIN_METERS + extraMarginMeters;
        double halfWidth = FieldConstants.TOWER_WIDTH_METERS / 2.0 + margin;
        double depth = FieldConstants.TOWER_DEPTH_METERS + margin;

        boolean blue = x <= depth
            && Math.abs(y - FieldConstants.BLUE_TOWER_CENTER_Y_METERS) <= halfWidth;
        boolean red = x >= FieldConstants.FIELD_LENGTH_METERS - depth
            && Math.abs(y - FieldConstants.RED_TOWER_CENTER_Y_METERS) <= halfWidth;

        return blue || red;
    }

    /**
     * True anywhere the shooter physically cannot take a shot — under a trench arm or under a
     * tower. This is what flips the shooter into storage behaviour automatically.
     *
     * @param extraMarginMeters padding applied to both zones, for hysteresis
     */
    public static boolean isInNoShootZone(double x, double y, double extraMarginMeters) {
        return isInTrench(x, y, extraMarginMeters) || isUnderTower(x, y, extraMarginMeters);
    }

    /**
     * True when the point is inside the given alliance's OWN zone — i.e. close enough to its own
     * hub to be shooting at it rather than shuttling toward it.
     *
     * @param wasInside the previous answer, so the boundary can be given
     *     {@code ZONE_HYSTERESIS_METERS} of stickiness instead of chattering when the robot parks
     *     on the line. Pass {@code false} for a fresh, unbiased evaluation.
     */
    public static boolean isInOwnAllianceZone(double x, boolean isRed, boolean wasInside) {
        // Push the line away from the current answer, so leaving costs more than staying.
        double hysteresis = wasInside
            ? FieldConstants.ZONE_HYSTERESIS_METERS
            : -FieldConstants.ZONE_HYSTERESIS_METERS;

        return isRed
            ? x >= FieldConstants.RED_ALLIANCE_ZONE_MIN_X_METERS - hysteresis
            : x <= FieldConstants.BLUE_ALLIANCE_ZONE_MAX_X_METERS + hysteresis;
    }

    /** The alliance's own hub, as a 3D aim point. */
    public static Translation3d hubTarget(boolean isRed) {
        return isRed
            ? ShooterConstants.RED_TARGET_POSITION
            : ShooterConstants.BLUE_TARGET_POSITION;
    }

    /**
     * The alliance's shuttle aim point: a patch of carpet next to its own hub, offset to whichever
     * side of the field the shooter is already on.
     *
     * @param shooterY field-relative Y of the shooter's launch point
     */
    public static Translation3d shuttleTarget(boolean isRed, double shooterY) {
        double tagY    = isRed ? ShooterConstants.RED_SHUTTLE_TARGET_Y_METERS
                               : ShooterConstants.BLUE_SHUTTLE_TARGET_Y_METERS;
        double targetX = isRed ? ShooterConstants.RED_SHUTTLE_TARGET_X_METERS
                               : ShooterConstants.BLUE_SHUTTLE_TARGET_X_METERS;
        double targetZ = isRed ? ShooterConstants.RED_SHUTTLE_TARGET_Z_METERS
                               : ShooterConstants.BLUE_SHUTTLE_TARGET_Z_METERS;

        double offset = ShooterConstants.SHUTTLE_SIDE_OFFSET_METERS;
        double targetY = (shooterY > tagY) ? tagY + offset : tagY - offset;

        return new Translation3d(targetX, targetY, targetZ);
    }
}
