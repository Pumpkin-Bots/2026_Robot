package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Constants.FieldConstants;
import frc.robot.constants.Constants.ShooterConstants;
import org.junit.jupiter.api.Test;

/**
 * Pins down the field-position logic that decides whether the robot shoots, shuttles, or parks the
 * shooter. Every number here is derived from {@link FieldConstants} rather than written out, so
 * re-measuring a structure on the field moves the tests with the constants instead of breaking them.
 */
class FieldZonesTest {

    private static final boolean BLUE = false;
    private static final boolean RED  = true;

    private static final double MID_Y = FieldConstants.FIELD_WIDTH_METERS / 2.0;

    @Test
    void ownAllianceZoneIsTheNearThirdForEachAlliance() {
        // Well inside each alliance's own end.
        assertTrue(FieldZones.isInOwnAllianceZone(2.0, BLUE, false));
        assertTrue(FieldZones.isInOwnAllianceZone(FieldConstants.FIELD_LENGTH_METERS - 2.0, RED, false));

        // Midfield is nobody's alliance zone — that is the shuttling case.
        assertFalse(FieldZones.isInOwnAllianceZone(FieldConstants.FIELD_LENGTH_METERS / 2.0, BLUE, false));
        assertFalse(FieldZones.isInOwnAllianceZone(FieldConstants.FIELD_LENGTH_METERS / 2.0, RED, false));

        // Standing deep in the OPPONENT's zone is still a shuttle, not a shot.
        assertFalse(FieldZones.isInOwnAllianceZone(FieldConstants.FIELD_LENGTH_METERS - 2.0, BLUE, false));
        assertFalse(FieldZones.isInOwnAllianceZone(2.0, RED, false));
    }

    @Test
    void allianceZoneBoundaryIsStickyBothWays() {
        // A shooter parked exactly on the line would otherwise flip decision every loop, swinging
        // the turret between the hub and the shuttle aim point.
        double edge = FieldConstants.BLUE_ALLIANCE_ZONE_MAX_X_METERS;
        double inside = edge - FieldConstants.ZONE_HYSTERESIS_METERS / 2.0;
        double outside = edge + FieldConstants.ZONE_HYSTERESIS_METERS / 2.0;

        // Just inside the line, but already shuttling: stay shuttling.
        assertFalse(FieldZones.isInOwnAllianceZone(inside, BLUE, false));
        // Just outside the line, but already shooting: keep shooting.
        assertTrue(FieldZones.isInOwnAllianceZone(outside, BLUE, true));

        // Push far enough past and the decision does flip, in both directions.
        double clearlyInside = edge - 2.0 * FieldConstants.ZONE_HYSTERESIS_METERS;
        double clearlyOutside = edge + 2.0 * FieldConstants.ZONE_HYSTERESIS_METERS;
        assertTrue(FieldZones.isInOwnAllianceZone(clearlyInside, BLUE, false));
        assertFalse(FieldZones.isInOwnAllianceZone(clearlyOutside, BLUE, true));
    }

    @Test
    void trenchChannelsAreDetectedOnBothSidesOfBothAllianceStations() {
        double nearRail = 0.3;
        double farRail = FieldConstants.FIELD_WIDTH_METERS - 0.3;

        assertTrue(FieldZones.isInTrench(FieldConstants.BLUE_TRENCH_CENTER_X_METERS, nearRail, 0.0));
        assertTrue(FieldZones.isInTrench(FieldConstants.BLUE_TRENCH_CENTER_X_METERS, farRail, 0.0));
        assertTrue(FieldZones.isInTrench(FieldConstants.RED_TRENCH_CENTER_X_METERS, nearRail, 0.0));
        assertTrue(FieldZones.isInTrench(FieldConstants.RED_TRENCH_CENTER_X_METERS, farRail, 0.0));
    }

    @Test
    void openFieldIsNotATrench() {
        // Right field-length station as a trench, but out in the middle of the field where the hub
        // is — this is the gap you shoot through, not the channel you drive under.
        assertFalse(FieldZones.isInTrench(FieldConstants.BLUE_TRENCH_CENTER_X_METERS, MID_Y, 0.0));

        // Against the guardrail, but nowhere near a trench station.
        assertFalse(FieldZones.isInTrench(FieldConstants.FIELD_LENGTH_METERS / 2.0, 0.3, 0.0));

        // Ordinary open carpet.
        assertFalse(FieldZones.isInNoShootZone(2.5, 2.5, 0.0));
    }

    @Test
    void towerOverhangIsDetectedAtBothAllianceWalls() {
        assertTrue(FieldZones.isUnderTower(0.3, FieldConstants.BLUE_TOWER_CENTER_Y_METERS, 0.0));
        assertTrue(FieldZones.isUnderTower(
            FieldConstants.FIELD_LENGTH_METERS - 0.3, FieldConstants.RED_TOWER_CENTER_Y_METERS, 0.0));

        // Same distance off the wall, but along the wall past the tower's width.
        assertFalse(FieldZones.isUnderTower(0.3, 1.0, 0.0));
        // On the tower's centreline, but out in the field beyond its depth.
        assertFalse(FieldZones.isUnderTower(4.0, FieldConstants.BLUE_TOWER_CENTER_Y_METERS, 0.0));
    }

    @Test
    void extraMarginOnlyEverGrowsTheNoShootZone() {
        // The hysteresis argument must never shrink a zone — a robot already in storage should not
        // be able to fall out of it while standing still.
        double justOutsideX = FieldConstants.BLUE_TRENCH_CENTER_X_METERS
            + FieldConstants.TRENCH_DEPTH_METERS / 2.0
            + FieldConstants.TRENCH_MARGIN_METERS
            + FieldConstants.ZONE_HYSTERESIS_METERS / 2.0;

        assertFalse(FieldZones.isInNoShootZone(justOutsideX, 0.3, 0.0));
        assertTrue(FieldZones.isInNoShootZone(
            justOutsideX, 0.3, FieldConstants.ZONE_HYSTERESIS_METERS));
    }

    @Test
    void shuttlePassIsAimedToTheSideTheShooterIsAlreadyOn() {
        double hubY = ShooterConstants.BLUE_SHUTTLE_TARGET_Y_METERS;
        double offset = ShooterConstants.SHUTTLE_SIDE_OFFSET_METERS;

        Translation3d fromHighSide = FieldZones.shuttleTarget(BLUE, hubY + 1.0);
        Translation3d fromLowSide = FieldZones.shuttleTarget(BLUE, hubY - 1.0);

        assertEquals(hubY + offset, fromHighSide.getY(), 1e-9);
        assertEquals(hubY - offset, fromLowSide.getY(), 1e-9);

        // The pass is thrown to the carpet, not into the hub.
        assertEquals(ShooterConstants.SHUTTLE_TARGET_Z_METERS, fromHighSide.getZ(), 1e-9);
    }

    @Test
    void eachAllianceShootsAtItsOwnHub() {
        assertEquals(ShooterConstants.BLUE_TARGET_POSITION, FieldZones.hubTarget(BLUE));
        assertEquals(ShooterConstants.RED_TARGET_POSITION, FieldZones.hubTarget(RED));

        // Sanity check that the two hubs really are at opposite ends, so a sign error in the
        // alliance-zone test above would not go unnoticed.
        assertTrue(FieldZones.hubTarget(BLUE).getX() < FieldConstants.FIELD_LENGTH_METERS / 2.0);
        assertTrue(FieldZones.hubTarget(RED).getX() > FieldConstants.FIELD_LENGTH_METERS / 2.0);
    }
}
