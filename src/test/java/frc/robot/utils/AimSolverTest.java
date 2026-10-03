package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.AimSolver.AimSolution;
import frc.robot.utils.AimSolver.AimTuning;
import frc.robot.utils.AimSolver.ArcPolicy;
import org.junit.jupiter.api.Test;

/**
 * Verifies the solver by flying the ball.
 *
 * <p>Rather than pinning down the numbers the solver happens to produce today, each test takes the
 * solution, reconstructs the launch velocity from it, integrates the trajectory forward under
 * gravity, and asserts the ball lands on the target. That catches a real sign error or a broken
 * correction, and stays valid when the calibration constants get retuned on the real robot.
 */
class AimSolverTest {

    private static final double GRAVITY = 9.80665;
    private static final double FLYWHEEL_DIAMETER = 0.0762;
    private static final double TOLERANCE_METERS = 0.02;

    /**
     * Baseline calibration: no empirical fudge factors, so the no-drag solve is exact and the ball
     * is expected to land precisely on target.
     */
    private static AimTuning tuning() {
        return new AimTuning(
            GRAVITY,
            12.0,   // descentMarginDeg
            15.0,   // rackMinAngleDeg
            42.0,   // rackMaxAngleDeg
            0.0,    // rackOffsetDeg
            0.0,    // turretOffsetDeg
            FLYWHEEL_DIAMETER,
            70.0,   // maxFlywheelRps
            1.0,    // speedScalar
            0.0,    // speedPerMeterMps
            0.0,    // flywheelRpsOffset
            1.0,    // shootOnTheMoveGain
            0.0,    // feederForwardPushMps
            0.0,    // feederBackwardPushAt90Mps
            true,   // motionRackFirst
            20.0);  // motionRackMaxSwingDeg
    }

    private static AimTuning withFeeder(double forward, double backwardAt90) {
        AimTuning t = tuning();
        return new AimTuning(
            t.gravityMps2(), t.descentMarginDeg(), t.rackMinAngleDeg(), t.rackMaxAngleDeg(),
            t.rackOffsetDeg(), t.turretOffsetDeg(), t.flywheelDiameterMeters(), t.maxFlywheelRps(),
            t.speedScalar(), t.speedPerMeterMps(), t.flywheelRpsOffset(), t.shootOnTheMoveGain(),
            forward, backwardAt90, t.motionRackFirst(), t.motionRackMaxSwingDeg());
    }

    /** The old division of labour: arc fixed by the policy, flywheel absorbs the motion. */
    private static AimTuning flywheelCarriesMotion() {
        AimTuning t = tuning();
        return new AimTuning(
            t.gravityMps2(), t.descentMarginDeg(), t.rackMinAngleDeg(), t.rackMaxAngleDeg(),
            t.rackOffsetDeg(), t.turretOffsetDeg(), t.flywheelDiameterMeters(), t.maxFlywheelRps(),
            t.speedScalar(), t.speedPerMeterMps(), t.flywheelRpsOffset(), t.shootOnTheMoveGain(),
            t.feederForwardPushMps(), t.feederBackwardPushAt90Mps(), false, t.motionRackMaxSwingDeg());
    }

    /**
     * Where the ball actually lands, given a solution and the conditions it was solved for.
     *
     * <p>Undoes the command offsets to recover the physical launch velocity, adds back the two
     * velocities the solver expected the ball to receive for free, then integrates ballistically to
     * the target's height on the way down.
     *
     * <p>The reconstruction is deliberately done the way the mechanism does it — build the velocity
     * in the robot's own axes from the two joint angles, then carry it into the field frame by the
     * robot's orientation. That is what makes these tests able to catch a tilt-compensation error:
     * a solver that got the rotation backwards produces joint angles that fly the ball somewhere
     * else entirely once they are interpreted on a robot that is actually tipped over.
     */
    private static Translation3d simulateLanding(
            AimSolution solution,
            Translation3d launchPosition,
            Rotation3d robotOrientation,
            double launchPointVelX,
            double launchPointVelY,
            double targetZ,
            AimTuning t) {

        double speed = (solution.flywheelRps() - t.flywheelRpsOffset())
            * Math.PI * t.flywheelDiameterMeters();
        double elevationRad = Math.toRadians(90.0 - (solution.rackAngleDeg() - t.rackOffsetDeg()));
        double turretAngleRad = Math.toRadians(solution.turretAngleDeg() - t.turretOffsetDeg());

        // Barrel direction in the robot's own axes: the turret's yaw about the chassis vertical,
        // the rack's elevation from the chassis plane.
        Translation3d robotFrameVel = new Translation3d(
            speed * Math.cos(elevationRad) * Math.cos(turretAngleRad),
            speed * Math.cos(elevationRad) * Math.sin(turretAngleRad),
            speed * Math.sin(elevationRad));

        // The feeder's push follows the turret angle the shot actually goes out at, which is what
        // the solver had to converge on — so this also checks that convergence landed somewhere
        // self-consistent, not just that the arithmetic ran. It acts along the chassis's forward
        // axis, so it belongs here, before the vector leaves the robot frame.
        robotFrameVel = robotFrameVel.plus(
            new Translation3d(AimSolver.feederPushForwardMps(turretAngleRad, t), 0.0, 0.0));

        Translation3d fieldVel = robotFrameVel.rotateBy(robotOrientation);

        // The launch point carries the ball with it — velocity the shooter deliberately did not
        // supply, so add it back to get the ball's true velocity.
        double vx = fieldVel.getX() + launchPointVelX * t.shootOnTheMoveGain();
        double vy = fieldVel.getY() + launchPointVelY * t.shootOnTheMoveGain();
        double vz = fieldVel.getZ();

        // Descending root of  launchZ + vz*t - g*t^2/2 = targetZ
        double dz = targetZ - launchPosition.getZ();
        double discriminant = vz * vz - 2 * t.gravityMps2() * dz;
        assertTrue(discriminant >= 0, "ball never reaches target height");
        double flightTime = (vz + Math.sqrt(discriminant)) / t.gravityMps2();

        return new Translation3d(
            launchPosition.getX() + vx * flightTime,
            launchPosition.getY() + vy * flightTime,
            targetZ);
    }

    private static void assertLandsOnTarget(
            Translation3d target,
            Translation3d launchPosition,
            double robotHeadingRad,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t) {
        assertLandsOnTarget(target, launchPosition, level(robotHeadingRad),
            launchPointVelX, launchPointVelY, t, ArcPolicy.DESCENT_MARGIN);
    }

    private static void assertLandsOnTarget(
            Translation3d target,
            Translation3d launchPosition,
            double robotHeadingRad,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t,
            ArcPolicy policy) {
        assertLandsOnTarget(target, launchPosition, level(robotHeadingRad),
            launchPointVelX, launchPointVelY, t, policy);
    }

    private static void assertLandsOnTarget(
            Translation3d target,
            Translation3d launchPosition,
            Rotation3d robotOrientation,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t,
            ArcPolicy policy) {

        AimSolution solution = AimSolver.solve(
            target, launchPosition, robotOrientation, launchPointVelX, launchPointVelY, t, policy);
        assertTrue(solution.feasible(), "solver reported an infeasible shot");

        Translation3d landing = simulateLanding(
            solution, launchPosition, robotOrientation,
            launchPointVelX, launchPointVelY, target.getZ(), t);

        assertEquals(target.getX(), landing.getX(), TOLERANCE_METERS, "landing X");
        assertEquals(target.getY(), landing.getY(), TOLERANCE_METERS, "landing Y");
    }

    /** A robot sitting flat, facing the given heading. */
    private static Rotation3d level(double headingRad) {
        return new Rotation3d(0.0, 0.0, headingRad);
    }

    /**
     * A robot with a wheel up on something. Positive roll is the left side lifted; positive pitch is
     * the nose down, which is WPILib's right-hand-rule convention and not always the one a gyro's
     * own datasheet uses.
     */
    private static Rotation3d tilted(double rollDeg, double pitchDeg, double headingRad) {
        return new Rotation3d(
            Math.toRadians(rollDeg), Math.toRadians(pitchDeg), headingRad);
    }

    private static final Translation3d TARGET = new Translation3d(8.0, 4.0, 2.2);
    private static final Translation3d LAUNCH = new Translation3d(4.0, 3.0, 0.4826);

    @Test
    void stationaryShotHitsTarget() {
        assertLandsOnTarget(TARGET, LAUNCH, Math.toRadians(30), 0.0, 0.0, tuning());
    }

    @Test
    void shotWhileTranslatingHitsTarget() {
        assertLandsOnTarget(TARGET, LAUNCH, Math.toRadians(30), 3.0, -2.0, tuning());
    }

    @Test
    void shotDrivingDirectlyAwayFromTargetHitsTarget() {
        assertLandsOnTarget(TARGET, LAUNCH, Math.toRadians(180), -3.5, 0.0, tuning());
    }

    @Test
    void feederPushIsCompensated() {
        assertLandsOnTarget(TARGET, LAUNCH, Math.toRadians(-70), 0.0, 0.0, withFeeder(0.45, 0.15));
    }

    @Test
    void feederAndMotionCompensateTogether() {
        assertLandsOnTarget(TARGET, LAUNCH, Math.toRadians(115), 2.5, 1.5, withFeeder(0.45, 0.15));
    }

    @Test
    void feederPushIsRearwardAndSymmetricAtNinetyDegrees() {
        AimTuning t = withFeeder(0.45, 0.15);

        double atZero = AimSolver.feederPushForwardMps(0.0, t);
        double atPlus90 = AimSolver.feederPushForwardMps(Math.toRadians(90), t);
        double atMinus90 = AimSolver.feederPushForwardMps(Math.toRadians(-90), t);
        double atOneEighty = AimSolver.feederPushForwardMps(Math.PI, t);

        assertEquals(0.45, atZero, 1e-9, "turret forward: push should be the full forward term");
        assertEquals(-0.15, atPlus90, 1e-9, "turret at +90: push should be rearward");
        assertEquals(-0.15, atMinus90, 1e-9, "turret at -90: same rearward push, not mirrored");
        assertEquals(atPlus90, atMinus90, 1e-12, "the rearward push must be symmetric about 0");
        assertEquals(0.45, atOneEighty, 1e-9, "turret straight back: barrel is aligned again");
    }

    @Test
    void feederPushIsCompensatedShootingOutTheSide() {
        // Turret near 90 degrees off the robot's heading is where the rearward term dominates.
        Translation3d sideTarget = new Translation3d(4.02, 8.0, 2.2);
        assertLandsOnTarget(sideTarget, LAUNCH, 0.0, 0.0, 0.0, withFeeder(0.45, 0.15));
    }

    @Test
    void shootOnTheMoveGainOfZeroIgnoresRobotVelocity() {
        AimTuning t = tuning();
        AimTuning noCompensation = new AimTuning(
            t.gravityMps2(), t.descentMarginDeg(), t.rackMinAngleDeg(), t.rackMaxAngleDeg(),
            t.rackOffsetDeg(), t.turretOffsetDeg(), t.flywheelDiameterMeters(), t.maxFlywheelRps(),
            t.speedScalar(), t.speedPerMeterMps(), t.flywheelRpsOffset(), 0.0,
            t.feederForwardPushMps(), t.feederBackwardPushAt90Mps(),
            t.motionRackFirst(), t.motionRackMaxSwingDeg());

        AimSolution moving = AimSolver.solve(TARGET, LAUNCH, 0.0, 3.0, -2.0, noCompensation);
        AimSolution still = AimSolver.solve(TARGET, LAUNCH, 0.0, 0.0, 0.0, noCompensation);

        assertEquals(still.turretAngleDeg(), moving.turretAngleDeg(), 1e-9);
        assertEquals(still.flywheelRps(), moving.flywheelRps(), 1e-9);
    }

    @Test
    void calibrationOffsetsShiftCommandsWithoutChangingPhysics() {
        AimTuning base = tuning();
        AimTuning offset = new AimTuning(
            base.gravityMps2(), base.descentMarginDeg(), base.rackMinAngleDeg(),
            base.rackMaxAngleDeg(), 3.0, -5.0, base.flywheelDiameterMeters(), base.maxFlywheelRps(),
            base.speedScalar(), base.speedPerMeterMps(), base.flywheelRpsOffset(),
            base.shootOnTheMoveGain(), base.feederForwardPushMps(), base.feederBackwardPushAt90Mps(),
            base.motionRackFirst(), base.motionRackMaxSwingDeg());

        AimSolution plain = AimSolver.solve(TARGET, LAUNCH, 0.5, 1.0, 1.0, base);
        AimSolution shifted = AimSolver.solve(TARGET, LAUNCH, 0.5, 1.0, 1.0, offset);

        assertEquals(plain.rackAngleDeg() + 3.0, shifted.rackAngleDeg(), 1e-9);
        assertEquals(plain.turretAngleDeg() - 5.0, shifted.turretAngleDeg(), 1e-9);
        assertEquals(plain.launchSpeedMps(), shifted.launchSpeedMps(), 1e-9);
    }

    /** A shuttle pass: long, and aimed at the carpet rather than at a goal's height. */
    private static final Translation3d FLOOR_TARGET = new Translation3d(14.0, 4.0, 0.0);

    @Test
    void shuttleShotPinsTheRackToItsHighStop() {
        AimTuning t = tuning();
        AimSolution solution = AimSolver.solve(
            FLOOR_TARGET, LAUNCH, Math.toRadians(20), 0.0, 0.0, t, ArcPolicy.FLATTEST);

        assertEquals(t.rackMaxAngleDeg(), solution.rackAngleDeg(), 1e-9,
            "a stationary shuttle shot should sit exactly on the rack's high stop");
        assertEquals(90.0 - t.rackMaxAngleDeg(), solution.launchAngleDeg(), 1e-9,
            "launch angle should be the flattest the mechanism can produce");
        assertTrue(!solution.rackClamped(),
            "choosing the flattest arc is the request, not a clamp");
    }

    @Test
    void shuttleShotIsFlatterThanTheSameShotWithDescentMargin() {
        AimSolution arced = AimSolver.solve(FLOOR_TARGET, LAUNCH, 0.0, 0.0, 0.0, tuning());
        AimSolution flat = AimSolver.solve(
            FLOOR_TARGET, LAUNCH, 0.0, 0.0, 0.0, tuning(), ArcPolicy.FLATTEST);

        assertTrue(flat.launchAngleDeg() < arced.launchAngleDeg(),
            "the shuttle policy must not pick a steeper arc than the default one");
        assertTrue(flat.launchSpeedMps() < arced.launchSpeedMps(),
            "the flatter arc is the point: it should also ask less of the flywheel");
    }

    @Test
    void shuttleShotLandsOnTheFloor() {
        assertLandsOnTarget(
            FLOOR_TARGET, LAUNCH, Math.toRadians(20), 0.0, 0.0, tuning(), ArcPolicy.FLATTEST);
    }

    @Test
    void movingShuttleShotLandsOnTheFloor() {
        assertLandsOnTarget(
            FLOOR_TARGET, LAUNCH, Math.toRadians(20), 2.0, 0.5,
            withFeeder(0.45, 0.15), ArcPolicy.FLATTEST);
    }

    /**
     * Far enough out that the descent-margin arc sits comfortably inside the rack's travel, so a
     * tilt has room to move the commanded rack angle without running into a stop. The close-range
     * TARGET above is already pinned against the rack's steep limit, which would mask the effect.
     */
    private static final Translation3d FAR_TARGET = new Translation3d(12.0, 3.0, 2.2);

    @Test
    void shotFromATiltedRobotHitsTarget() {
        // One wheel up on the depot: tipped over on both axes at once.
        assertLandsOnTarget(FAR_TARGET, LAUNCH, tilted(7.0, -4.0, Math.toRadians(35)),
            0.0, 0.0, tuning(), ArcPolicy.DESCENT_MARGIN);
    }

    @Test
    void shotFromATiltedRobotHitsTargetWithEveryOtherCorrectionRunning() {
        // Tilt, translation, and feeder push together — the corrections are applied to one shared
        // vector, so the thing worth checking is that they still compose.
        assertLandsOnTarget(FAR_TARGET, LAUNCH, tilted(-6.0, 5.0, Math.toRadians(-120)),
            2.5, -1.5, withFeeder(0.45, 0.15), ArcPolicy.DESCENT_MARGIN);
    }

    @Test
    void tiltedShuttlePassStillLandsWhereItWasAimed() {
        assertLandsOnTarget(FLOOR_TARGET, LAUNCH, tilted(5.0, 6.0, Math.toRadians(20)),
            1.0, 0.0, withFeeder(0.45, 0.15), ArcPolicy.FLATTEST);
    }

    @Test
    void noseDownPitchIsMadeUpForOneForOneByTheRack() {
        // Target dead ahead of a robot facing +X, so the shot is entirely in the pitch plane and the
        // arithmetic is exact: pitching the chassis nose-down by 10 degrees means the rack has to
        // elevate 10 degrees further to fire along the same line, and the rack's angle is measured
        // from vertical, so its command drops by exactly that.
        AimSolution levelShot = AimSolver.solve(FAR_TARGET, LAUNCH, level(0.0), 0.0, 0.0, tuning());
        AimSolution pitchedShot =
            AimSolver.solve(FAR_TARGET, LAUNCH, tilted(0.0, 10.0, 0.0), 0.0, 0.0, tuning());

        assertEquals(levelShot.rackAngleDeg() - 10.0, pitchedShot.rackAngleDeg(), 1e-9);
        assertEquals(0.0, pitchedShot.turretAngleDeg(), 1e-9,
            "a pure pitch with the target dead ahead should not move the turret");
        assertTrue(!pitchedShot.rackClamped(),
            "test geometry is wrong if the rack is against a stop — the effect would be hidden");
    }

    @Test
    void tiltChangesWhereToPointButNotHowHardToThrow() {
        // The compensation is a rotation, and a rotation preserves length. If a tilted robot is
        // being asked for a different flywheel speed than a level one in the same place, something
        // is scaling the vector that should not be.
        // Heading zero so the shot runs down the robot's own +X axis, where the roll and the pitch
        // each move one joint cleanly. At other headings they can partly cancel along the firing
        // line, which makes for a weaker assertion than it looks like.
        AimSolution levelShot =
            AimSolver.solve(FAR_TARGET, LAUNCH, level(0.0), 0.0, 0.0, tuning());
        AimSolution tiltedShot = AimSolver.solve(
            FAR_TARGET, LAUNCH, tilted(7.0, -4.0, 0.0), 0.0, 0.0, tuning());

        assertEquals(levelShot.launchSpeedMps(), tiltedShot.launchSpeedMps(), 1e-9);
        assertEquals(levelShot.flywheelRps(), tiltedShot.flywheelRps(), 1e-9);
        assertTrue(Math.abs(levelShot.rackAngleDeg() - tiltedShot.rackAngleDeg()) > 1.0,
            "the tilt should have moved the rack command");
        assertTrue(Math.abs(levelShot.turretAngleDeg() - tiltedShot.turretAngleDeg()) > 0.1,
            "a roll should have moved the turret command too");
    }

    @Test
    void zeroTiltSolvesIdenticallyToTheLevelOverload() {
        // The heading-only entry point is what the lookup-table path and every existing caller use.
        // It must stay bit-for-bit the same solve, or this change silently retunes the whole robot.
        AimSolution viaHeading = AimSolver.solve(TARGET, LAUNCH, 0.7, 2.0, -1.0, withFeeder(0.45, 0.15));
        AimSolution viaOrientation =
            AimSolver.solve(TARGET, LAUNCH, level(0.7), 2.0, -1.0, withFeeder(0.45, 0.15));

        assertEquals(viaHeading.turretAngleDeg(), viaOrientation.turretAngleDeg(), 1e-12);
        assertEquals(viaHeading.rackAngleDeg(), viaOrientation.rackAngleDeg(), 1e-12);
        assertEquals(viaHeading.flywheelRps(), viaOrientation.flywheelRps(), 1e-12);
    }

    /**
     * The whole point of handing the correction to the rack: the flywheel command must come out the
     * same as it would standing in the same spot, so a driver moving the sticks never asks a loaded
     * wheel to change speed. The ball still has to land on the target, which the shot tests above
     * cover — this one is about which joint did the work.
     */
    @Test
    void drivingAtTheTargetHoldsTheFlywheelAtItsStationarySpeed() {
        // Heading and velocity both down +X, straight at a target that is +X of the launch point.
        AimSolution still = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 0.0, 0.0, tuning());
        AimSolution moving = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 3.0, 0.0, tuning());

        assertEquals(still.flywheelRps(), moving.flywheelRps(), 0.01,
            "the flywheel command should not have moved at all");
        assertEquals(moving.nominalFlywheelRps(), moving.flywheelRps(), 0.01,
            "and it should be sitting exactly on the stationary reference it was solved against");
        assertTrue(!moving.motionRackSaturated(),
            "3 m/s at 8 m is well inside what the rack can absorb");
        assertTrue(moving.motionArcShiftDeg() > 2.0,
            "the arc is what should have moved instead");
        assertTrue(moving.rackAngleDeg() < still.rackAngleDeg() - 2.0,
            "steeper arc means a lower rack angle, since the rack is measured from vertical");
    }

    @Test
    void theOldBehaviourPutThatSameCorrectionInTheFlywheel() {
        AimTuning t = flywheelCarriesMotion();
        AimSolution still = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 0.0, 0.0, t);
        AimSolution moving = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 3.0, 0.0, t);

        assertTrue(Math.abs(moving.flywheelRps() - still.flywheelRps()) > 2.0,
            "with the switch off the flywheel must be the thing that changes");
        assertEquals(0.0, moving.motionArcShiftDeg(), 1e-12,
            "and the arc must be left exactly where the policy put it");
    }

    @Test
    void drivingAwayLeavesTheArcAloneAndSaysSo() {
        // Flattening is the mirror of steepening and barely recovers anything, so the solver
        // declines the trade rather than spending the descent margin on it.
        AimSolution still = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 0.0, 0.0, tuning());
        AimSolution moving = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, -3.0, 0.0, tuning());

        assertEquals(0.0, moving.motionArcShiftDeg(), 1e-12, "the arc should not have flattened");
        assertEquals(still.nominalFlywheelRps(), moving.nominalFlywheelRps(), 1e-9,
            "same spot, so the arc it was solved against is the same one");
        assertTrue(moving.motionRackSaturated(),
            "the flywheel is carrying this one, which has to be visible");
        assertTrue(moving.flywheelRps() > moving.nominalFlywheelRps() + 2.0,
            "and it is being asked for more speed than the stationary reference");
    }

    @Test
    void aStationaryShotIsUnaffectedByWhichJointCarriesMotion() {
        AimSolution rackFirst = AimSolver.solve(FAR_TARGET, LAUNCH, 0.4, 0.0, 0.0, tuning());
        AimSolution flywheelFirst =
            AimSolver.solve(FAR_TARGET, LAUNCH, 0.4, 0.0, 0.0, flywheelCarriesMotion());

        assertEquals(flywheelFirst.rackAngleDeg(), rackFirst.rackAngleDeg(), 1e-12);
        assertEquals(flywheelFirst.turretAngleDeg(), rackFirst.turretAngleDeg(), 1e-12);
        assertEquals(flywheelFirst.flywheelRps(), rackFirst.flywheelRps(), 1e-12);
    }

    @Test
    void theSwingCapHandsTheRemainderBackToTheFlywheel() {
        AimTuning t = tuning();
        AimTuning capped = new AimTuning(
            t.gravityMps2(), t.descentMarginDeg(), t.rackMinAngleDeg(), t.rackMaxAngleDeg(),
            t.rackOffsetDeg(), t.turretOffsetDeg(), t.flywheelDiameterMeters(), t.maxFlywheelRps(),
            t.speedScalar(), t.speedPerMeterMps(), t.flywheelRpsOffset(), t.shootOnTheMoveGain(),
            t.feederForwardPushMps(), t.feederBackwardPushAt90Mps(), true, 2.0);

        AimSolution uncapped = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 3.0, 0.0, t);
        AimSolution limited = AimSolver.solve(FAR_TARGET, LAUNCH, 0.0, 3.0, 0.0, capped);

        assertEquals(2.0, limited.motionArcShiftDeg(), 1e-6, "the arc should stop at the cap");
        assertTrue(limited.motionRackSaturated(), "stopping at the cap is saturation");
        assertTrue(limited.flywheelRps() < limited.nominalFlywheelRps() - 1.0,
            "whatever the arc could not take has to show up on the flywheel");
        assertTrue(uncapped.motionArcShiftDeg() > limited.motionArcShiftDeg(),
            "test geometry is wrong if the uncapped solve did not want to swing further");
    }

    @Test
    void aSaturatedArcStillLandsOnTarget() {
        // The rack giving up is a question of which joint is doing the work, never of accuracy:
        // every arc the search can choose, including the one it falls back to, hits the target.
        assertLandsOnTarget(FAR_TARGET, LAUNCH, Math.toRadians(140), -3.0, 1.5,
            withFeeder(0.45, 0.15), ArcPolicy.DESCENT_MARGIN);
    }

    @Test
    void movingShuttlePassAlsoHoldsTheFlywheelSteady() {
        // The shuttle policy pins the rack flat, so the arc has the whole rack travel available to
        // steepen into — and nothing below it, which is exactly the direction that was no use.
        AimSolution still = AimSolver.solve(
            FLOOR_TARGET, LAUNCH, 0.0, 0.0, 0.0, tuning(), ArcPolicy.FLATTEST);
        AimSolution moving = AimSolver.solve(
            FLOOR_TARGET, LAUNCH, 0.0, 3.0, 0.0, tuning(), ArcPolicy.FLATTEST);

        assertEquals(still.flywheelRps(), moving.flywheelRps(), 0.01);
        assertTrue(moving.motionArcShiftDeg() > 1.0, "the rack should have come off its high stop");
        assertTrue(moving.rackAngleDeg() < still.rackAngleDeg(),
            "and a shuttle pass taken while chasing it should be arced, not pinned");
    }

    @Test
    void unreachableTargetIsReportedNotSilentlyWrong() {
        // Directly overhead: no achievable rack angle produces a trajectory that gets there.
        Translation3d overhead = new Translation3d(4.02, 3.0, 5.0);
        AimSolution solution = AimSolver.solve(overhead, LAUNCH, 0.0, 0.0, 0.0, tuning());
        assertTrue(!solution.feasible() || solution.rackClamped(),
            "an impossible shot should be flagged, not returned as if it were fine");
    }
}
