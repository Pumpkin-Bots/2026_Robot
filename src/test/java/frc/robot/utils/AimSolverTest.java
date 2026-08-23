package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.AimSolver.AimSolution;
import frc.robot.utils.AimSolver.AimTuning;
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
            0.0);   // feederBackwardPushAt90Mps
    }

    private static AimTuning withFeeder(double forward, double backwardAt90) {
        AimTuning t = tuning();
        return new AimTuning(
            t.gravityMps2(), t.descentMarginDeg(), t.rackMinAngleDeg(), t.rackMaxAngleDeg(),
            t.rackOffsetDeg(), t.turretOffsetDeg(), t.flywheelDiameterMeters(), t.maxFlywheelRps(),
            t.speedScalar(), t.speedPerMeterMps(), t.flywheelRpsOffset(), t.shootOnTheMoveGain(),
            forward, backwardAt90);
    }

    /**
     * Where the ball actually lands, given a solution and the conditions it was solved for.
     *
     * <p>Undoes the command offsets to recover the physical launch velocity, adds back the two
     * velocities the solver expected the ball to receive for free, then integrates ballistically to
     * the target's height on the way down.
     */
    private static Translation3d simulateLanding(
            AimSolution solution,
            Translation3d launchPosition,
            double robotHeadingRad,
            double launchPointVelX,
            double launchPointVelY,
            double targetZ,
            AimTuning t) {

        double speed = (solution.flywheelRps() - t.flywheelRpsOffset())
            * Math.PI * t.flywheelDiameterMeters();
        double elevationRad = Math.toRadians(90.0 - (solution.rackAngleDeg() - t.rackOffsetDeg()));
        double bearingRad = robotHeadingRad
            + Math.toRadians(solution.turretAngleDeg() - t.turretOffsetDeg());

        double vx = speed * Math.cos(elevationRad) * Math.cos(bearingRad);
        double vy = speed * Math.cos(elevationRad) * Math.sin(bearingRad);
        double vz = speed * Math.sin(elevationRad);

        // The launch point carries the ball with it, and the feeder shoves it — both are velocity
        // the shooter deliberately did not supply, so add them back to get the ball's true velocity.
        vx += launchPointVelX * t.shootOnTheMoveGain();
        vy += launchPointVelY * t.shootOnTheMoveGain();

        // The feeder's push follows the turret angle the shot actually goes out at, which is what
        // the solver had to converge on — so this also checks that convergence landed somewhere
        // self-consistent, not just that the arithmetic ran.
        double turretAngleRad = Math.toRadians(solution.turretAngleDeg() - t.turretOffsetDeg());
        double push = AimSolver.feederPushForwardMps(turretAngleRad, t);
        vx += push * Math.cos(robotHeadingRad);
        vy += push * Math.sin(robotHeadingRad);

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

        AimSolution solution = AimSolver.solve(
            target, launchPosition, robotHeadingRad, launchPointVelX, launchPointVelY, t);
        assertTrue(solution.feasible(), "solver reported an infeasible shot");

        Translation3d landing = simulateLanding(
            solution, launchPosition, robotHeadingRad,
            launchPointVelX, launchPointVelY, target.getZ(), t);

        assertEquals(target.getX(), landing.getX(), TOLERANCE_METERS, "landing X");
        assertEquals(target.getY(), landing.getY(), TOLERANCE_METERS, "landing Y");
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
            t.feederForwardPushMps(), t.feederBackwardPushAt90Mps());

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
            base.shootOnTheMoveGain(), base.feederForwardPushMps(), base.feederBackwardPushAt90Mps());

        AimSolution plain = AimSolver.solve(TARGET, LAUNCH, 0.5, 1.0, 1.0, base);
        AimSolution shifted = AimSolver.solve(TARGET, LAUNCH, 0.5, 1.0, 1.0, offset);

        assertEquals(plain.rackAngleDeg() + 3.0, shifted.rackAngleDeg(), 1e-9);
        assertEquals(plain.turretAngleDeg() - 5.0, shifted.turretAngleDeg(), 1e-9);
        assertEquals(plain.launchSpeedMps(), shifted.launchSpeedMps(), 1e-9);
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
