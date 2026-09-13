package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Closed-form ballistic aiming solver.
 *
 * <p>Deliberately a pure static function of its inputs: it touches no motors, no NetworkTables, and
 * no subsystem state. That makes it unit-testable without a robot, replayable against logged data
 * to explain a miss after the fact, and directly reusable as the generator for a precomputed
 * shot table.
 *
 * <p>The solve runs in one pass, no iteration:
 *
 * <ol>
 *   <li>Pick an elevation angle a fixed margin steeper than the minimum-energy ballistic angle, so
 *       the ball is guaranteed to be descending when it arrives instead of skimming the rim on the
 *       way up. Clamp it to what the rack can physically reach.
 *   <li>Solve the no-drag range equation for the launch speed that hits the target at that angle.
 *   <li>Express that stationary solution as a field-relative 3D velocity vector.
 *   <li>Rotate that vector out of the field frame and into the robot's own axes, by the inverse of
 *       the robot's full 3D orientation. This is what makes the shot tilt-compensated: the turret
 *       spins about the <em>chassis's</em> vertical axis and the rack elevates from the
 *       <em>chassis's</em> plane, so with a wheel up on the depot or a corner on the bump, both of
 *       those axes are tipped over with respect to the field.
 *   <li>Subtract every velocity the ball is going to receive for free — the launch point's motion
 *       across the field, and the feeder's shove — from that vector. Whatever remains is what the
 *       shooter itself must impart. Because the corrections are subtractions on the same vector,
 *       they compose exactly and in any order.
 *   <li>Convert the remaining vector back into turret angle, rack angle, and flywheel RPS, then
 *       apply the mechanical calibration offsets.
 * </ol>
 *
 * <p>Steps 4 and 5 are worth dwelling on: they are exact, not approximations. The rotation is a
 * rigid one, so it changes neither the speed asked of the flywheel nor the trajectory the ball
 * flies — it only re-expresses the same vector in the frame the mechanism's two joints actually
 * measure their angles in. And because the ball's field-relative velocity ends up identical to the
 * stationary, level solution, it lands in the identical place. There is no iteration on flight time
 * and no virtual target, because none is needed.
 */
public final class AimSolver {

    private AimSolver() {}

    /**
     * How the launch angle gets chosen before any of the velocity corrections run.
     *
     * <p>Both options end up in the same place mechanically — a rack angle — but they answer
     * different questions, so the choice belongs to the caller rather than being buried in tuning.
     */
    public enum ArcPolicy {
        /**
         * Steepest-safe: bias a fixed margin above the minimum-energy angle so the ball is on its
         * way down when it arrives. This is what a shot into the hub wants, because entering the
         * goal descending is the whole point.
         */
        DESCENT_MARGIN,

        /**
         * Pin the rack against its high stop, giving the flattest trajectory the mechanism can
         * produce. This is what a shuttle pass wants: the ball is going to the carpet, where
         * arriving descending buys nothing, and the flattest arc needs the least flywheel speed and
         * spends the least time in the air on the way across the field.
         */
        FLATTEST
    }

    /**
     * Passes used to converge the feeder correction against the turret angle it depends on. Two is
     * already past the point where the answer stops moving; three costs nothing and leaves margin
     * if the push values ever get tuned much larger.
     */
    private static final int FEEDER_SOLVE_ITERATIONS = 3;

    /**
     * The feeder's push on the ball, as a robot-frame forward velocity in m/s. Negative means the
     * ball is being pushed toward the robot's rear.
     *
     * <p>Both terms act along the robot's fore/aft axis and swap over as the turret sweeps:
     *
     * <ul>
     *   <li>With the turret pointing straight ahead or straight back, the feeder's shove runs down
     *       the barrel and the ball leaves fast — {@code feederForwardPushMps}, scaled by cos².
     *   <li>With the turret pointing 90 degrees to either side, that help is gone and the ball
     *       instead comes out carrying velocity toward the robot's rear —
     *       {@code feederBackwardPushAt90Mps}, scaled by sin².
     * </ul>
     *
     * <p>sin² (not sin) is what makes the rearward term symmetric: it is the same push whether the
     * turret is at +90 or −90 degrees, which is how the effect actually behaves. The two terms are
     * independent, so each can be tuned at the turret angle where the other contributes nothing.
     *
     * @param turretAngleRad turret angle relative to the robot, 0 = pointing at the robot's front
     */
    public static double feederPushForwardMps(double turretAngleRad, AimTuning t) {
        double cos = Math.cos(turretAngleRad);
        double sin = Math.sin(turretAngleRad);
        return t.feederForwardPushMps() * cos * cos
             - t.feederBackwardPushAt90Mps() * sin * sin;
    }

    /**
     * Every calibration value the solve depends on, captured as one immutable snapshot.
     *
     * <p>Bundling them means the live-tunable dashboard values are read once per loop rather than
     * mid-solve, so a value changed halfway through cannot produce an inconsistent answer — and an
     * offline caller can build one by hand with no NetworkTables involved.
     */
    public record AimTuning(
        double gravityMps2,
        double descentMarginDeg,
        double rackMinAngleDeg,
        double rackMaxAngleDeg,
        double rackOffsetDeg,
        double turretOffsetDeg,
        double flywheelDiameterMeters,
        double maxFlywheelRps,
        double speedScalar,
        double speedPerMeterMps,
        double flywheelRpsOffset,
        double shootOnTheMoveGain,
        double feederForwardPushMps,
        double feederBackwardPushAt90Mps
    ) {}

    /**
     * A complete firing solution, plus enough context to explain itself when a shot misses.
     *
     * @param turretAngleDeg  robot-relative turret command, calibration offset already applied
     * @param rackAngleDeg    rack command in degrees, calibration offset already applied
     * @param flywheelRps     flywheel motor command in rotations per second
     * @param launchSpeedMps  speed the shooter must impart, relative to the moving robot
     * @param launchAngleDeg  elevation the shooter must fire at, measured from the chassis plane
     *                        rather than from level — this is the angle the rack is commanded to,
     *                        so on a tilted robot it deliberately differs from the elevation above
     *                        horizontal the ball actually leaves at
     * @param horizontalDistM ground distance from launch point to target
     * @param flightTimeS     predicted time of flight, from the no-drag solution
     * @param rackClamped     the physics wanted a rack angle outside the mechanism's range
     * @param speedClamped    the physics wanted more speed than the flywheel can produce
     * @param feasible        false when no achievable trajectory reaches the target at all
     */
    public record AimSolution(
        double turretAngleDeg,
        double rackAngleDeg,
        double flywheelRps,
        double launchSpeedMps,
        double launchAngleDeg,
        double horizontalDistM,
        double flightTimeS,
        boolean rackClamped,
        boolean speedClamped,
        boolean feasible
    ) {
        /** True when the mechanism can actually deliver this solution as commanded. */
        public boolean achievable() {
            return feasible && !rackClamped && !speedClamped;
        }
    }

    /**
     * Solves for the turret, rack, and flywheel commands that put a ball on the target, using the
     * descending-arc policy appropriate for a shot into the goal.
     *
     * @param target           field-relative position of the target
     * @param launchPosition   field-relative position the ball leaves from
     * @param robotOrientation robot orientation relative to the field — yaw, pitch, and roll
     * @param launchPointVelX  field-relative X velocity of the launch point (not the robot center)
     * @param launchPointVelY  field-relative Y velocity of the launch point
     * @param t                calibration snapshot
     */
    public static AimSolution solve(
            Translation3d target,
            Translation3d launchPosition,
            Rotation3d robotOrientation,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t) {
        return solve(target, launchPosition, robotOrientation,
            launchPointVelX, launchPointVelY, t, ArcPolicy.DESCENT_MARGIN);
    }

    /**
     * Solves for a robot known to be sitting level, where orientation is heading and nothing else.
     *
     * @param robotHeadingRad robot heading, field-relative, in radians
     */
    public static AimSolution solve(
            Translation3d target,
            Translation3d launchPosition,
            double robotHeadingRad,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t) {
        return solve(target, launchPosition, new Rotation3d(0.0, 0.0, robotHeadingRad),
            launchPointVelX, launchPointVelY, t, ArcPolicy.DESCENT_MARGIN);
    }

    /**
     * Solves for a robot known to be sitting level, where orientation is heading and nothing else.
     *
     * @param robotHeadingRad robot heading, field-relative, in radians
     */
    public static AimSolution solve(
            Translation3d target,
            Translation3d launchPosition,
            double robotHeadingRad,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t,
            ArcPolicy policy) {
        return solve(target, launchPosition, new Rotation3d(0.0, 0.0, robotHeadingRad),
            launchPointVelX, launchPointVelY, t, policy);
    }

    /**
     * Solves for the turret, rack, and flywheel commands that put a ball on the target.
     *
     * @param target           field-relative position of the target
     * @param launchPosition   field-relative position the ball leaves from
     * @param robotOrientation robot orientation relative to the field. Yaw does what it always did.
     *                         Pitch and roll are what make the shot tilt-compensated: pass the
     *                         gyro's, and a robot with one wheel up on the depot aims as accurately
     *                         as one sitting flat. Pass zero for both to aim as if level.
     * @param launchPointVelX  field-relative X velocity of the launch point (not the robot center)
     * @param launchPointVelY  field-relative Y velocity of the launch point
     * @param t                calibration snapshot
     * @param policy           how to pick the launch angle before the corrections run
     */
    public static AimSolution solve(
            Translation3d target,
            Translation3d launchPosition,
            Rotation3d robotOrientation,
            double launchPointVelX,
            double launchPointVelY,
            AimTuning t,
            ArcPolicy policy) {

        double dx = target.getX() - launchPosition.getX();
        double dy = target.getY() - launchPosition.getY();
        double dz = target.getZ() - launchPosition.getZ();
        double horizontalDist = Math.hypot(dx, dy);
        double bearingRad = Math.atan2(dy, dx);

        // launch_angle = 90 - rack_angle, so the rack's max angle is the flattest possible shot.
        double flattestRad = Math.toRadians(90.0 - t.rackMaxAngleDeg());
        double steepestRad = Math.toRadians(90.0 - t.rackMinAngleDeg());

        double launchAngleRad;
        boolean rackClamped;
        if (policy == ArcPolicy.FLATTEST) {
            // Sitting on the rack's high stop is the request here, not a compromise, so this does
            // not count as clamped — nothing was taken away from the solve.
            launchAngleRad = flattestRad;
            rackClamped = false;
        } else {
            double minEnergyRad = Math.PI / 4 + 0.5 * Math.atan2(dz, horizontalDist);
            double desiredRad = minEnergyRad + Math.toRadians(t.descentMarginDeg());
            launchAngleRad = MathUtil.clamp(desiredRad, flattestRad, steepestRad);
            rackClamped = Math.abs(launchAngleRad - desiredRad) > 1e-9;
        }
        boolean feasible = true;

        // From dz = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta)), solved for v. The denominator is
        // how far the ball's straight-line aim clears the target by; if it isn't positive, the shot
        // is climbing too steeply to ever get there and no speed can fix it.
        double denominator = horizontalDist * Math.tan(launchAngleRad) - dz;
        if (denominator <= 0.0) {
            launchAngleRad = steepestRad;
            denominator = horizontalDist * Math.tan(launchAngleRad) - dz;
            rackClamped = true;
            if (denominator <= 0.0) {
                denominator = 1e-6;
                feasible = false;
            }
        }

        double cosAngle = Math.cos(launchAngleRad);
        double stationarySpeed = Math.sqrt(
            t.gravityMps2() * horizontalDist * horizontalDist / (2.0 * cosAngle * cosAngle * denominator));

        // Empirical speed calibration. A real ball loses energy to air the whole way, so the no-drag
        // solve always asks for less speed than reality needs, and the shortfall grows with range.
        // These two terms live here — on the field-frame stationary solution — because that is the
        // flight the drag actually happens during.
        double correctedSpeed = stationarySpeed * t.speedScalar() + t.speedPerMeterMps() * horizontalDist;

        double horizontalSpeed = correctedSpeed * cosAngle;
        double flightTime = horizontalSpeed > 1e-6 ? horizontalDist / horizontalSpeed : 0.0;

        // The field-relative velocity the ball must end up with.
        double ballVelX = horizontalSpeed * Math.cos(bearingRad);
        double ballVelY = horizontalSpeed * Math.sin(bearingRad);
        double ballVelZ = correctedSpeed * Math.sin(launchAngleRad);

        // Shoot on the move: the launch point is already carrying the ball across the field at this
        // velocity, so the shooter only needs to supply the difference.
        ballVelX -= launchPointVelX * t.shootOnTheMoveGain();
        ballVelY -= launchPointVelY * t.shootOnTheMoveGain();

        // Everything above this line is field-relative. The mechanism is not: the turret's yaw is
        // measured about the chassis's own vertical axis and the rack's elevation from the
        // chassis's own plane, and on a robot with a wheel up on the depot neither of those agrees
        // with the field's. Rotating the vector by the inverse of the robot's orientation
        // re-expresses it in exactly the frame those two joints work in, which is what makes the
        // decomposition below correct on a tilt rather than merely correct when flat.
        //
        // A rotation changes no lengths, so the flywheel speed this produces is the same one a
        // level robot would be asked for — tilt moves where the barrel has to point, not how hard
        // it has to throw. On a level robot the rotation is a pure yaw and this reduces exactly to
        // the heading subtraction it replaces.
        Translation3d robotFrameVel =
            new Translation3d(ballVelX, ballVelY, ballVelZ).rotateBy(robotOrientation.unaryMinus());
        double baseVelX = robotFrameVel.getX();
        double baseVelY = robotFrameVel.getY();
        double baseVelZ = robotFrameVel.getZ();

        // Feeder push, subtracted the same way — and in this frame it is a single axis, because the
        // push is bolted to the chassis and so points straight down the robot's +X whatever the
        // robot is sitting on. Its magnitude depends on where the turret is pointing, so this has to
        // iterate: the turret angle is what we are solving for. It converges immediately — the push
        // is well under 1 m/s against a launch speed around 15 m/s, so the turret angle it produces
        // barely moves the push on the next pass.
        double outVelX = baseVelX;
        double turretAngleRad = Math.atan2(baseVelY, baseVelX);
        for (int i = 0; i < FEEDER_SOLVE_ITERATIONS; i++) {
            outVelX = baseVelX - feederPushForwardMps(turretAngleRad, t);
            turretAngleRad = Math.atan2(baseVelY, outVelX);
        }

        double outHorizontal = Math.hypot(outVelX, baseVelY);
        double outSpeed = Math.hypot(outHorizontal, baseVelZ);
        double outElevationRad = Math.atan2(baseVelZ, outHorizontal);

        // Already robot-relative — the rotation above took the heading out, so unlike the field-frame
        // bearing this replaces, there is nothing left to subtract.
        double turretAngleDeg = Math.toDegrees(turretAngleRad) + t.turretOffsetDeg();
        double rackAngleDeg = 90.0 - Math.toDegrees(outElevationRad) + t.rackOffsetDeg();
        // The motion corrections and the tilt rotation both swing the commanded elevation away from
        // the angle solved for above, so a command that started inside the rack's travel can finish
        // outside it — most obviously when the arc is already pinned flat and the robot is driving
        // away from the target, or when the robot is nose-down on a bump and the rack runs out of
        // travel making up the difference. The mechanism will clamp that silently; say so instead.
        if (rackAngleDeg < t.rackMinAngleDeg() - 1e-9 || rackAngleDeg > t.rackMaxAngleDeg() + 1e-9) {
            rackClamped = true;
        }
        double flywheelRps =
            outSpeed / (Math.PI * t.flywheelDiameterMeters()) + t.flywheelRpsOffset();

        return new AimSolution(
            turretAngleDeg,
            rackAngleDeg,
            flywheelRps,
            outSpeed,
            Math.toDegrees(outElevationRad),
            horizontalDist,
            flightTime,
            rackClamped,
            flywheelRps > t.maxFlywheelRps(),
            feasible);
    }
}
