package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.DoubleUnaryOperator;

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
 *
 * <h2>Which joint pays for the motion</h2>
 *
 * <p>Step 1 picks <em>an</em> arc, but the target does not care which one: for a fixed target there
 * is a whole one-parameter family of launch angles that reach it, each with its own launch speed.
 * Step 5 then hands the entire shoot-on-the-move correction to the flywheel, because the arc was
 * already nailed down before the robot's velocity was ever looked at. That is the wrong joint to
 * hand it to. A loaded flywheel takes the better part of a second to move a few RPS and a ball fed
 * mid-change leaves at whatever speed the wheel happens to be passing through, whereas the rack is
 * a geared screw that tracks a degree-scale step in a fraction of that — and the correction itself
 * swings around at the rate the driver moves the sticks.
 *
 * <p>So the arc is chosen <em>after</em> the velocity instead: search the family for the launch
 * angle whose required shooter speed is the speed the flywheel would be spun to standing in this
 * same spot ({@code nominalFlywheelRps}). The flywheel command then tracks only the distance to the
 * target, which changes slowly, and the rack absorbs what the driver does.
 *
 * <p>The search only ever goes <em>steeper</em> than the policy's arc, and that asymmetry is
 * physical rather than a simplification. Driving at the target leaves the shooter with less to
 * supply, and steepening takes speed out of the horizontal — the one axis the robot's velocity acts
 * along — so a handful of degrees absorbs several m/s. Driving away leaves the shooter with more to
 * supply, and flattening is the mirror image that does <em>not</em> work: the lower total energy it
 * buys comes back out as a longer horizontal component for the robot's motion to add to, so on an
 * 8 m shot spending the whole 12 degrees of descent margin recovers about 0.2 m/s. Trading the
 * margin that keeps the ball off the rim for that is a bad deal, so it isn't offered. Driving away,
 * the flywheel still has to spin up, exactly as it always did.
 *
 * <p>Two things make this safe to search numerically. Every candidate angle is a genuine solution —
 * a bisection that converged badly would give a shot that is harder on the flywheel, never one that
 * misses. And the reference speed is the stationary solve, not the wheel's measured speed: pointing
 * the rack at what the flywheel is actually doing would couple the arc to the droop and recovery of
 * every ball fired through it.
 *
 * <p>{@code motionRackSaturated} marks every loop where the arc could not take the whole
 * correction — because the robot is moving away or sideways, or because the rack ran out of travel
 * steepening — and the flywheel is back to making up the difference.
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
     * Bisection passes used to find the arc that holds the flywheel at its stationary speed. The
     * bracket is the rack's whole travel, under 30 degrees, so this resolves the angle to well
     * under a thousandth of a degree — far finer than the mechanism can be commanded to, and the
     * cost is a few dozen trig calls on a loop that has 20 ms to fill.
     */
    private static final int ARC_SOLVE_ITERATIONS = 18;

    /**
     * Launch-point speed below which the arc is left alone, in m/s. At a crawl the correction is
     * smaller than the rack's own backlash, and skipping it keeps a stationary robot's solution
     * bit-for-bit identical to the one it got before any of this existed.
     */
    private static final double MIN_MOTION_FOR_ARC_SHIFT_MPS = 0.02;

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
        double feederBackwardPushAt90Mps,
        boolean motionRackFirst,
        double motionRackMaxSwingDeg
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
     * @param nominalFlywheelRps the flywheel command this same shot would get standing still. With
     *                        the rack carrying the motion correction this is what {@code
     *                        flywheelRps} should equal; the gap between the two is how much of the
     *                        correction the rack could not take
     * @param motionArcShiftDeg how far the arc was steepened past the policy's angle to absorb the
     *                        robot's motion, in degrees of launch elevation. Never negative — the
     *                        solver does not flatten, see the class comment — so this reads zero
     *                        whenever the rack is taking no part in the correction
     * @param motionRackSaturated the arc could not take the whole correction and the flywheel is
     *                        making up the rest, either because the robot is not moving toward the
     *                        target or because the rack ran out of travel steepening
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
        double nominalFlywheelRps,
        double motionArcShiftDeg,
        boolean motionRackSaturated,
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

        double minEnergyRad = Math.PI / 4 + 0.5 * Math.atan2(dz, horizontalDist);

        double nominalAngleRad;
        boolean rackClamped;
        if (policy == ArcPolicy.FLATTEST) {
            // Sitting on the rack's high stop is the request here, not a compromise, so this does
            // not count as clamped — nothing was taken away from the solve.
            nominalAngleRad = flattestRad;
            rackClamped = false;
        } else {
            double desiredRad = minEnergyRad + Math.toRadians(t.descentMarginDeg());
            nominalAngleRad = MathUtil.clamp(desiredRad, flattestRad, steepestRad);
            rackClamped = Math.abs(nominalAngleRad - desiredRad) > 1e-9;
        }
        boolean feasible = true;

        // The denominator inside the range equation is how far the ball's straight-line aim clears
        // the target by; if it isn't positive, the shot is climbing too steeply to ever get there
        // and no speed can fix it.
        if (horizontalDist * Math.tan(nominalAngleRad) - dz <= 0.0) {
            nominalAngleRad = steepestRad;
            rackClamped = true;
            if (horizontalDist * Math.tan(nominalAngleRad) - dz <= 0.0) {
                feasible = false;
            }
        }

        // Shoot on the move: the launch point is already carrying the ball across the field at this
        // velocity, so the shooter only needs to supply the difference.
        double motionVelX = launchPointVelX * t.shootOnTheMoveGain();
        double motionVelY = launchPointVelY * t.shootOnTheMoveGain();

        // The speed this shot would be asking the flywheel for with the robot standing still. It is
        // both the reference the arc search aims at and, published on its own, the way to see from
        // the driver station whether the rack is actually taking the correction.
        double nominalRps = evaluateArc(nominalAngleRad, horizontalDist, dz, bearingRad,
            robotOrientation, 0.0, 0.0, t).flywheelRps();

        double launchAngleRad = nominalAngleRad;
        boolean motionRackSaturated = false;

        if (t.motionRackFirst() && feasible
                && Math.hypot(motionVelX, motionVelY) > MIN_MOTION_FOR_ARC_SHIFT_MPS) {
            final double hd = horizontalDist;
            final double bearing = bearingRad;
            DoubleUnaryOperator rpsAt = angle -> evaluateArc(
                angle, hd, dz, bearing, robotOrientation, motionVelX, motionVelY, t).flywheelRps();

            // The search runs from the policy's own arc up to the steepest the rack can reach, and
            // never flatter. Steepening is the direction that works: it moves speed out of the
            // horizontal, which is the axis the robot's velocity acts on, so a few degrees swallow
            // several m/s. Flattening is the opposite trade and barely moves the number at all — on
            // an 8 m shot, giving up the entire 12 degrees of descent margin buys 0.2 m/s, because
            // the lower total energy is handed straight back as a longer horizontal component for
            // the robot's motion to add to. Not worth having, so it isn't offered: driving away
            // from the target, the flywheel still has to spin up, and motionRackSaturated says so.
            double lo = nominalAngleRad;
            double hi = Math.min(steepestRad,
                nominalAngleRad + Math.toRadians(Math.max(0.0, t.motionRackMaxSwingDeg())));

            double rpsLo = hi > lo ? rpsAt.applyAsDouble(lo) : 0.0;
            double rpsHi = hi > lo ? rpsAt.applyAsDouble(hi) : 0.0;

            if (hi <= lo || rpsHi <= rpsLo || nominalRps <= rpsLo) {
                // Nothing on offer: the arc is already against the rack's steep stop or the swing
                // cap, or the shot needs at least the stationary speed even at the nominal arc
                // because the robot is moving away from the target rather than at it. Leave the arc
                // where the policy put it — the only other option is flattening, which is the trade
                // ruled out above — and let the flywheel carry the difference as it always did.
                launchAngleRad = lo;
                motionRackSaturated = true;
            } else if (nominalRps >= rpsHi) {
                // The rack helps as much as it is allowed to, and the flywheel takes the remainder.
                launchAngleRad = hi;
                motionRackSaturated = true;
            } else {
                // rpsLo < nominalRps < rpsHi, so there is an arc in between that needs exactly the
                // stationary speed. Bisect for it.
                for (int i = 0; i < ARC_SOLVE_ITERATIONS; i++) {
                    double mid = 0.5 * (lo + hi);
                    if (rpsAt.applyAsDouble(mid) < nominalRps) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                launchAngleRad = 0.5 * (lo + hi);
            }
        }

        Shot shot = evaluateArc(launchAngleRad, horizontalDist, dz, bearingRad,
            robotOrientation, motionVelX, motionVelY, t);

        // Already robot-relative — the rotation inside evaluateArc took the heading out, so unlike
        // the field-frame bearing this replaces, there is nothing left to subtract.
        double turretAngleDeg = Math.toDegrees(shot.turretAngleRad()) + t.turretOffsetDeg();
        double rackAngleDeg = 90.0 - Math.toDegrees(shot.elevationRad()) + t.rackOffsetDeg();
        // The motion corrections and the tilt rotation both swing the commanded elevation away from
        // the angle solved for above, so a command that started inside the rack's travel can finish
        // outside it — most obviously when the arc is already pinned flat and the robot is driving
        // away from the target, or when the robot is nose-down on a bump and the rack runs out of
        // travel making up the difference. The mechanism will clamp that silently; say so instead.
        if (rackAngleDeg < t.rackMinAngleDeg() - 1e-9 || rackAngleDeg > t.rackMaxAngleDeg() + 1e-9) {
            rackClamped = true;
        }
        return new AimSolution(
            turretAngleDeg,
            rackAngleDeg,
            shot.flywheelRps(),
            shot.outSpeed(),
            Math.toDegrees(shot.elevationRad()),
            horizontalDist,
            shot.flightTimeS(),
            nominalRps,
            Math.toDegrees(launchAngleRad - nominalAngleRad),
            motionRackSaturated,
            rackClamped,
            shot.flywheelRps() > t.maxFlywheelRps(),
            feasible);
    }

    /**
     * One candidate arc, carried through every correction to the joint commands it implies.
     *
     * @param turretAngleRad robot-frame turret angle, before the calibration offset
     * @param elevationRad   robot-frame elevation the shooter fires at, before the rack offset
     * @param outSpeed       speed the shooter itself must impart
     * @param flywheelRps    that speed as a motor command, offset applied
     * @param flightTimeS    no-drag time of flight for the arc, which the corrections do not change
     */
    private record Shot(
        double turretAngleRad,
        double elevationRad,
        double outSpeed,
        double flywheelRps,
        double flightTimeS
    ) {}

    /**
     * Works out what the mechanism has to do to put the ball on the target along one specific arc.
     *
     * <p>Split out of {@link #solve} because the arc is no longer decided up front: the search for
     * the angle that holds the flywheel at its stationary speed needs to ask this question of a
     * dozen-odd candidate angles before picking one. Every one of them is a complete, valid shot —
     * the only thing that distinguishes them is how the work is divided between the rack and the
     * flywheel.
     *
     * @param launchAngleRad field-frame elevation to throw the ball at
     * @param motionVelX     launch-point X velocity already scaled by the shoot-on-the-move gain
     * @param motionVelY     launch-point Y velocity already scaled by the shoot-on-the-move gain
     */
    private static Shot evaluateArc(
            double launchAngleRad,
            double horizontalDist,
            double dz,
            double bearingRad,
            Rotation3d robotOrientation,
            double motionVelX,
            double motionVelY,
            AimTuning t) {

        // From dz = d*tan(theta) - g*d^2 / (2*v^2*cos^2(theta)), solved for v. The caller has
        // already rejected the angles where this denominator goes non-positive — the shot climbing
        // too steeply to ever arrive — so a floor here only keeps an infeasible solve finite
        // instead of handing back a NaN that would spread to every row of telemetry.
        double denominator = Math.max(horizontalDist * Math.tan(launchAngleRad) - dz, 1e-6);

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

        // The field-relative velocity the ball must end up with, less the part the moving launch
        // point is already supplying for free.
        double ballVelX = horizontalSpeed * Math.cos(bearingRad) - motionVelX;
        double ballVelY = horizontalSpeed * Math.sin(bearingRad) - motionVelY;
        double ballVelZ = correctedSpeed * Math.sin(launchAngleRad);

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

        return new Shot(
            turretAngleRad,
            Math.atan2(baseVelZ, outHorizontal),
            outSpeed,
            outSpeed / (Math.PI * t.flywheelDiameterMeters()) + t.flywheelRpsOffset(),
            flightTime);
    }
}
