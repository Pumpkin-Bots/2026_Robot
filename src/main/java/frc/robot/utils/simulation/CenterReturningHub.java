package frc.robot.utils.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;

/**
 * A {@link RebuiltHub} that, on every score, ejects the fuel out the back of the hub on an arc
 * that lands back at the center field pile, matching how a real HUB's return chute feeds scored
 * fuel back out for re-collection (maple-sim's stock {@link RebuiltHub} only nudges the piece a
 * few inches behind the hub and drops it).
 *
 * <p>The scored piece is removed the instant it enters the hub (that part is maple-sim's own
 * {@code Goal.simulationSubTick}, not overridden here), and the return piece used to be spawned
 * in that same instant — a same-frame teleport that read as the ball just vanishing rather than
 * passing through the hub. The return is now delayed by {@link #RETURN_DELAY_SECONDS}, so there's
 * a believable gap while it's "inside" the hub's mechanism before it comes out the back.
 */
public class CenterReturningHub extends RebuiltHub {

    // Roughly the centroid of the center fuel pile Arena2026Rebuilt.placeGamePiecesOnField() lays
    // out (bottom-right corner (7.35737, 1.724406), spread over its x/y grid).
    private static final Translation2d FUEL_RETURN_TARGET = new Translation2d(8.19, 3.9);

    private static final double RETURN_LAUNCH_PITCH_DEG = 40.0;

    // Matches the gravity constant maple-sim's own projectile physics uses (see
    // RebuiltFuelOnFly), so the computed launch speed actually lands on target.
    private static final double GRAVITY_MPS2 = 11.0;

    // How long the fuel appears to spend traveling through the hub's internal mechanism before
    // it's ejected out the back.
    private static final double RETURN_DELAY_SECONDS = 0.4;

    private final Random random = new Random();
    private final List<PendingReturn> pendingReturns = new ArrayList<>();

    private static final class PendingReturn {
        double remainingSeconds;
        final Translation2d exitPosition;
        final Rotation2d yaw;
        final double heightMeters;
        final double speedMPS;

        PendingReturn(double remainingSeconds, Translation2d exitPosition, Rotation2d yaw, double heightMeters, double speedMPS) {
            this.remainingSeconds = remainingSeconds;
            this.exitPosition = exitPosition;
            this.yaw = yaw;
            this.heightMeters = heightMeters;
            this.speedMPS = speedMPS;
        }
    }

    public CenterReturningHub(Arena2026Rebuilt arena, boolean isBlue) {
        super(arena, isBlue);
    }

    @Override
    protected void addPoints() {
        arena.addValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        arena.addValueToMatchBreakdown(isBlue, "WastedFuel", arena.isActive(isBlue) ? 0 : 1);
        arena.addToScore(isBlue, arena.isActive(isBlue) ? 1 : 0);

        Pose3d exitPose = (isBlue ? blueShootPoses : redShootPoses)[random.nextInt(4)];
        Translation2d exitPosition = exitPose.getTranslation().toTranslation2d();
        double launchHeightMeters = exitPose.getZ();

        double rangeMeters = exitPosition.getDistance(FUEL_RETURN_TARGET);
        Rotation2d yawTowardTarget = FUEL_RETURN_TARGET.minus(exitPosition).getAngle();

        double pitchRad = Math.toRadians(RETURN_LAUNCH_PITCH_DEG);
        double cosPitch = Math.cos(pitchRad);
        double denominator = 2 * cosPitch * cosPitch * (launchHeightMeters + rangeMeters * Math.tan(pitchRad));
        double launchSpeedMPS = Math.sqrt((GRAVITY_MPS2 * rangeMeters * rangeMeters) / denominator);

        pendingReturns.add(
                new PendingReturn(RETURN_DELAY_SECONDS, exitPosition, yawTowardTarget, launchHeightMeters, launchSpeedMPS));
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);

        double dtSeconds = SimulatedArena.getSimulationDt().in(Seconds);
        Iterator<PendingReturn> iterator = pendingReturns.iterator();
        while (iterator.hasNext()) {
            PendingReturn pending = iterator.next();
            pending.remainingSeconds -= dtSeconds;
            if (pending.remainingSeconds <= 0) {
                arena.addPieceWithVariance(
                        pending.exitPosition,
                        pending.yaw,
                        Meters.of(pending.heightMeters),
                        MetersPerSecond.of(pending.speedMPS),
                        Degrees.of(RETURN_LAUNCH_PITCH_DEG),
                        0.05,
                        0.05,
                        10,
                        0.3,
                        5);
                iterator.remove();
            }
        }
    }
}
