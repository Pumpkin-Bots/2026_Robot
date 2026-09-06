package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.DashboardToggle;
import frc.robot.utils.FieldZones;

/**
 * The one offensive mode: intake continuously and put fuel wherever it should go from here.
 *
 * <p>Which of three things it does is decided every loop from where the SHOOTER is — not the robot
 * centre, since the turret hangs off centre and the two disagree on a robot straddling a line:
 *
 * <ul>
 *   <li>Inside our own alliance zone → shoot at our hub.
 *   <li>Neutral zone or the opponent's alliance zone → shuttle the fuel back toward our hub.
 *   <li>Under a trench arm or a tower → {@link StorageMode} outputs (rack down, flywheel and feeder
 *       off, turret still tracking), because no shot leaves the robot from there anyway. Drive speed
 *       is deliberately left alone: this is not boost mode, and the driver threading a trench does
 *       not want the robot to suddenly get faster. This third case, and only this one, can be
 *       switched off from the dashboard — see {@link #ZONE_STORAGE_ENABLED}.
 * </ul>
 *
 * <p>Every boundary is sticky by {@code ZONE_HYSTERESIS_METERS} so parking on a line doesn't strobe
 * the flywheel or swing the turret back and forth.
 *
 * <p>A fourth, non-geometric reason to drop into storage outputs: the turret has run out of range
 * and is sweeping the long way round to the other side. Nothing is pointed at the target for the
 * whole of that sweep, so a shot taken during it goes wherever the turret happens to be. See
 * {@link #m_turretResetting}.
 */
public class ShooterMode extends Command {
    /**
     * Operator switch for the trench/tower rule only, default on. Off means the shooter keeps
     * shooting or shuttling from under a trench arm or a tower instead of dropping into storage.
     *
     * <p>It is here because that rule is the one part of this command running on surveyed field
     * geometry rather than on something the robot can measure: if the constants in
     * {@code FieldConstants} turn out to be off, or the field is not where odometry thinks it is,
     * the symptom is a robot that refuses to shoot from somewhere it can plainly shoot from. This
     * is the switch that gets the robot scoring again for the rest of the match.
     *
     * <p>Static because a fresh ShooterMode is constructed on every button press, and each instance
     * holding its own NetworkTables entry on the same topic would leak a handle per press.
     *
     * <p>Scoped to the zone rule deliberately — the turret-reset gate below is not affected, since
     * that one is driven by the turret's own measured state and shooting through it just throws
     * fuel in a random direction.
     */
    public static final DashboardToggle ZONE_STORAGE_ENABLED =
        new DashboardToggle("Shooter/TrenchTowerStorageEnabled", true);

    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Timer m_timer = new Timer();

    // Previous frame's decisions, fed back in as the hysteresis bias.
    private boolean m_shooting = false;
    private boolean m_storage = false;

    // Set when the turret wraps 360° to the other side of its range, cleared once it has arrived
    // within TURRET_RESET_TOLERANCE_DEG of the commanded angle. Held in storage outputs throughout.
    //
    // Deliberately latched off the discrete wrap event rather than off "error exceeds the
    // tolerance": a turret merely lagging a fast-rotating robot is still roughly on target and
    // still worth shooting through, and gating on error alone would drop a turret with a soft PID
    // or a tight wiring chain in and out of storage all match instead of letting it shoot.
    private boolean m_turretResetting = false;

    // Whether storage outputs were applied last loop, for either reason. The intake handoff below
    // keys off this rather than off the zone alone, so a turret reset hands the intake over the
    // same way a trench does.
    private boolean m_storageOutputs = false;

    public ShooterMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_GroundIntake.resetAutoUnjam();
        m_GroundIntake.resetHopperLatch();
        m_GroundIntake.setPivotPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);

        // Seed both latches unbiased, so the first loop decides on geometry alone rather than on
        // whatever the last run of this command happened to leave behind.
        Translation3d launch = m_Shooter.getLaunchPosition();
        m_shooting = FieldZones.isInOwnAllianceZone(
            launch.getX(), FieldZones.isRedAlliance(), false);
        m_storage = ZONE_STORAGE_ENABLED.get()
            && FieldZones.isInNoShootZone(launch.getX(), launch.getY(), 0.0);

        // Discard any wrap left flagged by a previous command, so this run doesn't open holding a
        // shot for a sweep that already finished.
        m_Shooter.consumeTurretWrap();
        m_turretResetting = false;
        m_storageOutputs = m_storage;
    }

    @Override
    public void execute() {
        Translation3d launch = m_Shooter.getLaunchPosition();
        double shooterX = launch.getX();
        double shooterY = launch.getY();
        boolean isRed = FieldZones.isRedAlliance();

        // Pick the aim point first — the turret tracks it even while in storage, so coming out of a
        // trench doesn't cost a turret swing before the first shot.
        m_shooting = FieldZones.isInOwnAllianceZone(shooterX, isRed, m_shooting);
        Translation3d target = m_shooting
            ? FieldZones.hubTarget(isRed)
            : FieldZones.shuttleTarget(isRed, shooterY);

        // Switching the toggle off mid-match drops m_storage on the next loop, which the handoff
        // below then sees as an ordinary storage exit — the intake is handed over the same way it
        // would be on driving out of a trench.
        m_storage = ZONE_STORAGE_ENABLED.get()
            && FieldZones.isInNoShootZone(shooterX, shooterY,
                m_storage ? Constants.FieldConstants.ZONE_HYSTERESIS_METERS : 0.0);

        // A wrap is only discovered while the turret is being commanded, so this picks up one
        // flagged by last loop's command — a single 20 ms cycle into a sweep that takes the better
        // part of a second, which is not long enough to get a ball out of the feeder.
        if (m_Shooter.consumeTurretWrap()) {
            m_turretResetting = true;
        }
        if (m_turretResetting && Math.abs(m_Shooter.getTurretErrorDeg())
                < Constants.ShooterConstants.TURRET_RESET_TOLERANCE_DEG) {
            m_turretResetting = false;
        }

        boolean storageOutputs = m_storage || m_turretResetting;
        if (storageOutputs != m_storageOutputs) {
            m_storageOutputs = storageOutputs;
            // Each crossing hands the intake off clean: entering gets a fresh chance to fill,
            // leaving gets an intake that will actually intake again.
            m_GroundIntake.resetHopperLatch();
            m_GroundIntake.resetAutoUnjam();
        }

        SmartDashboard.putString("Shooter/Mode",
            m_storage ? "STORAGE"
                : m_turretResetting ? "TURRET_RESET"
                : m_shooting ? "SHOOT" : "SHUTTLE");
        SmartDashboard.putBoolean("Shooter/TurretResetting", m_turretResetting);

        if (storageOutputs) {
            // The turret is still commanded at the target inside here, so the sweep it is already
            // committed to carries on and the reset finishes as fast as it would have anyway.
            StorageMode.applyStorageOutputs(m_GroundIntake, m_Turret, m_Shooter, target);
            return;
        }

        if (m_shooting) {
            m_Shooter.calculatePhysicsShooterActions(target);
        } else {
            m_Shooter.calculatePhysicsShuttleActions(target);
        }

        if (m_timer.hasElapsed(0.25)) {
            // Reverses the intake path by itself for half a second whenever the roller stalls,
            // then goes back to intaking. The flywheel is commanded above either way, so it holds
            // its speed straight through the unjam.
            boolean unjamming = m_GroundIntake.runIntakeWithAutoUnjam();
            m_Turret.setTurretIndexerSpeed(unjamming
                ? -Constants.TurretConstants.TURRET_INDEXER_SPEED
                : Constants.TurretConstants.TURRET_INDEXER_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.resetHopperLatch();
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}
