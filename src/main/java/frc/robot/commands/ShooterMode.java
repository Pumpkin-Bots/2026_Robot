package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
 *       not want the robot to suddenly get faster.
 * </ul>
 *
 * <p>Every boundary is sticky by {@code ZONE_HYSTERESIS_METERS} so parking on a line doesn't strobe
 * the flywheel or swing the turret back and forth.
 */
public class ShooterMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Timer m_timer = new Timer();

    // Previous frame's decisions, fed back in as the hysteresis bias.
    private boolean m_shooting = false;
    private boolean m_storage = false;

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
        m_storage = FieldZones.isInNoShootZone(launch.getX(), launch.getY(), 0.0);
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

        boolean storage = FieldZones.isInNoShootZone(shooterX, shooterY,
            m_storage ? Constants.FieldConstants.ZONE_HYSTERESIS_METERS : 0.0);

        if (storage != m_storage) {
            m_storage = storage;
            // Each crossing hands the intake off clean: entering gets a fresh chance to fill,
            // leaving gets an intake that will actually intake again.
            m_GroundIntake.resetHopperLatch();
            m_GroundIntake.resetAutoUnjam();
        }

        SmartDashboard.putString("Shooter/Mode",
            m_storage ? "STORAGE" : (m_shooting ? "SHOOT" : "SHUTTLE"));

        if (m_storage) {
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
