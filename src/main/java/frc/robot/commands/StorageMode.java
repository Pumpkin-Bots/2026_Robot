package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.FieldZones;

/**
 * Carries fuel instead of firing it.
 *
 * <p>The flywheel and the feeder into the turret both stop, and the rack drops to its lowest
 * position so the shooter fits under a trench arm. The turret keeps tracking whatever it would be
 * aiming at from here, so the shot is already lined up the instant the robot comes back out.
 *
 * <p>The intake turns into a hopper: it runs normally until the roller jams once, then latches off.
 * The first jam means the throat is packed, so there is nowhere for another ball to go, and grinding
 * against it just burns the roller. Leaving this mode clears the latch and the intake resumes.
 *
 * <p>Held on the left trigger at 25% drive speed, and on the right trigger — "boost mode" — at full
 * drive speed, for sprinting to the next pile while still picking up whatever is on the way. Both
 * are sub-states of shooter mode: the triggers do nothing from any other mode, and releasing one
 * returns to shooter mode. {@link ShooterMode} also drops into these same outputs on its own
 * whenever the shooter is under a trench or a tower.
 */
public class StorageMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;

    public StorageMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void initialize() {
        m_GroundIntake.resetHopperLatch();
        m_GroundIntake.setPivotPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
    }

    @Override
    public void execute() {
        applyStorageOutputs(m_GroundIntake, m_Turret, m_Shooter, aimTargetFromHere(m_Shooter));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Hand the next mode an intake that will actually intake — the latch is scoped to storage.
        m_GroundIntake.resetHopperLatch();
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }

    /**
     * The aim point the shooter would be using from where it currently stands — the hub inside our
     * own alliance zone, the shuttle drop anywhere else. Storage mode does not fire, but it keeps
     * the turret on this point so no time is lost swinging it when firing resumes.
     */
    static Translation3d aimTargetFromHere(ShooterSubsystem shooter) {
        Translation3d launch = shooter.getLaunchPosition();
        boolean isRed = FieldZones.isRedAlliance();
        return FieldZones.isInOwnAllianceZone(launch.getX(), isRed, false)
            ? FieldZones.hubTarget(isRed)
            : FieldZones.shuttleTarget(isRed, launch.getY());
    }

    /**
     * The actual storage outputs, factored out so {@link ShooterMode} runs byte-for-byte the same
     * thing when it decides on its own that it is under a trench or a tower.
     */
    static void applyStorageOutputs(
            GroundIntakeSubsystem intake,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            Translation3d aimTarget) {
        shooter.setShooterFlywheelVelocity(0.0);
        shooter.setShooterRackAngle(Constants.ShooterConstants.RACK_STORAGE_ANGLE_DEG);
        shooter.aimTurretAt(aimTarget);

        // Feeder off — nothing should reach a stopped flywheel.
        turret.setTurretIndexerSpeed(0.0);

        intake.runIntakeUntilJam();
    }
}
