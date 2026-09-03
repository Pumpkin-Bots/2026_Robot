package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.FieldZones;

/**
 * Shuttles every ball toward our own hub, unconditionally.
 *
 * <p>{@link ShooterMode} now picks shoot-vs-shuttle on its own from where the shooter is standing,
 * so this is the manual override: it shuttles even from inside our own alliance zone. Kept bound to
 * its own button, and registered as a PathPlanner named command, for autos and for the times a
 * driver wants to force the pass.
 */
public class ShuttleMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Timer m_timer = new Timer();

    public ShuttleMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;

        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void initialize() {

        
        m_GroundIntake.resetAutoUnjam();
        m_GroundIntake.setPivotPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
        m_timer.restart();
        m_timer.start();
    }

    @Override
    public void execute() {
        // Same automatic jam recovery as ShooterMode — see runIntakeWithAutoUnjam().
        boolean unjamming = m_GroundIntake.runIntakeWithAutoUnjam();
        m_Turret.setTurretIndexerSpeed(unjamming
            ? -Constants.TurretConstants.TURRET_INDEXER_SPEED
            : Constants.TurretConstants.TURRET_INDEXER_SPEED);

        // Alliance's shuttle drop, offset to whichever side of the field the shooter is already on
        // (see FieldZones.shuttleTarget). Keyed off the shooter's launch point rather than the
        // robot centre, the same as every other zone decision.
        Translation3d targetPosition = FieldZones.shuttleTarget(
            FieldZones.isRedAlliance(), m_Shooter.getLaunchPosition().getY());
        m_Shooter.calculatePhysicsShuttleActions(targetPosition);
        if (m_timer.hasElapsed(0.75)) {
            m_GroundIntake.neutralMode();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}

