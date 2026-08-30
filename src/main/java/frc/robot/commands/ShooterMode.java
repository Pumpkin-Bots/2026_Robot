package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Timer m_timer = new Timer();
    
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
        m_GroundIntake.setPivotPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
    }

    @Override
    public void execute() {

        // Select target based on alliance color (defaults to blue if unknown)
        Translation3d targetPosition = Constants.ShooterConstants.BLUE_TARGET_POSITION;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targetPosition = Constants.ShooterConstants.RED_TARGET_POSITION;
        }

        m_Shooter.calculatePhysicsShooterActions(targetPosition);

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
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}
