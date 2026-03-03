package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class JamMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;

    public JamMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void execute() {
        m_GroundIntake.setPivotMotorPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
        m_GroundIntake.setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_JAM_SPEED);
        m_Turret.setTurretIndexerSpeed(-Constants.TurretConstants.TURRET_INDEXER_SPEED);

        // Select target based on alliance color (defaults to blue if unknown)
        Translation3d targetPosition = Constants.ShooterConstants.BLUE_TARGET_POSITION;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targetPosition = Constants.ShooterConstants.RED_TARGET_POSITION;
        }

        m_Shooter.calculateShooterActions(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}
