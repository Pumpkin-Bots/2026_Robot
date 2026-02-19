package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;

    private static final Translation3d TARGET_POSITION = new Translation3d(
        Constants.ShooterConstants.TARGET_X_METERS,
        Constants.ShooterConstants.TARGET_Y_METERS,
        Constants.ShooterConstants.TARGET_Z_METERS);

    public ShooterMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void execute() {
        m_GroundIntake.setPivotMotorPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
        m_GroundIntake.setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_INTAKE_SPEED);
        m_Turret.setTurretIndexerSpeed(Constants.TurretConstants.TURRET_INDEXER_SPEED);

        m_Shooter.calculateShooterActions(TARGET_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}
