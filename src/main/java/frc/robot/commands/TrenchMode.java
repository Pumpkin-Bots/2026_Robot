package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrenchMode extends Command {
    private GroundIntakeSubsystem m_GroundIntake;
    private TurretSubsystem m_Turret;
    private ShooterSubsystem m_Shooter;
    public TrenchMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        addRequirements(m_GroundIntake);
        addRequirements(m_Turret);
        addRequirements(m_Shooter);
    }

    @Override
    public void execute() {
        m_GroundIntake.setPivotMotorPosition(Constants.GroundIntakeConstants.TRENCH_POSITION);
        m_Turret.setTurretIndexerSpeed(0);
        m_Shooter.setTurretRotatorPosition(0);
        m_Shooter.setShooterRackPosition(0);
        m_Shooter.setShooterFlywheelVelocity(0);
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
        m_Turret.stop();
        m_Shooter.stop();
    }
}
