package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class ShooterMode extends Command{
    private GroundIntakeSubsystem m_GroundIntake;
    public ShooterMode(GroundIntakeSubsystem groundIntake) {
        m_GroundIntake = groundIntake;
        addRequirements(m_GroundIntake);
    }

    @Override
    public void execute() {
        m_GroundIntake.setPivotMotorPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
        m_GroundIntake.setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
    }
}
