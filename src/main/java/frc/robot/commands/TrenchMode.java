package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class TrenchMode extends Command{
    private GroundIntakeSubsystem m_GroundIntake;
    public TrenchMode(GroundIntakeSubsystem groundIntake) {
        m_GroundIntake = groundIntake;
        addRequirements(m_GroundIntake);
    }

    @Override
    public void execute() {
        m_GroundIntake.setPivotMotorPosition(Constants.GroundIntakeConstants.TRENCH_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        m_GroundIntake.stop();
    }
}
