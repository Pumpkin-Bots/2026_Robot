package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.TurretSubsystem;


public class TestCommand extends Command {
    private final TurretSubsystem m_Turret;

    public TestCommand(TurretSubsystem turret) {
        m_Turret = turret;
    }
    @Override
    public void initialize() {
        DriverStation.reportWarning("Initialized Test Command", false);
    }
    @Override
    public void execute() {
        DriverStation.reportWarning("Executed Test Command", false);
        m_Turret.setTurretIndexerSpeed(Constants.TurretConstants.TURRET_INDEXER_SPEED);
        DriverStation.reportWarning("Execute Test Command after turret indexer", false);
    }
    @Override
    public boolean isFinished() {
        DriverStation.reportWarning("IsFinished Test Command", false);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.reportWarning("Ended Test Command", false);
    }
}
