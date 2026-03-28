package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShuttleMode extends Command {
    private final GroundIntakeSubsystem m_GroundIntake;
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final CommandSwerveDrivetrain m_Drivetrain;
    private final Timer m_timer = new Timer();

    public ShuttleMode(GroundIntakeSubsystem groundIntake, TurretSubsystem turret, ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain) {
        m_GroundIntake = groundIntake;
        m_Turret = turret;
        m_Shooter = shooter;
        m_Drivetrain = drivetrain;
        
        addRequirements(m_GroundIntake, m_Turret, m_Shooter);
    }

    @Override
    public void initialize() {

        
        m_GroundIntake.setPivotPosition(Constants.GroundIntakeConstants.SHOOTER_POSITION);
        m_timer.restart();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_GroundIntake.setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_INTAKE_SPEED);
        m_GroundIntake.setLeftIndexerMotorSpeed(Constants.GroundIntakeConstants.LEFT_INDEXER_SPEED);
        m_GroundIntake.setRightIndexerMotorSpeed(Constants.GroundIntakeConstants.RIGHT_INDEXER_SPEED);
        m_Turret.setTurretIndexerSpeed(Constants.TurretConstants.TURRET_INDEXER_SPEED);

        // Select target based on alliance color (defaults to blue if unknown)
        double tagY;
        double targetX;
        double targetZ;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            tagY = Constants.ShooterConstants.RED_SHUTTLE_TARGET_Y_METERS;
            targetX = Constants.ShooterConstants.RED_SHUTTLE_TARGET_X_METERS;
            targetZ = Constants.ShooterConstants.RED_SHUTTLE_TARGET_Z_METERS;
        } else {
            tagY = Constants.ShooterConstants.BLUE_SHUTTLE_TARGET_Y_METERS;
            targetX = Constants.ShooterConstants.BLUE_SHUTTLE_TARGET_X_METERS;
            targetZ = Constants.ShooterConstants.BLUE_SHUTTLE_TARGET_Z_METERS;
        }

        // Offset Y by +2 or -2 based on robot position relative to the tag
        double robotY = m_Drivetrain.getState().Pose.getY();
        double targetY = (robotY > tagY) ? tagY + 2.0 : tagY - 2.0;

        Translation3d targetPosition = new Translation3d(targetX, targetY, targetZ);
        m_Shooter.calculateShooterActions(targetPosition);
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

