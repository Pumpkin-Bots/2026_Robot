package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class ShooterSubsystem implements Subsystem {

    private final TalonFX m_turretRotatorMotor;
    private final TalonFX m_shooterRackMotor;
    private final TalonFX m_shooterFlywheelMotor;

    private static final Slot0Configs turretRotatorGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.ROTATOR_KP)
        .withKI(Constants.ShooterConstants.ROTATOR_KI)
        .withKD(Constants.ShooterConstants.ROTATOR_KD);

    private static final Slot0Configs rackGains = new Slot0Configs()
        .withKP(Constants.ShooterConstants.RACK_KP)
        .withKI(Constants.ShooterConstants.RACK_KI)
        .withKD(Constants.ShooterConstants.RACK_KD);

    /** Creates a new GroundIntakeSubsystem. */
    public ShooterSubsystem() {
        m_turretRotatorMotor = new TalonFX(Constants.ShooterConstants.TURRET_ROTATOR_ID);
        m_shooterRackMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_RACK_ID);
        m_shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_ID);

        m_turretRotatorMotor.getConfigurator().apply(turretRotatorGains);
        m_shooterRackMotor.getConfigurator().apply(rackGains);
    }
       

    public void setTurretRotatorPosition(double position) {
        m_turretRotatorMotor.setControl(new PositionVoltage(position));
    }

    public void setShooterRackPosition(double position) {
        m_shooterRackMotor.setControl(new PositionVoltage(position));
    }

    public void setShooterFlywheelVelocity(double velocity) {
        m_shooterFlywheelMotor.setControl(new VelocityDutyCycle(velocity));
    }

    public void stop() {
        m_turretRotatorMotor.setControl(new StaticBrake());
        m_shooterRackMotor.setControl(new NeutralOut());
        m_shooterFlywheelMotor.setControl(new NeutralOut());
    }

    public Command disabledCommand() {
        return run(() -> {
            m_turretRotatorMotor.setControl(new NeutralOut());
            m_shooterRackMotor.setControl(new NeutralOut());
            m_shooterFlywheelMotor.setControl(new NeutralOut());
        }).ignoringDisable(true);
    }

    public Command maintainStateCommand() {
        return run(() -> {
            setTurretRotatorPosition(0);
            setShooterRackPosition(0);
            setShooterFlywheelVelocity(0);
        });
    }
}
