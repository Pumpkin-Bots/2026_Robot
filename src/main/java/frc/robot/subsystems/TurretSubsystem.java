package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class TurretSubsystem implements Subsystem {

    private final TalonFX m_turretIndexerMotor;

    private static final CurrentLimitsConfigs turretIndexerCurrentLimit = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);

    // FOC stated explicitly rather than relying on the Phoenix 6 default, which is already true.
    // This is the same request TalonFX.set() would have built internally, with the commutation mode
    // written down instead of implied.
    private final DutyCycleOut m_indexerRequest = new DutyCycleOut(0).withEnableFOC(true);

    /** Creates a new TurretSubsystem. */
    public TurretSubsystem() {
        m_turretIndexerMotor = new TalonFX(Constants.TurretConstants.TURRET_INDEXER_ID);
        m_turretIndexerMotor.getConfigurator().apply(turretIndexerCurrentLimit);
        m_turretIndexerMotor.optimizeBusUtilization();
    }

    public void setTurretIndexerSpeed(double speed) {
        m_turretIndexerMotor.setControl(m_indexerRequest.withOutput(speed));
    }

    public void stop() {
        m_turretIndexerMotor.setControl(m_indexerRequest.withOutput(0));
    }

    public Command disabledCommand() {
        return run(() -> {
            m_turretIndexerMotor.setControl(new NeutralOut());
        }).ignoringDisable(true);
    }

    public Command maintainStateCommand() {
        return run(() -> {
            setTurretIndexerSpeed(0.0);
        });
    }
}
