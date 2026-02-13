package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class GroundIntakeSubsystem implements Subsystem {

    private final TalonFX m_leftPivotMotor;
    private final TalonFX m_rightPivotMotor;
    private final TalonFX m_rollerMotor;

    private static final Slot0Configs intakeGains = new Slot0Configs()
        .withKP(Constants.GroundIntakeConstants.PIVOT_KP)
        .withKI(Constants.GroundIntakeConstants.PIVOT_KI)
        .withKD(Constants.GroundIntakeConstants.PIVOT_KD);

    /** Creates a new GroundIntakeSubsystem. */
    public GroundIntakeSubsystem() {
        m_leftPivotMotor = new TalonFX(Constants.GroundIntakeConstants.LEFT_PIVOT_ID);
        m_rightPivotMotor = new TalonFX(Constants.GroundIntakeConstants.RIGHT_PIVOT_ID);
        m_rollerMotor = new TalonFX(Constants.GroundIntakeConstants.ROLLER_ID);

        m_leftPivotMotor.getConfigurator().apply(intakeGains);
    }

    public void setPivotMotorPosition(double position) {
        m_leftPivotMotor.setControl(new PositionVoltage(position));
        m_rightPivotMotor.setControl(new StrictFollower(Constants.GroundIntakeConstants.LEFT_PIVOT_ID));
    }

    public void setRollerSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    public void stop() {
        m_rollerMotor.set(0);
        m_leftPivotMotor.setControl(new StaticBrake());
        m_rightPivotMotor.setControl(new StaticBrake());
    }

    public Command disabledCommand() {
        return run(() -> {
            m_leftPivotMotor.setControl(new StaticBrake());
            m_rightPivotMotor.setControl(new StaticBrake());
            m_rollerMotor.setControl(new NeutralOut());
        }).ignoringDisable(true);
    }

    public Command maintainStateCommand() {
        return run(() -> {
            setPivotMotorPosition(Constants.GroundIntakeConstants.HOME_POSITION);
            setRollerSpeed(0.0);
        });
    }
}
