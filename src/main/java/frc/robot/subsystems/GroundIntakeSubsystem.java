// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {

    public enum IntakeState { HOME, TRENCH, SHOOTER }

    private final TalonFX leftPivot  = new TalonFX(GroundIntakeConstants.LEFT_PIVOT_ID);
    private final TalonFX rightPivot = new TalonFX(GroundIntakeConstants.RIGHT_PIVOT_ID);
    private final TalonFX roller     = new TalonFX(GroundIntakeConstants.ROLLER_ID);

    private final PositionVoltage pivotRequest    = new PositionVoltage(0).withSlot(0);
    private final Follower        rightFollower   = new Follower(GroundIntakeConstants.LEFT_PIVOT_ID, MotorAlignmentValue.Aligned);
    private final DutyCycleOut    rollerSpin      = new DutyCycleOut(0);
    private final NeutralOut      rollerStop      = new NeutralOut();

    private IntakeState currentState = IntakeState.HOME;

    public GroundIntakeSubsystem() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0 = new Slot0Configs()
            .withKP(GroundIntakeConstants.PIVOT_KP)
            .withKI(GroundIntakeConstants.PIVOT_KI)
            .withKD(GroundIntakeConstants.PIVOT_KD);
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftPivot.getConfigurator().apply(pivotConfig);
        leftPivot.clearStickyFaults();
        leftPivot.setPosition(0);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightPivot.getConfigurator().apply(rightConfig);
        rightPivot.clearStickyFaults();

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        roller.getConfigurator().apply(rollerConfig);
    }

    private double targetPosition() {
        return switch (currentState) {
            case HOME    -> GroundIntakeConstants.HOME_POSITION;
            case TRENCH  -> GroundIntakeConstants.TRENCH_POSITION;
            case SHOOTER -> GroundIntakeConstants.SHOOTER_POSITION;
        };
    }

    public Command homeCommand() {
        return runOnce(() -> {
            currentState = IntakeState.HOME;
            System.out.println("[GroundIntake] State -> HOME");
        });
    }

    public Command trenchCommand() {
        return runOnce(() -> {
            currentState = IntakeState.TRENCH;
            System.out.println("[GroundIntake] State -> TRENCH");
        });
    }

    public Command shooterCommand() {
        return runOnce(() -> {
            currentState = IntakeState.SHOOTER;
            System.out.println("[GroundIntake] State -> SHOOTER");
        });
    }

    @Override
    public void periodic() {
        leftPivot.setControl(pivotRequest.withPosition(targetPosition()));
        rightPivot.setControl(rightFollower);
        roller.setControl(currentState == IntakeState.SHOOTER
            ? rollerSpin.withOutput(GroundIntakeConstants.ROLLER_INTAKE_SPEED)
            : rollerStop);

        SmartDashboard.putString("Intake/State",         currentState.toString());
        SmartDashboard.putNumber("Intake/PivotPosition", leftPivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotTarget",   targetPosition());
    }
}
