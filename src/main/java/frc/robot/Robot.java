// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.utils.PowerBudget;
import frc.robot.utils.TuningMode;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        // No fast periodic callback for the flywheel droop detector: it samples on its own thread,
        // woken by each velocity frame as it arrives rather than by a timer. An addPeriodic callback
        // is interleaved with this loop on this thread, so a loop overrun starves it and frames are
        // lost in exactly the bursts of activity that accompany shooting — see
        // ShooterSubsystem.flywheelSamplerLoop().
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        TuningMode.periodic();
        // Before the scheduler, so every subsystem periodic and every command that runs this loop
        // sees one consistent answer about what the battery can currently afford.
        PowerBudget.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.updateMechanismTelemetry();
        m_robotContainer.updateSimulation();
        m_robotContainer.updateAutoFire();
        m_robotContainer.updateIntakeAutoFire();
    }

    @Override
    public void disabledInit() {
        // The flywheel's learned droop curve costs a hopper of practice balls to measure and is
        // still true after the robot is power-cycled, which is exactly what happens between
        // matches. Saving on disable means the write never lands in the middle of a shot.
        m_robotContainer.saveFlywheelLearning();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
