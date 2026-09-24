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

        // The flywheel droop detector runs far faster than the main loop. A ball is in contact with
        // the wheel for roughly 10-25 ms, so at 50 Hz the dip it produces is at most one sample
        // wide — the detector would miss most shots and mis-measure the rest. This callback is
        // interleaved with the main loop on the same thread, so nothing here needs locking.
        addPeriodic(
            m_robotContainer::sampleFlywheelDroop,
            RobotContainer.flywheelSamplePeriodSeconds());
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
    public void disabledInit() {}

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
