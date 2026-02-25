// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.HomeMode;
import frc.robot.commands.TrenchMode;
import frc.robot.commands.ShooterMode;
import frc.robot.commands.JamMode;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionPoseEstimate;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final GroundIntakeSubsystem intake = new GroundIntakeSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands for PathPlanner BEFORE configuring AutoBuilder
        registerNamedCommands();

        drivetrain.configurePathPlanner();

        // Build auto chooser from all PathPlanner autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureVision();
    }

    /**
     * Registers commands with PathPlanner for use in autonomous paths and event markers.
     * These commands can be referenced by name in PathPlanner GUI.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("HomeMode", new HomeMode(intake, turret, shooter));
        NamedCommands.registerCommand("TrenchMode", new TrenchMode(intake, turret, shooter));
        NamedCommands.registerCommand("ShooterMode", new ShooterMode(intake, turret, shooter));
        NamedCommands.registerCommand("JamMode", new JamMode(intake, turret, shooter));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    //TO DO:    joystick.rightbumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // ---- Intake / robot-mode buttons ----
        // A → shooter mode  (pivot to -4 rot / horizontal, rollers at 20 %)
        // B → trench mode   (pivot to -2 rot / ~45°, rollers off)
        // Y → home position (pivot to 0 rot / vertical, rollers off)
        
        joystick.y().onTrue(new HomeMode(intake, turret, shooter));
        joystick.b().onTrue(new TrenchMode(intake, turret, shooter));
        joystick.a().onTrue(new ShooterMode(intake, turret, shooter));
        joystick.x().onTrue(new JamMode(intake, turret, shooter));

        // Zero turret encoder on first enable (turret must be facing forward).
        RobotModeTriggers.disabled().onFalse(shooter.runOnce(() -> shooter.zeroTurretEncoderOnce()));

        // Apply disabled behaviour while the robot is disabled
        RobotModeTriggers.disabled().whileTrue(intake.disabledCommand());
        RobotModeTriggers.disabled().whileTrue(turret.disabledCommand());
        RobotModeTriggers.disabled().whileTrue(shooter.disabledCommand());

        // Default commands: continuously re-apply current state so motor outputs
        // are restored correctly after a disabled → enabled transition.
        intake.setDefaultCommand(intake.maintainStateCommand());
        turret.setDefaultCommand(turret.maintainStateCommand());
        shooter.setDefaultCommand(shooter.maintainStateCommand());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configures the vision subsystem to update the drivetrain pose estimator.
     * This runs periodically to fuse vision measurements with odometry.
     */
    private void configureVision() {
        // Run vision updates periodically as a default command
        vision.setDefaultCommand(
            vision.run(() -> {
                // Get all valid pose estimates from cameras
                List<VisionPoseEstimate> estimates = vision.getEstimatedPoses();

                // Add each estimate to the drivetrain's pose estimator
                for (VisionPoseEstimate estimate : estimates) {
                    drivetrain.addVisionMeasurement(
                        estimate.pose(),
                        estimate.timestampSeconds(),
                        estimate.standardDeviations()
                    );
                }

                // Update telemetry with vision data
                logger.updateVision(estimates, vision.isBackLeftConnected(), vision.isBackRightConnected(), vision.isIntakeRightConnected(), vision.isIntakeLeftConnected());
            })
        );
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        return autoChooser.getSelected();
    }
}
