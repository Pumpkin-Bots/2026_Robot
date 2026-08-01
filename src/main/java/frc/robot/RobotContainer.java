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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.DefenseMode;
import frc.robot.commands.TrenchMode;
import frc.robot.commands.ShooterMode;
import frc.robot.commands.JamMode;
import frc.robot.commands.ShuttleMode;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionPoseEstimate;

public class RobotContainer {
    // Base speeds (100% capability)
    private final double BaseMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double BaseMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

    // Current speed limits (adjustable for different modes)
    private double MaxSpeed = BaseMaxSpeed * Constants.DriveConstants.NORMAL_MAX_SPEED_MULTIPLIER;
    private double MaxAngularRate = BaseMaxAngularRate * Constants.DriveConstants.NORMAL_MAX_ANGULAR_RATE_MULTIPLIER;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final GroundIntakeSubsystem intake = new GroundIntakeSubsystem(drivetrain);
    private final VisionSubsystem vision = new VisionSubsystem(intake);
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);

    private final LEDSubsystem leds = new LEDSubsystem();

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
        NamedCommands.registerCommand("TrenchMode", new TrenchMode(intake, turret, shooter));
        NamedCommands.registerCommand("ShooterMode", new ShooterMode(intake, turret, shooter));
        NamedCommands.registerCommand("JamMode", new JamMode(intake, turret, shooter));
        NamedCommands.registerCommand("ShuttleMode", new ShuttleMode(intake, turret, shooter, drivetrain));
        //NamedCommands.registerCommand("TestCommand", new TestCommand(turret));

    }

    private double applyLinearDeadband(double value) {
        double deadband = 0.1;
        if (Math.abs(value) < deadband) return 0.0;
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    /**
     * Returns the rotation joystick axis. On the real robot this is the right stick's X axis
     * (axis 4), which is correct for the Windows Driver Station. In simulation on this Mac, the
     * OS reports controller axes in a different order — right stick X shows up on axis 2 instead
     * — so this reads raw axis 2 only when simulating. Sim-only: never affects real-robot input.
     */
    private double getRotationInput() {
        return RobotBase.isSimulation() ? joystick.getRawAxis(2) : joystick.getRightX();
    }

    /**
     * Returns the left trigger's value, normalized to 0 (released) - 1 (fully pressed). On the
     * real robot this is the standard trigger axis. In simulation on this Mac, the OS reports it
     * as raw axis 5 ranging -1 to 1 instead of 0 to 1, so it's remapped here. Sim-only: never
     * affects real-robot input.
     */
    private double getFireTriggerInput() {
        return RobotBase.isSimulation()
            ? (joystick.getRawAxis(5) + 1.0) / 2.0
            : joystick.getLeftTriggerAxis();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(applyLinearDeadband(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(applyLinearDeadband(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(applyLinearDeadband(-getRotationInput()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        // A → shooter mode  (pivot to -4 rot / horizontal, rollers at 20 %, REDUCED DRIVE SPEED)
        // B → trench mode   (pivot to -2 rot / ~45°, rollers off, normal drive speed)
        // Y → home position (pivot to 0 rot / vertical, rollers off, normal drive speed)
        // X → jam mode      (pivot at current position, rollers reverse, normal drive speed)

        joystick.rightBumper().onTrue(
            new DefenseMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setNormalSpeeds))
        );
        joystick.b().onTrue(
            new TrenchMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setNormalSpeeds))
        );
        joystick.a().onTrue(
            new ShooterMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setShooterModeSpeeds))
        );
        joystick.x().onTrue(
            new JamMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setNormalSpeeds))
        );
        joystick.y().onTrue(
            new ShuttleMode(intake, turret, shooter, drivetrain).alongWith(Commands.runOnce(this::setShooterModeSpeeds))
        );

        // Zero turret encoder on first enable (turret must be facing forward).
        RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> shooter.zeroTurretEncoderOnce()));

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

        // Sim-only: drive over a fuel piece with the intake deployed to pick it up, and it's
        // immediately fired from the shooter's current aim. hasFuel()/tryConsumeFuel() are both
        // no-ops on a real robot, so this trigger never fires there.
        new Trigger(intake::hasFuel).onTrue(Commands.runOnce(() -> {
            if (intake.tryConsumeFuel()) {
                shooter.launchProjectile();
            }
        }));

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
                logger.updateVision(estimates, vision.isBackLeftConnected(), vision.isBackRightConnected(), vision.isFrontRightConnected(), vision.isFrontLeftConnected());
            })
        );
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        return autoChooser.getSelected();
    }

    /** Publishes current mechanism angles as 3D poses for AdvantageScope. Called every loop from {@link Robot}. */
    public void updateMechanismTelemetry() {
        logger.updateMechanismPoses(
            shooter.getTurretRotatorAngleDeg(),
            shooter.getShooterRackAngleDeg(),
            intake.getPivotAngleDeg(),
            drivetrain.getState().ModuleStates,
            drivetrain.getModuleLocations()
        );
        logger.updateGamePieces();
    }

    /** Feeds ground-truth pose into simulated vision and telemetry each loop. No-op on a real robot. */
    public void updateSimulation() {
        drivetrain.getSimulatedGroundTruthPose().ifPresent(pose -> {
            vision.updateSimulatedVision(pose);
            logger.updateGroundTruthPose(pose);
        });
    }

    private static final double MAX_PROJECTILE_FIRE_RATE_PER_SEC = 15.0;
    private double m_lastProjectileFireTime = 0.0;

    /**
     * Sim-only: fires simulated projectiles at a rate scaled by how far the left trigger is
     * pressed, from 0 (released) up to MAX_PROJECTILE_FIRE_RATE_PER_SEC (fully pressed). No-op
     * on a real robot, since launchProjectile() itself is a no-op there.
     */
    public void updateAutoFire() {
        double rate = getFireTriggerInput() * MAX_PROJECTILE_FIRE_RATE_PER_SEC;
        if (rate <= 0.0) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (now - m_lastProjectileFireTime >= 1.0 / rate) {
            m_lastProjectileFireTime = now;
            shooter.launchProjectile();
        }
    }

    /**
     * Sets the drivetrain to shooter mode speeds (reduced for precise aiming).
     */
    public void setShooterModeSpeeds() {
        MaxSpeed = BaseMaxSpeed * Constants.DriveConstants.SHOOTER_MODE_MAX_SPEED_MULTIPLIER;
        MaxAngularRate = BaseMaxAngularRate * Constants.DriveConstants.SHOOTER_MODE_MAX_ANGULAR_RATE_MULTIPLIER;
    }

    /**
     * Restores the drivetrain to normal driving speeds.
     */
    public void setNormalSpeeds() {
        MaxSpeed = BaseMaxSpeed * Constants.DriveConstants.NORMAL_MAX_SPEED_MULTIPLIER;
        MaxAngularRate = BaseMaxAngularRate * Constants.DriveConstants.NORMAL_MAX_ANGULAR_RATE_MULTIPLIER;
    }
}
