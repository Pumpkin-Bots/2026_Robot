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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.ironmaple.simulation.SimulatedArena;

import frc.robot.commands.DefenseMode;
import frc.robot.commands.TrenchMode;
import frc.robot.commands.ShooterMode;
import frc.robot.commands.JamMode;
import frc.robot.commands.ShuttleMode;
import frc.robot.commands.StorageMode;
import frc.robot.commands.TestCommand;
import frc.robot.constants.Constants;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VelocityEstimator;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionPoseEstimate;
import frc.robot.utils.PowerBudget;

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
    // Runs its filter from periodic(), which the scheduler calls whether or not a command needs it.
    private final VelocityEstimator velocityEstimator = new VelocityEstimator(drivetrain);
    private final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain, velocityEstimator);

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
        NamedCommands.registerCommand("ShuttleMode", new ShuttleMode(intake, turret, shooter));
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

    /**
     * Left trigger, 0 to 1 — holds storage mode. Reads flat zero in simulation: the sim's raw axis
     * order on this Mac doesn't match the Driver Station's, and the left trigger is already spoken
     * for there by {@link #getFireTriggerInput()}, so storage and boost are simply disabled in sim
     * rather than fighting over an axis.
     */
    private double getStorageTriggerInput() {
        return RobotBase.isSimulation() ? 0.0 : joystick.getLeftTriggerAxis();
    }

    /** Right trigger, 0 to 1 — holds boost mode. Disabled in simulation, same as storage. */
    private double getBoostTriggerInput() {
        return RobotBase.isSimulation() ? 0.0 : joystick.getRightTriggerAxis();
    }

    // ---- Shooter-mode sub-states ----
    // Storage and boost are sub-functions of shooter mode from the driver's seat: the triggers do
    // nothing from defense, trench, jam, or shuttle mode, and releasing one drops straight back into
    // shooter mode. This flag is what "we are in shooter mode" means for that gate — set by the mode
    // buttons below, and false the moment any other mode takes over.
    private boolean m_shooterModeActive = false;

    /** Applies a mode's drive speeds and records whether it is shooter mode. */
    private void enterMode(boolean isShooterMode, Runnable speeds) {
        m_shooterModeActive = isShooterMode;
        speeds.run();
    }

    /**
     * Re-enters shooter mode after storage or boost is released. A fresh instance is built each
     * time: the one bound to the A button lives inside an alongWith() composition and can't be
     * scheduled on its own.
     *
     * <p>No-ops if shooter mode is no longer active — that happens when the driver presses another
     * mode button while still holding the trigger, and they should get the mode they just asked for,
     * not get yanked back into shooter mode when they let go.
     */
    private void restoreShooterMode() {
        if (!m_shooterModeActive) {
            return;
        }
        setShooterModeSpeeds();
        CommandScheduler.getInstance().schedule(new ShooterMode(intake, turret, shooter));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // The power-budget scale is applied here, at the driver's stick, rather than inside the
        // drivetrain — which means autonomous is untouched by it. That is deliberate: a path
        // follower that is quietly speed-limited does not drive its path slower, it drives a
        // different path. See PowerBudget for what the scale is protecting and why the drivetrain
        // is second in line behind the intake.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double driveScale = PowerBudget.driveOutputScale();
                return drive
                    .withVelocityX(applyLinearDeadband(-joystick.getLeftY()) * MaxSpeed * driveScale) // Drive forward with negative Y (forward)
                    .withVelocityY(applyLinearDeadband(-joystick.getLeftX()) * MaxSpeed * driveScale) // Drive left with negative X (left)
                    .withRotationalRate(applyLinearDeadband(-getRotationInput()) * MaxAngularRate * driveScale); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    //TO DO:    joystick.rightbumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // ---- Intake / robot-mode buttons ----
        // A  → shooter mode  (intake + auto shoot/shuttle/storage by field position, 25 % drive)
        // B  → trench mode   (pivot to -2 rot / ~45°, rollers off, normal drive speed)
        // Y  → shuttle mode  (force a shuttle pass from anywhere, 25 % drive)
        // X  → jam mode      (pivot at current position, rollers reverse, normal drive speed)
        // RB → defense mode  (intake stowed, normal drive speed)
        //
        // Each of these latches: it runs until something else takes the subsystems.

        joystick.rightBumper().onTrue(
            new DefenseMode(intake, turret, shooter).alongWith(
                Commands.runOnce(() -> enterMode(false, this::setNormalSpeeds)))
        );
        joystick.b().onTrue(
            new TrenchMode(intake, turret, shooter).alongWith(
                Commands.runOnce(() -> enterMode(false, this::setNormalSpeeds)))
        );
        joystick.a().onTrue(
            new ShooterMode(intake, turret, shooter).alongWith(
                Commands.runOnce(() -> enterMode(true, this::setShooterModeSpeeds)))
        );
        joystick.x().onTrue(
            new JamMode(intake, turret, shooter).alongWith(
                Commands.runOnce(() -> enterMode(false, this::setNormalSpeeds)))
        );
        joystick.y().onTrue(
            new ShuttleMode(intake, turret, shooter).alongWith(
                Commands.runOnce(() -> enterMode(false, this::setShooterModeSpeeds)))
        );

        // ---- Shooter-mode sub-states, held on the triggers ----
        // Left trigger  → storage mode: stop shooting, carry fuel, keep the 25 % shooter-mode speed.
        // Right trigger → boost mode: the same storage outputs at full drive speed, for sprinting
        //                 to the next pile without dumping power into a flywheel you aren't using.
        // Both are gated on shooter mode being active, so a trigger pressed from defense, trench,
        // jam, or shuttle mode does nothing at all. Releasing either returns to shooter mode.
        // Boost wins if both are held — it is the more specific request.
        final double triggerThreshold = Constants.DriveConstants.MODE_TRIGGER_THRESHOLD;
        Trigger storageTrigger = new Trigger(
            () -> m_shooterModeActive && getStorageTriggerInput() >= triggerThreshold);
        Trigger boostTrigger = new Trigger(
            () -> m_shooterModeActive && getBoostTriggerInput() >= triggerThreshold);

        boostTrigger.whileTrue(
            new StorageMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setBoostSpeeds))
        );
        storageTrigger.and(boostTrigger.negate()).whileTrue(
            new StorageMode(intake, turret, shooter).alongWith(Commands.runOnce(this::setShooterModeSpeeds))
        );

        // Registered after both whileTrue bindings so that on the loop a trigger is released, the
        // storage command is cancelled before this reschedules shooter mode underneath it.
        storageTrigger.or(boostTrigger).onFalse(Commands.runOnce(this::restoreShooterMode));

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

        // Sim-only: d-pad down puts all game pieces back in their starting piles.
        joystick.povDown().onTrue(
            Commands.runOnce(() -> {
                if (RobotBase.isSimulation()) {
                    SimulatedArena.getInstance().resetFieldForAuto();
                }
            }).ignoringDisable(true)
        );

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

    /**
     * Feeds the flywheel droop detector one fast sample. Scheduled by {@link Robot} at
     * {@link ShooterSubsystem#flywheelSamplePeriodSeconds()}, far faster than the main loop — see
     * {@link ShooterSubsystem#sampleFlywheelDroop()} for why it cannot live in periodic().
     */
    public void sampleFlywheelDroop() {
        shooter.sampleFlywheelDroop();
    }

    /** How often {@link #sampleFlywheelDroop()} wants to be called, in seconds. */
    public static double flywheelSamplePeriodSeconds() {
        return ShooterSubsystem.flywheelSamplePeriodSeconds();
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
        logger.updateScore();
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
        // Nothing leaves a parked flywheel. Besides being physically right, this is what keeps the
        // left trigger from both entering storage mode and firing the sim's projectiles at once.
        if (shooter.getCommandedFlywheelRps() <= 0.0) {
            return;
        }

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
     * Sim-only: immediately fires any fuel currently held in the intake, every loop. A plain
     * per-loop check rather than an edge-triggered Trigger, so it can't miss a pickup that
     * happens to land on the same loop it's consumed. No-ops on a real robot (hasFuel() is
     * always false there).
     */
    public void updateIntakeAutoFire() {
        // Same gate as updateAutoFire(): in storage/boost mode the flywheel is off and the feeder is
        // stopped, so picked-up fuel should stay in the hopper rather than teleporting out the barrel.
        if (shooter.getCommandedFlywheelRps() <= 0.0) {
            return;
        }

        if (intake.hasFuel() && intake.tryConsumeFuel()) {
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
     * Sets the drivetrain to boost mode speeds (full authority — the shooter is parked, so all the
     * power goes into getting to the next pile of fuel).
     */
    public void setBoostSpeeds() {
        MaxSpeed = BaseMaxSpeed * Constants.DriveConstants.BOOST_MAX_SPEED_MULTIPLIER;
        MaxAngularRate = BaseMaxAngularRate * Constants.DriveConstants.BOOST_MAX_ANGULAR_RATE_MULTIPLIER;
    }

    /**
     * Restores the drivetrain to normal driving speeds.
     */
    public void setNormalSpeeds() {
        MaxSpeed = BaseMaxSpeed * Constants.DriveConstants.NORMAL_MAX_SPEED_MULTIPLIER;
        MaxAngularRate = BaseMaxAngularRate * Constants.DriveConstants.NORMAL_MAX_ANGULAR_RATE_MULTIPLIER;
    }
}
