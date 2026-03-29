package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

public class GroundIntakeSubsystem implements Subsystem {

    private final TalonFX m_leftPivotMotor;
    private final TalonFX m_rightPivotMotor;
    private final TalonFX m_rollerMotor;
    private final TalonFX m_rightIndexerMotor;
    private final TalonFX m_leftIndexerMotor;

    // Reusable control requests
    private final MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0).withSlot(0);
    private double m_pivotTargetPosition = 0.0;
    private final Follower m_rightPivotFollower = new Follower(
        Constants.GroundIntakeConstants.LEFT_PIVOT_ID, MotorAlignmentValue.Opposed);

    // ---- Pivot gains: PID + feed-forwards, with cosine gravity compensation ----
    // GravityTypeValue.Arm_Cosine tells Phoenix 6 to apply: kG * cos(2π * mechanism_pos)
    // This automatically scales gravity compensation based on the arm's angle, so you
    // only need to tune kG for the horizontal position and it handles the rest.
    private static final Slot0Configs kPivotGains = new Slot0Configs()
        .withKP(Constants.GroundIntakeConstants.PIVOT_KP)
        .withKI(Constants.GroundIntakeConstants.PIVOT_KI)
        .withKD(Constants.GroundIntakeConstants.PIVOT_KD)
        .withKS(Constants.GroundIntakeConstants.PIVOT_KS)
        .withKV(Constants.GroundIntakeConstants.PIVOT_KV)
        .withKA(Constants.GroundIntakeConstants.PIVOT_KA)
        .withKG(Constants.GroundIntakeConstants.PIVOT_KG)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withGravityArmPositionOffset(Constants.GroundIntakeConstants.PIVOT_GRAVITY_OFFSET);

    // ---- Motion Magic profile ----
    // Positions and velocities here are in mechanism rotations (after gear ratio),
    // because SensorToMechanismRatio is set below.
    private static final MotionMagicConfigs kPivotMotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(Constants.GroundIntakeConstants.PIVOT_CRUISE_VELOCITY_RPS)
        .withMotionMagicAcceleration(Constants.GroundIntakeConstants.PIVOT_ACCELERATION_RPS2);

    // ---- Feedback: tells Phoenix 6 the gear ratio so it works in mechanism rotations ----
    // SensorToMechanismRatio = rotor rotations per mechanism rotation (e.g., 100:1 box → 100.0).
    // This affects how Phoenix 6 interprets position AND how it computes the cosine angle for kG.
    private static final FeedbackConfigs kPivotFeedback = new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.GroundIntakeConstants.PIVOT_GEAR_RATIO);

    private static final CurrentLimitsConfigs kPivotCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(80);

    private static final CurrentLimitsConfigs kRollerCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(70);

    private static final CurrentLimitsConfigs kIndexerCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(50);

    public GroundIntakeSubsystem() {
        m_leftPivotMotor  = new TalonFX(Constants.GroundIntakeConstants.LEFT_PIVOT_ID);
        m_rightPivotMotor = new TalonFX(Constants.GroundIntakeConstants.RIGHT_PIVOT_ID);
        m_rollerMotor      = new TalonFX(Constants.GroundIntakeConstants.ROLLER_ID);
        m_leftIndexerMotor = new TalonFX(Constants.GroundIntakeConstants.LEFT_INDEXER_ID);
        m_rightIndexerMotor = new TalonFX(Constants.GroundIntakeConstants.RIGHT_INDEXER_ID);

        m_leftPivotMotor.getConfigurator().apply(kPivotGains);
        m_leftPivotMotor.getConfigurator().apply(kPivotMotionMagic);
        m_leftPivotMotor.getConfigurator().apply(kPivotFeedback);
        m_leftPivotMotor.getConfigurator().apply(kPivotCurrentLimits);
        // Register only the position signal 8we actually read, then silence everything else.
        m_leftPivotMotor.getPosition().setUpdateFrequency(50);
        m_leftPivotMotor.optimizeBusUtilization();

        // Right motor mirrors the left; configure it identically then set as follower.
        m_rightPivotMotor.getConfigurator().apply(kPivotGains);
        m_rightPivotMotor.getConfigurator().apply(kPivotMotionMagic);
        m_rightPivotMotor.getConfigurator().apply(kPivotFeedback);
        m_rightPivotMotor.getConfigurator().apply(kPivotCurrentLimits);
        m_rightPivotMotor.setControl(m_rightPivotFollower);
        m_rightPivotMotor.optimizeBusUtilization(); // follower — no readbacks needed

        m_rollerMotor.getConfigurator().apply(kRollerCurrentLimits);
        m_rollerMotor.optimizeBusUtilization();

        m_leftIndexerMotor.getConfigurator().apply(kIndexerCurrentLimits);
        m_leftIndexerMotor.optimizeBusUtilization();

        m_rightIndexerMotor.getConfigurator().apply(kIndexerCurrentLimits);
        m_rightIndexerMotor.optimizeBusUtilization();

        

        // Seed the encoder immediately, assuming the robot always starts with
        // the intake resting on the UP hard stop. Phoenix 6 resets rotor position
        // to 0 on every power cycle, so we tell it where the arm actually is.
        m_leftPivotMotor.setPosition(
            Constants.GroundIntakeConstants.PIVOT_UP_ROTATIONS
            * Constants.GroundIntakeConstants.PIVOT_GEAR_RATIO);
    }

    /**
     * Commands the pivot to a target position using Motion Magic.
     *
     * @param mechanismRotations Target angle expressed in mechanism rotations
     *   (0 = horizontal, positive = above horizontal, negative = below horizontal).
     *   Use the PIVOT_UP_ROTATIONS / PIVOT_DOWN_ROTATIONS constants.
     */
    public void setPivotPosition(double mechanismRotations) {
        m_pivotTargetPosition = mechanismRotations;
        m_leftPivotMotor.setControl(m_pivotRequest.withPosition(mechanismRotations));
        // Right motor follows automatically via the Follower set in the constructor.
    }

    /** Moves pivot to the stowed (up) position. */
    public void pivotUp() {
        setPivotPosition(Constants.GroundIntakeConstants.PIVOT_UP_ROTATIONS);
    }

    /** Moves pivot to the deployed (down) position. */
    public void pivotDown() {
        setPivotPosition(Constants.GroundIntakeConstants.PIVOT_DOWN_ROTATIONS);
    }

    /** Returns the current pivot position in mechanism rotations. */
    public double getPivotPosition() {
        return m_leftPivotMotor.getPosition().getValueAsDouble();
    }

    /** Returns true when the pivot is within tolerance of the target position. */
    public boolean isAtPosition(double targetMechanismRotations) {
        return Math.abs(getPivotPosition() - targetMechanismRotations)
            <= Constants.GroundIntakeConstants.PIVOT_TOLERANCE_ROTATIONS;
    }

    public boolean isInIntakePosition() {
        return isAtPosition(Constants.GroundIntakeConstants.PIVOT_DOWN_ROTATIONS);
    }

    public void setRollerSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    public void neutralMode(){
        m_leftPivotMotor.setControl(new NeutralOut());
        m_rightPivotMotor.setControl(new NeutralOut());
    }

    public void setLeftIndexerMotorSpeed(double speed) {
        m_leftIndexerMotor.set(speed);
    }

    public void setRightIndexerMotorSpeed(double speed) {
        m_rightIndexerMotor.set(speed);
    }

    public void stop() {
        m_leftPivotMotor.setControl(new StaticBrake());
        m_rightPivotMotor.setControl(m_rightPivotFollower);
        m_rollerMotor.set(0);
        m_leftIndexerMotor.set(0);
        m_rightIndexerMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PivotPosition", getPivotPosition());
        SmartDashboard.putNumber("Intake/PivotTarget", m_pivotTargetPosition);
        SmartDashboard.putBoolean("Intake/IsDown", isInIntakePosition());
    }

    public Command disabledCommand() {
        return run(() -> {
            m_leftPivotMotor.setControl(new StaticBrake());
            m_rollerMotor.setControl(new NeutralOut());
        }).ignoringDisable(true);
    }

    public Command maintainStateCommand() {
        return run(() -> {
            pivotUp();
            setRollerSpeed(0.0);
        });
    }
}
