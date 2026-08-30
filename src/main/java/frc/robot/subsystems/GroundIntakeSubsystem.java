package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;

public class GroundIntakeSubsystem implements Subsystem {

    private final TalonFX m_leftPivotMotor;
    private final TalonFX m_rightPivotMotor;
    private final TalonFX m_rollerMotor;
    private final TalonFX m_rightIndexerMotor;
    private final TalonFX m_leftIndexerMotor;

    // ---- Simulation ----
    // The pivot skips physics simulation and just snaps its simulated position straight to
    // whatever was last commanded (see simulationPeriodic()) — same approach as the shooter's
    // turret/rack, and for the same reason: a physics model here needs real tuning (moment of
    // inertia, gains) that isn't available, and the point of this sim is testing mode-switching
    // logic (e.g. DefenseMode raising the intake), not sim-only PID convergence.

    // ---- Simulation-only fuel pickup ----
    // "Touch it, get it" collision model from maple-sim — collects a "Fuel" piece on contact
    // while the intake is deployed. Capacity of 8 lets the intake buffer up to 8 pieces (e.g.
    // driving through a dense cluster faster than they can be fired) rather than a single-shot
    // feed. Placement/size (front, 20in wide, 8in extension) is a rough guess — adjust to match
    // the real intake's mounting side and footprint.
    private static final int INTAKE_CAPACITY = 8;
    private final IntakeSimulation m_intakeSim;

    // Caps how fast the robot can actually intake fuel — gates tryConsumeFuel(), which (once the
    // buffer above is full) also gates new pickups, so driving through a dense pile faster than
    // this rate just leaves the extra pieces on the field instead of collecting them.
    private static final double MAX_INTAKE_RATE_PER_SEC = 8.0;
    private double m_lastFuelConsumedTimestamp = -1.0 / MAX_INTAKE_RATE_PER_SEC;

    // ---- Automatic jam recovery ----
    // Roller signals used to spot a jam: a wedged ball stalls the roller, which reads as high
    // stator current while the motor is barely turning. Both are needed — high current alone is
    // just a hard-working intake, and low speed alone is just a stopped one.
    private final StatusSignal<Current> m_rollerStatorCurrent;
    private final StatusSignal<AngularVelocity> m_rollerVelocity;

    // Debounced stall state, recomputed once per loop in periodic() rather than on demand, so the
    // timing below is unaffected by how often callers ask. -1 means "not currently stalling".
    private double m_rollerStallStartTime = -1.0;
    private boolean m_rollerStalled = false;

    // Set while running the reverse burst; see runIntakeWithAutoUnjam().
    private boolean m_unjamming = false;
    private final Timer m_unjamTimer = new Timer();

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

    public GroundIntakeSubsystem(CommandSwerveDrivetrain drivetrain) {
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
        // Jam detection reads these two, so they have to be registered before
        // optimizeBusUtilization() silences everything that wasn't asked for.
        m_rollerStatorCurrent = m_rollerMotor.getStatorCurrent();
        m_rollerVelocity = m_rollerMotor.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(50, m_rollerStatorCurrent, m_rollerVelocity);
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

        if (RobotBase.isSimulation()) {
            m_intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                drivetrain.getMapleSimDrive(),
                Units.Inches.of(20),
                Units.Inches.of(8),
                IntakeSide.FRONT,
                INTAKE_CAPACITY
            );
        } else {
            m_intakeSim = null;
        }
    }

    /**
     * Returns true if the intake is currently holding a fuel piece. Always false on a real robot.
     *
     * <p>Synchronized on the arena instance because maple-sim increments the intake's piece
     * count from its own physics thread (inside SimulatedArena's synchronized
     * simulationPeriodic(), driven by the drivetrain's Notifier) — reading/writing it from the
     * main robot thread without this lock is a real race that eventually leaves the count stuck
     * at capacity, after which the intake can never register a new pickup again.
     */
    public boolean hasFuel() {
        if (m_intakeSim == null) {
            return false;
        }
        synchronized (SimulatedArena.getInstance()) {
            return m_intakeSim.getGamePiecesAmount() > 0;
        }
    }

    /**
     * Removes one held fuel piece, if any. Used to hand a piece off to the shooter on pickup.
     * Throttled to {@link #MAX_INTAKE_RATE_PER_SEC} — see the field comment above for why.
     * See {@link #hasFuel()} for why this is synchronized.
     *
     * @return true if a piece was present, removed, and the rate limit allowed it
     */
    public boolean tryConsumeFuel() {
        if (m_intakeSim == null) {
            return false;
        }
        double now = Timer.getFPGATimestamp();
        if (now - m_lastFuelConsumedTimestamp < 1.0 / MAX_INTAKE_RATE_PER_SEC) {
            return false;
        }
        synchronized (SimulatedArena.getInstance()) {
            boolean consumed = m_intakeSim.obtainGamePieceFromIntake();
            if (consumed) {
                m_lastFuelConsumedTimestamp = now;
            }
            return consumed;
        }
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

    /** Returns the current pivot angle in degrees (0 = horizontal, positive = above horizontal). */
    public double getPivotAngleDeg() {
        return getPivotPosition() * 360.0;
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

    public void forceDownMode(){
        m_leftPivotMotor.set(Constants.GroundIntakeConstants.PIVOT_FORCE_DOWN_POWER);
        m_rightPivotMotor.set(-Constants.GroundIntakeConstants.PIVOT_FORCE_DOWN_POWER);
    }

    /**
     * True when the roller motor has been stalled — high stator current while barely turning —
     * for longer than {@code ROLLER_STALL_DEBOUNCE_SECONDS}. Always false in simulation, where
     * the roller's sim state isn't driven and both signals read zero.
     */
    public boolean isRollerStalled() {
        return m_rollerStalled;
    }

    /**
     * Runs the roller and both indexers inward, automatically reversing the whole path at full
     * speed for {@code UNJAM_DURATION_SECONDS} whenever {@link #isRollerStalled()} trips, then
     * resuming intake on its own. Call once per loop from a command's execute().
     *
     * <p>Only the intake's own motors are touched — the shooter flywheel keeps running whatever
     * the calling command commanded, so it stays at speed through the unjam.
     *
     * @return true while the reverse burst is running, so the caller can reverse the turret
     *     indexer to match and clear the throat above the intake as well.
     */
    public boolean runIntakeWithAutoUnjam() {
        if (m_unjamming) {
            if (m_unjamTimer.hasElapsed(Constants.GroundIntakeConstants.UNJAM_DURATION_SECONDS)) {
                m_unjamming = false;
                // Make the detector earn a fresh full debounce window before it can fire again,
                // instead of re-triggering off the stall it was already tracking. If the ball is
                // still wedged this just means another burst a fraction of a second later.
                m_rollerStallStartTime = -1.0;
                m_rollerStalled = false;
            }
        } else if (m_rollerStalled) {
            m_unjamming = true;
            m_unjamTimer.restart();
        }

        if (m_unjamming) {
            setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_UNJAM_SPEED);
            setLeftIndexerMotorSpeed(Constants.GroundIntakeConstants.LEFT_INDEXER_UNJAM_SPEED);
            setRightIndexerMotorSpeed(Constants.GroundIntakeConstants.RIGHT_INDEXER_UNJAM_SPEED);
        } else {
            setRollerSpeed(Constants.GroundIntakeConstants.ROLLER_INTAKE_SPEED);
            setLeftIndexerMotorSpeed(Constants.GroundIntakeConstants.LEFT_INDEXER_SPEED);
            setRightIndexerMotorSpeed(Constants.GroundIntakeConstants.RIGHT_INDEXER_SPEED);
        }

        return m_unjamming;
    }

    /**
     * Clears any in-progress unjam and the stall detector's history. Call from the initialize()
     * of any command that intakes, so a jam detected under the previous command doesn't carry
     * over into the new one.
     */
    public void resetAutoUnjam() {
        m_unjamming = false;
        m_unjamTimer.stop();
        m_unjamTimer.reset();
        m_rollerStallStartTime = -1.0;
        m_rollerStalled = false;
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
        updateStallDetection();

        SmartDashboard.putNumber("Intake/PivotPosition", getPivotPosition());
        SmartDashboard.putNumber("Intake/PivotTarget", m_pivotTargetPosition);
        SmartDashboard.putBoolean("Intake/IsDown", isInIntakePosition());

        if (m_intakeSim != null) {
            // See hasFuel() for why this needs to be synchronized with the physics thread.
            synchronized (SimulatedArena.getInstance()) {
                if (isInIntakePosition()) {
                    m_intakeSim.startIntake();
                } else {
                    m_intakeSim.stopIntake();
                }
            }
        }
    }

    /**
     * Refreshes the roller signals and updates the debounced stall flag. Runs every loop from
     * periodic(), and always before commands execute (the scheduler runs subsystem periodics
     * first), so {@link #runIntakeWithAutoUnjam()} acts on a same-loop reading.
     */
    private void updateStallDetection() {
        BaseStatusSignal.refreshAll(m_rollerStatorCurrent, m_rollerVelocity);
        double amps = m_rollerStatorCurrent.getValueAsDouble();
        double rps = m_rollerVelocity.getValueAsDouble();

        boolean stallingNow =
            amps >= Constants.GroundIntakeConstants.ROLLER_STALL_CURRENT_AMPS
            && Math.abs(rps) <= Constants.GroundIntakeConstants.ROLLER_STALL_VELOCITY_RPS;

        double now = Timer.getFPGATimestamp();
        if (!stallingNow) {
            m_rollerStallStartTime = -1.0;
        } else if (m_rollerStallStartTime < 0.0) {
            m_rollerStallStartTime = now;
        }

        m_rollerStalled = m_rollerStallStartTime >= 0.0
            && now - m_rollerStallStartTime
               >= Constants.GroundIntakeConstants.ROLLER_STALL_DEBOUNCE_SECONDS;

        SmartDashboard.putNumber("Intake/RollerCurrent", amps);
        SmartDashboard.putNumber("Intake/RollerVelocity", rps);
        SmartDashboard.putBoolean("Intake/RollerStalled", m_rollerStalled);
        SmartDashboard.putBoolean("Intake/Unjamming", m_unjamming);
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

    /**
     * Snaps the simulated pivot straight to its last commanded position instead of running a
     * physics model through it — see the note on the simulation fields above for why. This motor
     * has SensorToMechanismRatio configured, so getPosition() already returns mechanism
     * rotations — meaning the raw rotor value is the mechanism angle multiplied by the gear ratio.
     */
    @Override
    public void simulationPeriodic() {
        m_leftPivotMotor.getSimState().setRawRotorPosition(
            m_pivotTargetPosition * Constants.GroundIntakeConstants.PIVOT_GEAR_RATIO);
    }
}
