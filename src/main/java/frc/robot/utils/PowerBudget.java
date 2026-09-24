package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.Constants.PowerConstants;

/**
 * Decides what the robot gives up when the battery starts to fold, so that the flywheel and the
 * turret never have to.
 *
 * <p>The RoboRIO already has brownout protection, and the problem with it is that it has no
 * opinion about what matters: at 6.8 V it cuts PWM outputs across the board, which on this robot
 * means a flywheel dropping mid-shot and a turret drifting off its solution, and every ball fired
 * in the second after that is wasted. This watches the same battery a long way further up and
 * spends the margin deliberately instead — slowing the intake first, then the drivetrain, and
 * leaving the shooter alone at every tier.
 *
 * <p>Three tiers, entered on a low-passed battery voltage with hysteresis and a minimum dwell.
 * Hysteresis and dwell are both load-bearing: cutting the intake raises the voltage, which is
 * exactly the condition for restoring the intake, and without them that loop runs at 50 Hz.
 *
 * <p>Static because the thing it describes is singular — there is one battery — and because the
 * consumers (the drivetrain's request lambda, the intake's outputs) are in places where threading
 * an instance through buys nothing. {@link #update()} must be called once per loop from
 * {@code Robot.robotPeriodic()}; until it is, everything here reports NORMAL and changes nothing.
 */
public final class PowerBudget {

    /**
     * Operator kill switch, live during a match and NOT gated behind tuning mode. Off pins the
     * robot at {@link Tier#NORMAL} — full authority everywhere, exactly as if none of this existed.
     */
    public static final DashboardToggle ENABLED =
        new DashboardToggle("Power/BrownoutProtectionEnabled", true);

    // ---- Live tuning, under "Tuning/Power/" ----
    // Every threshold and every scale, so a practice field can find the numbers that match this
    // robot's real battery behaviour without a redeploy between attempts.
    //
    // Gated behind TuningMode like every other TunableDouble, which is the usual gotcha: a value
    // typed here does nothing until Tuning/TuningModeEnabled is on, and reverts to the compiled
    // constant the moment it goes off. That is deliberate — see TunableDouble — and it is why the
    // kill switch above is a DashboardToggle instead, since turning protection off is a decision
    // that has to work mid-match.
    private static final TunableDouble s_reducedEnterVolts = new TunableDouble(
        "Tuning/Power/ReducedEnterVolts", PowerConstants.REDUCED_ENTER_VOLTS);
    private static final TunableDouble s_reducedExitVolts = new TunableDouble(
        "Tuning/Power/ReducedExitVolts", PowerConstants.REDUCED_EXIT_VOLTS);
    private static final TunableDouble s_criticalEnterVolts = new TunableDouble(
        "Tuning/Power/CriticalEnterVolts", PowerConstants.CRITICAL_ENTER_VOLTS);
    private static final TunableDouble s_criticalExitVolts = new TunableDouble(
        "Tuning/Power/CriticalExitVolts", PowerConstants.CRITICAL_EXIT_VOLTS);

    private static final TunableDouble s_filterTauSeconds = new TunableDouble(
        "Tuning/Power/VoltageFilterTauSec", PowerConstants.VOLTAGE_FILTER_TAU_SECONDS);
    private static final TunableDouble s_minTierHoldSeconds = new TunableDouble(
        "Tuning/Power/MinTierHoldSec", PowerConstants.MIN_TIER_HOLD_SECONDS);

    private static final TunableDouble s_reducedDriveScale = new TunableDouble(
        "Tuning/Power/ReducedDriveScale", PowerConstants.REDUCED_DRIVE_SCALE);
    private static final TunableDouble s_criticalDriveScale = new TunableDouble(
        "Tuning/Power/CriticalDriveScale", PowerConstants.CRITICAL_DRIVE_SCALE);
    private static final TunableDouble s_reducedIntakeScale = new TunableDouble(
        "Tuning/Power/ReducedIntakeScale", PowerConstants.REDUCED_INTAKE_SCALE);
    private static final TunableDouble s_criticalIntakeScale = new TunableDouble(
        "Tuning/Power/CriticalIntakeScale", PowerConstants.CRITICAL_INTAKE_SCALE);

    private static final TunableDouble s_reducedSupplyAmps = new TunableDouble(
        "Tuning/Power/ReducedIntakeSupplyAmps", PowerConstants.REDUCED_INTAKE_SUPPLY_AMPS);
    private static final TunableDouble s_reducedStatorAmps = new TunableDouble(
        "Tuning/Power/ReducedIntakeStatorAmps", PowerConstants.REDUCED_INTAKE_STATOR_AMPS);
    private static final TunableDouble s_criticalSupplyAmps = new TunableDouble(
        "Tuning/Power/CriticalIntakeSupplyAmps", PowerConstants.CRITICAL_INTAKE_SUPPLY_AMPS);
    private static final TunableDouble s_criticalStatorAmps = new TunableDouble(
        "Tuning/Power/CriticalIntakeStatorAmps", PowerConstants.CRITICAL_INTAKE_STATOR_AMPS);

    /** How hard the battery is being protected right now. */
    public enum Tier {
        /** Everything at full authority. */
        NORMAL,
        /** Intake rollers and drive speed cut back; shooter untouched. */
        REDUCED,
        /** Intake stopped outright and drive cut hard; shooter still untouched. */
        CRITICAL
    }

    private static Tier s_tier = Tier.NORMAL;
    private static double s_filteredVolts = 12.0;
    private static double s_tierEnteredTime = Double.NEGATIVE_INFINITY;
    private static double s_lastUpdateTime = -1.0;

    private PowerBudget() {}

    /** Samples the battery and updates the tier. Call once per loop, before the scheduler runs. */
    public static void update() {
        double now = Timer.getFPGATimestamp();
        double dt = s_lastUpdateTime < 0.0 ? 0.02 : Math.max(1e-3, now - s_lastUpdateTime);
        s_lastUpdateTime = now;

        double volts = RobotController.getBatteryVoltage();
        double alpha = dt / (Math.max(0.0, s_filterTauSeconds.get()) + dt);
        s_filteredVolts += alpha * (volts - s_filteredVolts);

        // A disabled robot draws nothing, so its voltage says nothing useful about what the next
        // enable can afford — and a tier latched from the last match must not follow the robot into
        // the next one. Reset rather than decay: the filter reconverges in a tenth of a second.
        if (DriverStation.isDisabled() || !ENABLED.get()) {
            if (DriverStation.isDisabled()) {
                s_filteredVolts = volts;
            }
            setTier(Tier.NORMAL, now);
            publish(volts);
            return;
        }

        if (now - s_tierEnteredTime >= s_minTierHoldSeconds.get()) {
            setTier(nextTier(s_filteredVolts), now);
        }

        publish(volts);
    }

    /**
     * The tier the current voltage calls for, given the one already in force. Entry thresholds are
     * below exit thresholds, so each tier has to be clearly left rather than merely brushed.
     */
    private static Tier nextTier(double volts) {
        if (volts <= s_criticalEnterVolts.get()) {
            return Tier.CRITICAL;
        }
        if (volts <= s_reducedEnterVolts.get()) {
            return Tier.REDUCED;
        }
        return switch (s_tier) {
            case CRITICAL -> volts >= s_criticalExitVolts.get() ? Tier.REDUCED : Tier.CRITICAL;
            case REDUCED  -> volts >= s_reducedExitVolts.get()  ? Tier.NORMAL  : Tier.REDUCED;
            case NORMAL   -> Tier.NORMAL;
        };
    }

    private static void setTier(Tier tier, double now) {
        if (tier != s_tier) {
            s_tier = tier;
            s_tierEnteredTime = now;
        }
    }

    /** The tier currently in force. */
    public static Tier tier() {
        return s_tier;
    }

    /** True whenever anything is being held back. */
    public static boolean isLimiting() {
        return s_tier != Tier.NORMAL;
    }

    /** Low-passed battery voltage the tier decision is made on. */
    public static double filteredVolts() {
        return s_filteredVolts;
    }

    /**
     * Multiplier on the driver's commanded drive speed, 0 to 1.
     *
     * <p>Teleop only by construction — it is applied where the driver's stick is read, so
     * autonomous paths are untouched. That is on purpose: a path follower that is silently speed
     * limited does not drive its path slower, it drives a different path.
     */
    public static double driveOutputScale() {
        return switch (s_tier) {
            case NORMAL   -> PowerConstants.NORMAL_DRIVE_SCALE;
            case REDUCED  -> clampScale(s_reducedDriveScale.get());
            case CRITICAL -> clampScale(s_criticalDriveScale.get());
        };
    }

    /** Keeps a fat-fingered dashboard scale from inverting a motor or doubling its authority. */
    private static double clampScale(double scale) {
        return Math.min(1.0, Math.max(0.0, scale));
    }

    /**
     * Multiplier on the ground intake's roller and indexer outputs, 0 to 1. Zero at
     * {@link Tier#CRITICAL} — collecting fuel is worth nothing if the robot cannot shoot it.
     *
     * <p>Applies to the rollers and the intake's own indexers only. The intake pivot is not scaled
     * (it has to hold position against gravity), and neither is the turret's feeder, which is part
     * of the shooter as far as this is concerned.
     */
    public static double intakeOutputScale() {
        return switch (s_tier) {
            case NORMAL   -> PowerConstants.NORMAL_INTAKE_SCALE;
            case REDUCED  -> clampScale(s_reducedIntakeScale.get());
            case CRITICAL -> clampScale(s_criticalIntakeScale.get());
        };
    }

    /**
     * Supply current limit to force on the intake's rollers and indexers, in amps, or
     * {@link Double#NaN} at {@link Tier#NORMAL} to mean "put each motor's own configured limit
     * back". Output scaling alone does not cover a stalled roller, which will draw whatever the
     * stator limit allows however gently it is commanded.
     */
    public static double intakeSupplyCurrentLimitAmps() {
        return switch (s_tier) {
            case NORMAL   -> Double.NaN;
            case REDUCED  -> s_reducedSupplyAmps.get();
            case CRITICAL -> s_criticalSupplyAmps.get();
        };
    }

    /** Stator current limit for the intake's rollers and indexers — see {@link #intakeSupplyCurrentLimitAmps()}. */
    public static double intakeStatorCurrentLimitAmps() {
        return switch (s_tier) {
            case NORMAL   -> Double.NaN;
            case REDUCED  -> s_reducedStatorAmps.get();
            case CRITICAL -> s_criticalStatorAmps.get();
        };
    }

    private static void publish(double rawVolts) {
        SmartDashboard.putNumber("Power/BatteryVolts", rawVolts);
        SmartDashboard.putNumber("Power/FilteredVolts", s_filteredVolts);
        SmartDashboard.putString("Power/Tier", s_tier.name());
        SmartDashboard.putNumber("Power/DriveScale", driveOutputScale());
        SmartDashboard.putNumber("Power/IntakeScale", intakeOutputScale());
        // The RIO's own protection, for reference. If this ever goes true the tiers above were
        // either too slow or too generous.
        SmartDashboard.putBoolean("Power/RioBrownedOut", RobotController.isBrownedOut());
    }
}
