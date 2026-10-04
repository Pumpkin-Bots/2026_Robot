// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {

    public static final class DriveConstants {
        // Normal driving speeds (100% of max)
        public static final double NORMAL_MAX_SPEED_MULTIPLIER = 1.0;
        public static final double NORMAL_MAX_ANGULAR_RATE_MULTIPLIER = 1.0;

        // Shooter mode speeds (reduced for precise positioning)
        public static final double SHOOTER_MODE_MAX_SPEED_MULTIPLIER = 0.25; // 25% of max speed
        public static final double SHOOTER_MODE_MAX_ANGULAR_RATE_MULTIPLIER = 0.5; // 50% of max rotation speed

        // Boost mode speeds — full authority, for sprinting between fuel piles while the shooter
        // is parked. Separate from NORMAL_* so the two can be tuned apart later.
        public static final double BOOST_MAX_SPEED_MULTIPLIER = 1.0;
        public static final double BOOST_MAX_ANGULAR_RATE_MULTIPLIER = 1.0;

        // How far an analog trigger must be pressed before its mode engages.
        public static final double MODE_TRIGGER_THRESHOLD = 0.2;
    }

    /**
     * Field geometry for the 2026 REBUILT field, used to decide what the shooter should be doing
     * from where it is standing. All coordinates are WPILib blue-origin field coordinates: X runs
     * from the blue alliance wall (0) to the red alliance wall, Y runs from the scoring-table side
     * (0) to the far side.
     *
     * <p>Sources for the numbers below: field length/width and the hub/trench/tower stations come
     * from the 2026 AprilTag layout ({@code 2026-rebuilt-welded.json}); structure sizes come from
     * the game manual's arena chapter. Everything here is a plain constant precisely so it can be
     * re-measured and corrected on the field without touching command logic.
     */
    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = 16.541;
        public static final double FIELD_WIDTH_METERS  = 8.069;

        // ---- Alliance zones ----
        // Each ALLIANCE ZONE runs 158.6 in from its own alliance wall toward midfield; the NEUTRAL
        // ZONE is everything between them. 4.028 m lands exactly on the near face of the hub /
        // trench line, which is the boundary you can actually see on the carpet.
        public static final double ALLIANCE_ZONE_DEPTH_METERS = 4.028; // 158.6 in

        /** Largest X still inside the BLUE alliance zone. */
        public static final double BLUE_ALLIANCE_ZONE_MAX_X_METERS = ALLIANCE_ZONE_DEPTH_METERS;
        /** Smallest X still inside the RED alliance zone. */
        public static final double RED_ALLIANCE_ZONE_MIN_X_METERS =
            FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS;

        // ---- Trenches ----
        // A TRENCH sits against each long guardrail at the same field-length station as that
        // alliance's hub — four of them in total (two per alliance, one per side of the field).
        // The X centres below are the hub centres, taken from the AprilTag layout: the trench
        // AprilTags (17/22/23/28 blue, 1/6/7/12 red) sit on the same station line.
        public static final double BLUE_TRENCH_CENTER_X_METERS =  4.6255;
        public static final double RED_TRENCH_CENTER_X_METERS  = 11.9155;

        // Front-to-back depth of the trench structure along X (47 in per the manual).
        public static final double TRENCH_DEPTH_METERS = 1.194;

        // How far the drivable channel under the trench arm reaches in from the guardrail. The
        // manual gives 50.34 in of clearance under the arm; beyond that the structure is solid.
        public static final double TRENCH_CHANNEL_DEPTH_METERS = 1.279;

        // Padding added around the trench box before it counts as "in the trench". Grow this if
        // the shooter is still spinning up as the robot noses into the trench.
        public static final double TRENCH_MARGIN_METERS = 0.15;

        // ---- Towers ----
        // A TOWER is built into each alliance wall between driver stations 2 and 3, 49.25 in wide
        // and 45 in deep. The Y centres are the midpoints of that wall's tower AprilTag pairs
        // (31/32 blue, 15/16 red), which is why they are not exactly on the field centreline.
        public static final double TOWER_DEPTH_METERS = 1.143; // 45 in, measured off the wall
        public static final double TOWER_WIDTH_METERS = 1.251; // 49.25 in, along Y
        public static final double BLUE_TOWER_CENTER_Y_METERS = 3.962;
        public static final double RED_TOWER_CENTER_Y_METERS  = 4.107;

        // Padding around the tower box, same idea as TRENCH_MARGIN_METERS.
        public static final double TOWER_MARGIN_METERS = 0;

        // ---- Boundary hysteresis ----
        // Every zone test above is a hard edge, and a robot parked on one would otherwise flip
        // decisions every loop — swinging the turret between the hub and the shuttle aim point, or
        // strobing the flywheel on and off. Once a boundary has been crossed, the robot has to come
        // back this far past it before the decision flips again.
        public static final double ZONE_HYSTERESIS_METERS = 0.15;
    }

    public static final class ShooterConstants{
        public static final int TURRET_ROTATOR_ID = 25;
        public static final int SHOOTER_RACK_ID = 26;
        public static final int SHOOTER_FLYWHEEL_ID = 27;


        public static final double ROTATOR_KP = 4; // optimal is 12, but too violent, need stronger wiring chain.
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.25;
        // Velocity feedforward for turret omega tracking (V·s/rot, motor units).
        // Start at 0 (disabled), increment by ~0.05 until turret tracks robot rotation smoothly.
        public static final double ROTATOR_KV = 0.25;
        // Static friction feedforward, in volts, applied by Slot0 in whichever direction the closed
        // loop is driving — symmetric, unlike ROTATOR_KFF below. Tunable live as Tuning/Turret/kS.
        // Start at 0 and raise until the turret just begins to break away from a small error.
        public static final double ROTATOR_KS = 0.65;

        // One-directional static feedforward, in volts, for the direction the wiring chain spools
        // against. Only that direction gets the assist; the other gets nothing, since it is not the
        // one fighting the chain. Positive assists CCW (increasing turret angle), negative assists
        // CW — so the sign picks the direction and the magnitude is the voltage.
        // Tunable live as Tuning/Turret/kFF. Start at 0 and raise until the lag closes.
        public static final double ROTATOR_KFF = 0;
        // Below this commanded turret rate the term is held off, so it does not creep the turret
        // while it is trying to sit still. Deg/s of turret travel.
        public static final double ROTATOR_KFF_DEADBAND_DEG_PER_SEC = 0;

        public static final double TURRET_ROTATOR_GEAR_RATIO = -20 / 200.0;
        public static final double TURRET_ROTATOR_MIN_ANGLE = -200;
        public static final double TURRET_ROTATOR_MAX_ANGLE = 300;

        // How close the turret has to get to its commanded angle before a wrap counts as finished.
        // Only consulted while the turret is unwrapping — see ShooterSubsystem.consumeTurretWrap()
        // and the reset gate in ShooterMode. Plain tracking error, however large, never engages
        // that gate, so this is not a "the shot is good enough" tolerance and shooting is not
        // otherwise held off on it.
        public static final double TURRET_RESET_TOLERANCE_DEG = 10.0;


        public static final double RACK_KP = 20;
        public static final double RACK_KI = 0.0;
        public static final double RACK_KD = 0.25;

        public static final double RACK_GEAR_RATIO = -1.0 / 250;
        public static final double RACK_MIN_ANGLE = 18; // 15 deg
        public static final double RACK_MAX_ANGLE = 38; // 45 deg
        // Rack gear ratio is backwards
        // MAX Rotations is at maximum height (lower shot)
        // MIN Rotations is at minimum height (higher shot)

        // ---- Flywheel velocity loop ----
        // kV carries the steady-state voltage, kP rejects what is left. All five are live-tunable
        // under "Tuning/Flywheel/" — a stiffer loop is the first half of fixing a wheel that sags
        // when a ball goes through it (the second half is FLYWHEEL_DROOP_* below).
        //
        // kA is deliberately NOT a constant here: the voltage it takes to accelerate the wheel is a
        // property of the wheel's inertia and the motor's winding resistance, not a number worth
        // guessing at, so it is computed from FLYWHEEL_MOI_KG_M2 — see
        // ShooterSubsystem.flywheelKaFromMoi(). It only does anything because the flywheel request
        // is given the setpoint's own rate of change as its acceleration, so a shot that is walking
        // the setpoint up as the robot backs away gets the extra volts immediately instead of
        // waiting for kP to notice the wheel falling behind.
        public static final double FLYWHEEL_KP = 0.4;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;
        public static final double FLYWHEEL_KV = 0.125;
        // Static friction, in volts. Raise until a wheel commanded to a very low speed just breaks
        // away instead of sitting still.
        public static final double FLYWHEEL_KS = 0.0;

        // Rotational inertia of the spinning assembly about the LARGE (4 in) flywheel's shaft, in
        // kg·m² — the large wheel itself plus the small wheel and the rotor reflected through their
        // gearing. Get it from CAD; it is the one number that decides both how far the wheel sags
        // when a ball passes through it and how hard the motor has to work to put that speed back,
        // and doubling it roughly halves the sag. A 4 in, 1 kg wheel on its own is around 0.0013.
        //
        // Used in four places: the kA above, the peak acceleration the motor can produce, the
        // simulation's flywheel model, and the physics seed for the droop compensator — which is
        // what the robot shoots on before it has measured a single shot of its own. The three that
        // work in motor units use FLYWHEEL_ROTOR_MOI_KG_M2, the reflected form of this.
        // TODO: measure from CAD.
        public static final double FLYWHEEL_MOI_KG_M2 = 0.0013;

        // Mass of one fuel game piece, kg. Only used for the droop seed above.
        // TODO: weigh one.
        public static final double FUEL_MASS_KG = 0.145;

        // Of the energy the wheel gives up to a passing ball, the fraction that ends up as ball
        // kinetic energy. The rest goes into compressing the ball, scrubbing it against the hood,
        // and heat. Lower means the wheel loses MORE speed per ball than the ball's energy alone
        // would suggest, so the seeded droop estimate gets bigger.
        public static final double FLYWHEEL_SHOT_ENERGY_EFFICIENCY = 0.5;

        // ---- Gearing ----
        // MOTOR rotations per rotation of the LARGE (4 in) flywheel. Greater than 1 is a reduction:
        // 28/18 trades top speed for torque, which is what keeps the wheel from being dragged down
        // as far by each ball in the first place.
        //
        // Until this was wired into FLYWHEEL_EFFECTIVE_DIAMETER_METERS below, this constant reached
        // nothing but the simulation — every speed on the real robot was converted straight from
        // motor RPS through a hand-entered diameter, so changing it did nothing. It is load-bearing
        // now, and getting it wrong scales every shot in the match.
        public static final double FLYWHEEL_GEAR_RATIO = 28.0 / 18.0;

        // Rotations of the SMALL (2 in) flywheel per rotation of the LARGE (4 in) one. The two are
        // geared to each other, not just to the motor, and the difference in their surface speeds
        // is what puts backspin on the ball.
        public static final double FLYWHEEL_SMALL_PER_LARGE_RATIO = 18.0 / 28.0;

        public static final double FLYWHEEL_LARGE_DIAMETER_METERS = 0.1016; // 4 inches
        public static final double FLYWHEEL_SMALL_DIAMETER_METERS = 0.0508; // 2 inches
        // Ceiling on the commanded flywheel speed, in MOTOR RPS. 80 is 80% of a Kraken X60's 100
        // RPS free speed, which is the real constraint now — the motor cannot hold much past this
        // under load whatever the number here says.
        //
        // This used to be sized off the longest shot the robot is asked to make: a shuttle pass
        // from the far side of the opposing alliance zone, turret ~0.46 m off their back wall,
        // driving parallel to that wall at the drivetrain's full 5.44 m/s. That worst case wanted
        // 69.3 motor RPS when the flywheel was direct-drive, and 80 left comfortable margin.
        //
        // The 28:18 reduction changed that. The same shot now needs roughly 1.77x the motor speed —
        // past the motor's free speed, never mind this ceiling — so the longest shuttle passes are
        // no longer physically available and the solver will clamp them. That is reported, not
        // hidden: watch Shooter/Physics/SpeedClamped and Shooter/Physics/Achievable. Shots inside
        // our own alliance zone are unaffected; they were nowhere near the ceiling before and are
        // still well under it.
        public static final double FLYWHEEL_MAX_REV_PER_SEC = 90.0;
        /**
         * Effective flywheel diameter, in meters, referenced to the MOTOR: the whole of
         * {@code launchSpeed = π · d · motorRps}. Every speed conversion in the project — the aim
         * solver, the simulated projectile, the droop seed — goes through this one number, which is
         * why it carries both gear ratios rather than being a bare wheel size.
         *
         * <p>The ball is squeezed between two wheels of different sizes turning at different
         * speeds, and its centre leaves at the average of the two contact surface speeds (the
         * difference between them is the backspin). Per rotation of the large wheel that is
         * {@code (d_large + d_small · smallPerLarge) / 2}, and dividing by the motor reduction
         * re-references it to motor rotations.
         *
         * <p>This used to be the flat average of the two diameters, 0.0762, which silently assumed
         * both wheels turned at the same speed as the motor. Both assumptions are now false, and
         * the number is a third smaller as a result.
         *
         * <p><b>Consequence of the 28:18 reduction:</b> ball speed per motor rotation dropped by
         * about 44%, so every entry in {@code flywheelRPSTable} and every shot the physics solver
         * produces now asks for proportionally more motor RPS. Expect long shuttle passes to run
         * into {@code FLYWHEEL_MAX_REV_PER_SEC} — the solver reports that honestly as
         * {@code Shooter/Physics/SpeedClamped}, so watch that row before trusting a long pass.
         */
        public static final double FLYWHEEL_EFFECTIVE_DIAMETER_METERS =
            (FLYWHEEL_LARGE_DIAMETER_METERS
                + FLYWHEEL_SMALL_DIAMETER_METERS * FLYWHEEL_SMALL_PER_LARGE_RATIO)
            / 2.0 / FLYWHEEL_GEAR_RATIO;

        /**
         * The flywheel's inertia as the MOTOR ROTOR feels it, kg·m² — {@code J / G²}.
         *
         * <p>A reduction makes the wheel look lighter from the rotor's side by the square of the
         * ratio, and everything computed against the motor's own velocity signal — kA, the peak
         * acceleration the motor can produce, the droop seed — needs the reflected figure rather
         * than the raw one.
         */
        public static final double FLYWHEEL_ROTOR_MOI_KG_M2 =
            FLYWHEEL_MOI_KG_M2 / (FLYWHEEL_GEAR_RATIO * FLYWHEEL_GEAR_RATIO);

        // ---- Shot droop compensation ----
        // A ball passing through the flywheel takes energy out of it, so the wheel is slower at the
        // moment the ball separates than it was a few milliseconds earlier — the ball leaves slower
        // than the aim solution asked for, and during sustained fire the wheel never fully recovers
        // between balls. FlywheelDroopCompensator watches the velocity signal, measures how far the
        // wheel actually sags on each shot, learns the average over the match, and biases the
        // commanded speed up so what the ball sees is what was asked for.
        //
        // Everything below is live-tunable under "Tuning/Flywheel/", and everything the compensator
        // has measured is published under "Shooter/Flywheel/".

        // What is learned is the TROUGH DEFICIT: how far below the commanded setpoint the wheel has
        // sagged at the instant the ball separates from it. That is the number that decides how
        // fast the ball actually leaves, and biasing the command up by exactly it puts the trough
        // on target. It is learned per commanded-speed bin rather than as a single figure, because
        // the net dip is badly non-linear in speed — the motor's ability to push back during the
        // contact collapses as it approaches free speed, so near the bottom of the range it
        // replaces nearly everything the ball takes and near the top almost none of it.
        //
        // One deficit subsumes both effects that used to be handled separately: a single ball's
        // drop, and the deeper trough of the fifth ball in a burst that the wheel never recovered
        // from. Both are just "how far down was it when the ball left".
        //
        // How much of the learned deficit to add back, 0 to 1:
        //   0.0 — off. The compensator still measures, learns, and publishes; it just doesn't
        //         change the shot. This is the setting to re-take the lookup tables under.
        //   1.0 — the default, and the physically correct one: the ball leaves at the wheel's speed
        //         at separation, so pre-biasing by the full deficit puts separation on target.
        // Back it off only if shots start going long, which would mean the measured trough is
        // deeper than the speed the ball really left at.
        public static final double FLYWHEEL_COMPENSATION_GAIN = 1.0;

        // How far the tracking error has to climb back off a trough before that trough is confirmed
        // and counted as one ball, in motor RPS. THIS IS THE ONE THAT SEPARATES ONE BALL FROM TWO.
        //
        // During rapid fire the wheel climbs only part of the way back between balls — a clear step
        // up, nowhere near where it started. Waiting for a full return to baseline would read a
        // whole burst as one long sag and learn nothing from the busiest part of the match; waiting
        // for a reversal counts each ball on its own.
        //
        // Too high and consecutive balls merge into one (symptom: ShotCount lags balls fired, and
        // LastPerBallDropRps reads implausibly deep). Too low and velocity noise splits one ball
        // into several (symptom: ShotCount runs ahead of balls fired, LastPerBallDropRps shallow).
        // Should sit below FLYWHEEL_SHOT_DETECT_DROP_RPS.
        public static final double FLYWHEEL_SHOT_REBOUND_RPS = 1.0;

        // How much each newly measured ball moves its bin's average, 0 to 1. 0.20 means a ball is
        // worth a fifth of the estimate, so a bin settles in a handful of balls but a single weird
        // reading cannot run away with it. The first ball into an empty bin is taken whole.
        public static final double FLYWHEEL_DROOP_LEARNING_RATE = 0.20;

        // Hard ceiling on the total bias the compensator may add, in motor RPS. This is the guard
        // that keeps a mis-detection, or a wheel that is voltage-saturated and can never reach its
        // setpoint, from winding the commanded speed up indefinitely.
        public static final double FLYWHEEL_MAX_COMPENSATION_RPS = 20.0;

        // How far below the running baseline the velocity has to dip before it counts as a ball
        // going through, in motor RPS. Too low and encoder noise registers as shots; too high and
        // real shots are missed. Watch "Shooter/Flywheel/LastPerBallDropRps" against a known number of
        // balls fired to set it.
        public static final double FLYWHEEL_SHOT_DETECT_DROP_RPS = 1.5;

        // A measured droop larger than this is thrown away rather than learned from — that is a
        // stall, a jam, or the flywheel being commanded somewhere new, not a ball.
        public static final double FLYWHEEL_MAX_PLAUSIBLE_DROOP_RPS = 25.0;

        // How close the wheel has to be to its setpoint to count as up to speed, in motor RPS.
        // Published as "Shooter/Flywheel/AtSpeed", and the detector will not look for shots until
        // the wheel has reached speed at least once, so spin-up is never mistaken for a shot.
        public static final double FLYWHEEL_AT_SPEED_TOLERANCE_RPS = 1.0;

        // Below this commanded speed the compensator does nothing at all — the wheel is parked or
        // on its way there, and neither is a state where a shot can be measured.
        public static final double FLYWHEEL_MIN_DETECT_RPS = 10.0;

        // The aim solution moves the setpoint continuously as the robot drives, and a setpoint
        // stepping upward looks exactly like a shot from the velocity signal's point of view. Any
        // shot detected while the setpoint is slewing faster than this (motor RPS per second) is
        // discarded. Ordinary aim tracking moves it by a few RPS/s.
        public static final double FLYWHEEL_MAX_SETPOINT_SLEW_RPS_PER_SEC = 25.0;

        // Ball launch position relative to robot center
        // X: forward offset (meters, positive = toward robot front)
        // Y: lateral offset (meters, positive = toward robot left)
        // Z: height above floor (meters)
        // TODO: measure from CAD or physical robot
        public static final double BALL_LAUNCH_FRONT_OFFSET_METERS = -0.114;
        public static final double BALL_LAUNCH_LATERAL_OFFSET_METERS = 0.0;
        public static final double BALL_LAUNCH_HEIGHT_METERS = 0.4826;

        // ---- Physics aiming calibration ----
        // Every value below is exposed live on SmartDashboard under "Tuning/Shooter/..." (see
        // ShooterTuning). Tune on the dashboard, then copy the winning number back here so it
        // survives a reboot. Suggested tuning order is 1 → 6.

        // (0) Gravity used by the ballistic solve. maple-sim's projectiles use a flat 11.0 m/s^2
        // instead of 9.81 to fake air drag, so sim and the real robot want different values here.
        // ShooterTuning picks the right default automatically; override only if you know why.
        public static final double PHYSICS_GRAVITY_SIM_MPS2  = 11.0;
        public static final double PHYSICS_GRAVITY_REAL_MPS2 = 9.80665;

        // (1) How much steeper than the minimum-energy angle to aim. The minimum-energy angle
        // reaches the target exactly at the apex of its arc, which at close range can arrive on
        // the way UP and skim the rim. Biasing steeper guarantees the ball is descending on
        // arrival. Raise if shots ride the rim, lower if they drop short and steep.
        public static final double DESCENT_MARGIN_DEG = 8.0;

        // (2) Mechanical zero calibration. Pure command offsets applied AFTER the physics solve —
        // these correct "the rack reads 20 deg but is physically at 22 deg", not the physics.
        // RACK: positive = flatter shot (rack angle up). TURRET: positive = counter-clockwise.
        public static final double RACK_ANGLE_OFFSET_DEG   = 0.0;
        public static final double TURRET_ANGLE_OFFSET_DEG = 0.0;

        // (3) Speed calibration. The no-drag solve always UNDER-predicts the speed a real ball
        // needs, and the shortfall grows with range, so there are two knobs:
        //   SPEED_SCALAR      — flat multiplier on required launch speed. Fixes "every shot is
        //                       short/long by the same fraction". Start here.
        //   SPEED_PER_METER   — extra m/s added per meter of distance. Fixes "close shots are
        //                       right but long shots fall short" (that's air drag).
        public static final double SPEED_SCALAR_DEFAULT    = 1.0;
        public static final double SPEED_PER_METER_DEFAULT = 0;

        // (4) Final flywheel trim in motor RPS, applied after the speed→RPS conversion. Use this
        // for a small constant bias (e.g. ball compression losses at the exit roller) rather than
        // distorting FLYWHEEL_EFFECTIVE_DIAMETER_METERS, which also affects the sim projectile.
        public static final double FLYWHEEL_RPS_OFFSET_DEFAULT = -3;

        // (5) Shoot-on-the-move authority, 0 to 1. 1.0 = fully compensate for robot velocity,
        // 0.0 = ignore it entirely (aim as if stopped). Set to 0 to isolate a stationary aiming
        // problem from a motion-compensation problem, then walk it back up.
        public static final double SHOOT_ON_THE_MOVE_GAIN = 1.0;

        // (5b) Which joint pays for that compensation. On — the default — the solver picks the arc
        // whose required launch speed is the speed the flywheel would be held at standing in the
        // same spot, so the rack swings and the flywheel command is left tracking distance alone.
        // That is worth doing because the rack moves a few degrees in a fraction of the time a
        // loaded flywheel takes to move a few RPS, and the correction changes as fast as the
        // driver's sticks. Off, the arc is fixed first and the flywheel absorbs everything.
        //
        // The correction is only ever taken by STEEPENING the arc — flattening is the mirror trade
        // and the geometry makes it nearly worthless, see AimSolver — so driving away from the
        // target the flywheel still has to spin up, and Shooter/Physics/MotionRackSaturated says
        // when that is the case.
        //
        // The ceiling bounds how far the arc may be steepened past the policy's angle, in degrees
        // of launch elevation. What it protects is hang time: a much steeper shot is in the air
        // longer, which is longer for the robot's own velocity estimate to have been wrong by. 20
        // degrees is wider than the rack's whole travel, i.e. effectively off; lower it if moving
        // shots start arriving late and scattered while the stationary ones are still good.
        public static final boolean MOTION_RACK_FIRST = true;
        public static final double MOTION_RACK_MAX_SWING_DEG = 20.0;

        // (6) Tilt compensation. The turret yaws about the chassis's vertical axis and the rack
        // elevates from the chassis's plane, so a robot with a wheel up on the depot or a corner on
        // the bump is aiming in a frame that is tipped over relative to the field. The solver
        // corrects for that by rotating its answer through the gyro's pitch and roll — see
        // AimSolver. A degree of uncorrected tilt is roughly a degree of aiming error, which is
        // about 9 cm of miss at 5 m.
        //
        // Authority, 0 to 1, exactly like SHOOT_ON_THE_MOVE_GAIN above: 1.0 fully compensates,
        // 0.0 aims as if the robot were level. Set it to 0 to isolate a tilt-compensation problem
        // from an aiming problem.
        public static final double TILT_COMPENSATION_GAIN = 1.0;

        // Mounting calibration for the gyro's pitch and roll, in degrees, subtracted from what it
        // reports. A Pigeon bolted down a degree out of plane reads a degree of tilt on a robot
        // that is sitting perfectly flat, and the compensation would dutifully aim a degree wrong
        // all match — worse than not compensating at all. Park the robot on flat carpet, read
        // Shooter/Tilt/RollDeg and Shooter/Tilt/PitchDeg, and put those numbers here.
        public static final double TILT_ROLL_OFFSET_DEG  = 0.0;
        public static final double TILT_PITCH_OFFSET_DEG = 0.0;

        // Ceiling on how much tilt will be compensated for, in degrees, per axis. Well past
        // anything the robot can drive over and stay upright, so it never limits a real shot — it
        // is here so that a gyro fault reading a wild angle swings the turret by a bounded amount
        // instead of sending it to the far stop.
        public static final double TILT_MAX_COMPENSATED_DEG = 20.0;

        // Field-relative 3D position of the shooting target (AprilTag 26)
        private static final Pose3d TAG_26_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(26)
            .orElseThrow();
        public static final double BLUE_TARGET_X_METERS = TAG_26_POSE.getX() + 0.597;
        public static final double BLUE_TARGET_Y_METERS = TAG_26_POSE.getY() + 0;
        // Lowered from +0.610 — maple-sim's RebuiltHub scores fuel between z=1.5748m and
        // z=1.8288m (a 10 in tall zone starting at the hub's own position), so +0.610 (aiming
        // near the top of that zone) was causing shots to overshoot, worse at longer range.
        // +0.45 aims near the low edge of the real scoring zone instead.
        public static final double BLUE_TARGET_Z_METERS = TAG_26_POSE.getZ() + 0.45;

        // Field-relative 3D position of the red side shooting target (AprilTag 10)
        private static final Pose3d TAG_10_POSE = VisionConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(10)
            .orElseThrow();
        public static final double RED_TARGET_X_METERS = TAG_10_POSE.getX() - 0.597;
        public static final double RED_TARGET_Y_METERS = TAG_10_POSE.getY() - 0;
        public static final double RED_TARGET_Z_METERS = TAG_10_POSE.getZ() + 0.45;


        // Shuttle passes are thrown to the carpet, not into the hub, so the aim point is the floor.
        // The physics solver takes a real z and solves for where the ball actually comes down;
        // borrowing the tag's height here (as this used to) told it to land the ball 1.2 m in the
        // air, which lands every pass short of where it was aimed.
        public static final double SHUTTLE_TARGET_Z_METERS = 1;

        // How far to the side of the hub a shuttle pass lands. The pass is aimed to whichever side
        // of the hub the shooter is already on, so the ball stays off the hub structure and comes
        // down where a teammate on that side of the field can pick it up.
        public static final double SHUTTLE_SIDE_OFFSET_METERS = 2.0;

        // Rack angle held in storage/boost mode. RACK_MIN_ANGLE is the rack's lowest physical
        // position (0 rack rotations), which is where it has to be to fit under a trench arm.
        public static final double RACK_STORAGE_ANGLE_DEG = RACK_MIN_ANGLE;

        // How far in front of the hub tag the pass lands, toward midfield. This used to sit 0.25 m
        // *behind* the tag; moving it 1 m toward midfield takes a meter off every shuttle shot,
        // which is a meter further back the robot can be standing when it takes one. The sideways
        // offset applied in ShuttleMode keeps the ball clear of the hub structure itself, so
        // landing level with the hub does not mean landing on it.
        private static final double SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS = -0.75;

        public static final double BLUE_SHUTTLE_TARGET_X_METERS =
            TAG_26_POSE.getX() + SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS;
        public static final double BLUE_SHUTTLE_TARGET_Y_METERS = TAG_26_POSE.getY();
        public static final double BLUE_SHUTTLE_TARGET_Z_METERS = SHUTTLE_TARGET_Z_METERS;

        // Red mirrors it: midfield is the other direction from the red hub.
        public static final double RED_SHUTTLE_TARGET_X_METERS =
            TAG_10_POSE.getX() - SHUTTLE_TARGET_MIDFIELD_OFFSET_METERS;
        public static final double RED_SHUTTLE_TARGET_Y_METERS = TAG_10_POSE.getY();
        public static final double RED_SHUTTLE_TARGET_Z_METERS = SHUTTLE_TARGET_Z_METERS;

        // Translation3d constants for easy use in commands
        public static final Translation3d BLUE_TARGET_POSITION = new Translation3d(
            BLUE_TARGET_X_METERS,
            BLUE_TARGET_Y_METERS,
            BLUE_TARGET_Z_METERS);
        public static final Translation3d RED_TARGET_POSITION = new Translation3d(
            RED_TARGET_X_METERS,
            RED_TARGET_Y_METERS,
            RED_TARGET_Z_METERS);
        public static final Translation3d BLUE_SHUTTLE_TARGET_POSITION = new Translation3d(
            BLUE_SHUTTLE_TARGET_X_METERS,
            BLUE_SHUTTLE_TARGET_Y_METERS,
            BLUE_SHUTTLE_TARGET_Z_METERS);
        public static final Translation3d RED_SHUTTLE_TARGET_POSITION = new Translation3d(
            RED_SHUTTLE_TARGET_X_METERS,
            RED_SHUTTLE_TARGET_Y_METERS,
            RED_SHUTTLE_TARGET_Z_METERS);

    }

    /**
     * Tuning for the fused field-relative velocity estimate used by shoot-on-the-move.
     *
     * <p>The Pigeon 2's accelerometer is the primary source: it responds instantly to a direction
     * change, where wheel odometry lags and lies during a slip. Its weakness is drift — integrating
     * acceleration accumulates error over a long straight. So wheel odometry is folded back in as a
     * slow correction, and the vision-corrected pose as an even slower one, which pins the estimate
     * down without giving up the IMU's fast response.
     */
    public static final class VelocityEstimatorConstants {
        public static final double GRAVITY_MPS2 = 9.80665;

        // How hard wheel odometry pulls the IMU-integrated velocity back, as a first-order time
        // constant in seconds. This is THE main knob.
        //   Larger (0.5+) = trust the IMU more: snappier response to direction changes, more drift.
        //   Smaller (0.1) = trust the wheels more: less drift, back toward plain wheel odometry.
        public static final double WHEEL_TRUST_TAU_SECONDS = 0.15;

        // Accelerometer bias learning rate, per second, as the gain of a first-order tracker.
        // Learning only ever runs while the robot is standing still, because that is the only time
        // the observation is clean: the true acceleration is known to be zero, so whatever the
        // accelerometer reports is bias by definition. 0.20 gives a ~5 s time constant, which the
        // robot has many times over during a match.
        // Set to 0 to disable bias learning entirely (pure complementary filter).
        //
        // This deliberately does NOT learn from the wheel-vs-IMU residual while driving. That
        // residual is dominated by transients — wheel slip, weight transfer tipping the
        // accelerometer so a little gravity leaks into its horizontal axes — none of which are
        // bias, and integrating them is windup: the filter's steady-state velocity error is the
        // accelerometer error times WHEEL_TRUST_TAU_SECONDS, so a bias wound up during a hard stop
        // becomes a phantom velocity that lingers after the robot has stopped. Drift on a long
        // straight is already bounded by the wheel correction, which is what that term is for.
        public static final double ACCEL_BIAS_GAIN = 0.20;

        // Hard bound on the learned bias, m/s^2. A real MEMS accelerometer offset is a few tenths;
        // anything past this is a fault or a bad observation, and should not be allowed to steer
        // the integrator.
        public static final double ACCEL_BIAS_MAX_MPS2 = 1.0;

        // ---- Standstill detection (zero-velocity update) ----
        // Integrating an accelerometer has no way to discover on its own that the robot has stopped:
        // every scrap of error accumulated during the stop just sits in the estimate and washes out
        // over WHEEL_TRUST_TAU_SECONDS, which is a shooter still leading a target the robot is no
        // longer moving toward. Wheel odometry does know, and at a standstill it is exactly right —
        // a swerve's wheels cannot read zero while the chassis is still translating. So when the
        // wheels have read stopped for a moment, the estimate is snapped to zero outright.
        //
        // Below this wheel-derived chassis speed the robot counts as stopped. Above carpet noise,
        // well below any speed worth compensating a shot for.
        public static final double ZERO_VELOCITY_WHEEL_SPEED_MPS = 0.10;

        // How long the wheels must agree they are stopped before the estimate is snapped. This is
        // what keeps a genuine four-wheel skid — wheels braked to a halt while the robot is still
        // sliding — from being mistaken for a standstill.
        public static final double ZERO_VELOCITY_DWELL_SECONDS = 0.10;

        // Bias learning additionally requires the robot not be spinning. A robot pivoting in place
        // is standing still by the definition above (its center isn't going anywhere), but the
        // Pigeon is mounted off-center, so what it reports there is mostly centripetal acceleration
        // that the estimator has had to subtract off — not a clean look at the sensor's offset.
        public static final double ZERO_VELOCITY_YAW_RATE_RAD_PER_SEC = 0.20;

        // Vision correction: velocity derived by differentiating the vision-fused pose over
        // VISION_SAMPLE_WINDOW_SECONDS. Slow and noisy, but unbiased — it catches systematic wheel
        // odometry error (wrong wheel radius, carpet scrub) that the wheels alone cannot see.
        // Set VISION_TRUST_TAU_SECONDS very high to disable.
        public static final double VISION_TRUST_TAU_SECONDS     = 0.75;
        public static final double VISION_SAMPLE_WINDOW_SECONDS = 0.25;
        // Ignore a vision-derived sample this far off the current estimate — a vision pose jump
        // differentiates into a huge bogus velocity spike, and this rejects it.
        public static final double VISION_REJECT_THRESHOLD_MPS = 2.0;

        // Rotation of the Pigeon's accelerometer axes into robot axes, in degrees. Only needed if
        // the Pigeon is not mounted with its X axis pointing robot-forward. The proper fix is the
        // Pigeon 2 MountPose config in Tuner X; this is the quick field workaround.
        public static final double IMU_MOUNT_YAW_OFFSET_DEG = 0.0;

        // ---- Pigeon mounting position, relative to robot center (meters, robot frame) ----
        // X: positive toward the robot FRONT.  Y: positive toward the robot LEFT.
        //
        // This mattered not at all when the Pigeon was only a gyro: yaw rate is identical
        // everywhere on a rigid body. It matters a great deal now that its ACCELEROMETER is being
        // used, because an off-center point on a rotating robot is genuinely accelerating even when
        // the robot's center is not. Spinning in place at 8 rad/s with the Pigeon 0.2 m off center
        // makes it read ~13 m/s^2 of pure centripetal acceleration that the robot center never
        // experiences — integrate that and the velocity estimate is garbage the moment the robot
        // turns. The estimator subtracts both the centripetal (omega^2 * r) and tangential
        // (alpha * r) terms to recover the center's acceleration.
        //
        // Measure from CAD or by tape measure to the Pigeon chip itself. Leave both 0 only if the
        // Pigeon really is at the robot's rotational center.
        // TODO: measure on the real robot.
        public static final double PIGEON_OFFSET_FORWARD_METERS = -0.0127;
        public static final double PIGEON_OFFSET_LEFT_METERS    = -0.2921;

        // Smoothing time constant for the yaw acceleration (alpha) used by the tangential term.
        // Alpha comes from differentiating yaw rate, which is noisy, so it gets low-passed. Larger =
        // smoother but laggier. Only has any effect when the Pigeon offsets above are non-zero.
        public static final double YAW_ACCEL_FILTER_TAU_SECONDS = 0.04;

        // Hard sanity clamp on the fused estimate so a bad accelerometer can never run away.
        public static final double MAX_PLAUSIBLE_SPEED_MPS = 6.0;
    }

    public static final class TurretConstants {
        public static final int TURRET_INDEXER_ID = 28;

        public static final double TURRET_INDEXER_SPEED = -.65; // 60% before

        // ---- Feeder / indexer disturbance compensation ----
        // The feeder shoves the ball as it enters the turret, so the ball leaves carrying a little
        // velocity the shooter never asked for. Both terms below act along the robot's FORE/AFT
        // axis, and which one dominates depends on where the turret is pointing:
        //
        // FEEDER_FORWARD_PUSH_MPS: with the turret pointing straight ahead (or straight back), the
        //   feeder's shove runs down the barrel and the ball leaves faster than commanded. Scales
        //   as cos^2(turretAngle), so it is at full strength at 0/180 deg and gone at +/-90.
        //   Symptom when wrong: shots go long/short, by an amount that changes as the robot rotates
        //   relative to the target.
        //
        // FEEDER_BACKWARD_PUSH_AT_90_MPS: as the turret swings toward 90 deg to EITHER side, that
        //   help disappears and the ball instead comes out with extra velocity toward the ROBOT
        //   REAR. Scales as sin^2(turretAngle) — sin SQUARED, so the push is the same at +90 and
        //   -90, which is how the real effect behaves.
        //   Symptom when wrong: shots are fine with the turret forward but consistently off when
        //   shooting out the side, in a way that is mirrored left and right.
        //
        // Corrected by plain vector subtraction, the same mechanism as shoot-on-the-move.
        //
        // Both are live-tunable on SmartDashboard under "Tuning/Shooter/". The two terms are
        // independent, so tune each at the turret angle where the other contributes nothing: park
        // the turret at 0 deg for the forward term, then at 90 deg for the rearward term. Robot
        // STATIONARY for both.
        // TODO: tune empirically from test shots.
        public static final double FEEDER_FORWARD_PUSH_MPS        = 0.45;
        public static final double FEEDER_BACKWARD_PUSH_AT_90_MPS = 0.15;
    }

    public static final class GroundIntakeConstants {

        // ---- Motor CAN IDs ----
        public static final int RIGHT_PIVOT_ID = 20;
        public static final int LEFT_PIVOT_ID  = 21;
        public static final int ROLLER_ID = 22;
        public static final int LEFT_INDEXER_ID = 23;
        public static final int RIGHT_INDEXER_ID = 24;

        // ---- Gear ratio ----
        // Number of motor rotor rotations per one full mechanism (pivot arm) rotation.
        // Example: if the pivot has a 100:1 gearbox, this is 100.0.
        // Used by Phoenix 6 SensorToMechanismRatio so all positions below are in
        // mechanism rotations (i.e. 0.25 = 90°), not raw rotor counts.
        public static final double PIVOT_GEAR_RATIO = 18.0; // 10 rotor rotations per 1 arm rotation

        // ---- Pivot target positions (mechanism rotations, 1.0 = full 360°) ----
        // ZERO CONVENTION: 0.0 mechanism rotations = arm horizontal (pointing straight out).
        //   - At this angle gravity torque is maximum → Arm_Cosine kG applies at 100%.
        //   - The encoder is seeded automatically in the constructor, assuming the robot
        //     starts with the intake resting on the UP hard stop.
        //
        // UP position: intake stowed, resting on the upper hard stop.
        //   Measure with a protractor or CAD: if the arm is 30° above horizontal, this is 30/360 = 0.0833
        public static final double PIVOT_UP_ROTATIONS   = 0.0; // TODO: measure (mechanism rotations)

        // DOWN position: intake deployed, resting on the lower hard stop.
        //   If the arm is 20° below horizontal, this is -20/360 = -0.0556
        public static final double PIVOT_DOWN_ROTATIONS = 0.355; // 0.35 mechanism rot × 10:1 gear ratio = 3.5 rotor rotations

        // ---- Motion Magic profile ----
        // Cruise velocity: max mechanism speed during a move (rotations/second).
        public static final double PIVOT_CRUISE_VELOCITY_RPS = 1.0;

        // Acceleration: how fast to ramp up to cruise velocity (rotations/second²).
        public static final double PIVOT_ACCELERATION_RPS2 = 2.0;

        // Jerk: limits rate of acceleration change (rotations/second³). 0 = disabled.
        public static final double PIVOT_JERK_RPS3 = 0.0;

        // ---- Pivot PID + feed-forward gains (Slot 0) ----
        public static final double PIVOT_KP = 60.0;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 2.0;
        public static final double PIVOT_KS = 0.0;
        public static final double PIVOT_KV = 0.96;
        public static final double PIVOT_KA = 0.0;
        public static final double PIVOT_KG = 1.3;

        // GravityOffsetPosition: position offset (mechanism rotations) applied to the
        // cosine calculation so that kG is correct when encoder zero ≠ horizontal.
        // Formula: kG × cos(2π × (position + PIVOT_GRAVITY_OFFSET))
        public static final double PIVOT_GRAVITY_OFFSET = 0.2;

        // ---- Named positions for commands (mechanism rotations) ----
        public static final double DEFENSE_POSITION = PIVOT_UP_ROTATIONS;   // intake stowed
        public static final double TRENCH_POSITION  = PIVOT_DOWN_ROTATIONS; // intake deployed for pickup
        public static final double SHOOTER_POSITION = PIVOT_DOWN_ROTATIONS; // intake deployed for feeding shooter

        // ---- Roller / indexer speeds ----
        public static final double INDEXER_TO_ROLLER_RATIO = 0.9244;
        public static final double ROLLER_INTAKE_SPEED = 0.95;
        public static final double RIGHT_INDEXER_SPEED = ROLLER_INTAKE_SPEED * INDEXER_TO_ROLLER_RATIO; // 60% duty cycle
        public static final double LEFT_INDEXER_SPEED = -RIGHT_INDEXER_SPEED; // 60% duty cycle
        public static final double ROLLER_JAM_SPEED = -0.2; // 20% duty cycle

        // ---- Automatic jam recovery ----
        // While intaking, a stalled roller motor (high stator current + not turning) means a ball
        // is wedged. The intake path reverses at full speed for UNJAM_DURATION_SECONDS to spit it
        // back out, then resumes intaking. The flywheel is never touched, so it stays at speed.
        //
        // Stall current: the roller's stator limit is 70 A, so a genuinely stalled roller pins at
        // 70 A. Anything below the limit but well above normal intaking draw works here.
        public static final double ROLLER_STALL_CURRENT_AMPS = 55.0;

        // Stall velocity, in MOTOR ROTOR rotations/second (the roller has no
        // SensorToMechanismRatio configured, so this is the motor's own speed). Free speed at
        // ROLLER_INTAKE_SPEED is roughly 75 rps, so this is "basically not turning".
        public static final double ROLLER_STALL_VELOCITY_RPS = 10.0;

        // How long both conditions must hold before it counts as a jam. Mainly there so the
        // current spike during roller spin-up (high current, still slow) doesn't read as a stall.
        // Lower it for faster recovery, raise it if spin-up false-triggers an unjam.
        public static final double ROLLER_STALL_DEBOUNCE_SECONDS = 0.125;

        // How long to run in reverse before going back to intaking.
        public static final double UNJAM_DURATION_SECONDS = 0.125;

        // Full-speed reverse burst used by the automatic recovery — the mirror of the intake
        // speeds above, at 100% instead of ROLLER_JAM_SPEED's gentler manual 20%.
        public static final double ROLLER_UNJAM_SPEED        = -1.0;
        public static final double RIGHT_INDEXER_UNJAM_SPEED = -1.0;
        public static final double LEFT_INDEXER_UNJAM_SPEED  =  1.0;

        // ---- Position tolerance ----
        // How close (in mechanism rotations) counts as "at position".
        public static final double PIVOT_TOLERANCE_ROTATIONS = 0.02; // ~7°


        public static final double PIVOT_FORCE_DOWN_POWER = 0;
    }

    /**
     * What gets sacrificed when the battery starts to fold, and in what order.
     *
     * <p>The robot cannot have everything it wants from a sagging battery, so rather than letting
     * the RoboRIO's own brownout protection decide — it cuts everything at once, flywheel included —
     * this states a priority up front and enforces it before the RIO ever has to:
     *
     * <ul>
     *   <li><b>Never reduced:</b> the flywheel, the rack, the turret rotator, and the turret's
     *       feeder. A shot that leaves slow is a wasted ball and a wasted cycle, and a turret that
     *       is not where the solution says it is makes every shot after it wrong too.
     *   <li><b>Reduced first:</b> the ground intake's rollers and indexers. Picking fuel up more
     *       slowly costs a little cycle time and nothing else.
     *   <li><b>Reduced second:</b> the swerve drive, by scaling the driver's speed command. Teleop
     *       only — autonomous is left alone, because a path follower that is quietly speed-limited
     *       does not drive the path slower, it drives it wrong.
     * </ul>
     *
     * <p>Tiers are hysteretic and have a minimum dwell, so the hard current spike from a full-speed
     * launch cannot strobe the robot in and out of protection. The whole thing can be switched off
     * mid-match from the dashboard — see {@code PowerBudget.ENABLED}.
     */
    public static final class PowerConstants {
        // ---- Tier thresholds, in volts at the RoboRIO input ----
        // Entry is lower than exit so the tier has to be clearly left, not just brushed. For
        // reference, the RoboRIO's own brownout cutoff is 6.3 V and its warning trips at 6.8 V —
        // these sit well above both, because the point is to never get there.
        public static final double REDUCED_ENTER_VOLTS  = 8.25;
        public static final double REDUCED_EXIT_VOLTS   = 8.75;
        public static final double CRITICAL_ENTER_VOLTS = 7.75;
        public static final double CRITICAL_EXIT_VOLTS  = 8.25;

        // Battery voltage is noisy enough that a single sample means very little; this is the time
        // constant of the low-pass the thresholds are actually compared against. Long enough to
        // ignore the spike from a module reversing direction, short enough to react inside the
        // second or so a real sag takes to become a brownout.
        public static final double VOLTAGE_FILTER_TAU_SECONDS = 0.08;

        // Once a tier is entered it is held at least this long, even if the voltage recovers
        // immediately. Without it, cutting the intake raises the voltage, which restores the intake,
        // which drops the voltage — at loop rate.
        public static final double MIN_TIER_HOLD_SECONDS = 0.75;

        // ---- Drive authority per tier, as a multiplier on the driver's commanded speed ----
        public static final double NORMAL_DRIVE_SCALE   = 1.0;
        public static final double REDUCED_DRIVE_SCALE  = 0.55;
        public static final double CRITICAL_DRIVE_SCALE = 0.30;

        // ---- Intake roller/indexer authority per tier, as a multiplier on commanded output ----
        // Critical is 0: the intake stops entirely rather than browning the robot out to keep
        // collecting fuel it has no power to shoot. The pivot is NOT scaled — it has to hold its
        // position against gravity, and an intake arm falling onto the carpet mid-match is its own
        // problem.
        public static final double NORMAL_INTAKE_SCALE   = 1.0;
        public static final double REDUCED_INTAKE_SCALE  = 0.40;
        public static final double CRITICAL_INTAKE_SCALE = 0.0;

        // ---- Intake roller/indexer current limits per tier, in amps ----
        // Output scaling alone is not enough: a stalled roller at 40% output still pulls whatever
        // the stator limit allows. These are applied to the roller and both indexers on a tier
        // change only, never per loop. NORMAL restores each motor's own configured limits rather
        // than a shared number, so this table only needs the reduced cases.
        public static final double REDUCED_INTAKE_SUPPLY_AMPS  = 15.0;
        public static final double REDUCED_INTAKE_STATOR_AMPS  = 35.0;
        public static final double CRITICAL_INTAKE_SUPPLY_AMPS = 5.0;
        public static final double CRITICAL_INTAKE_STATOR_AMPS = 10.0;

        // Jam detection compares roller stator current against ROLLER_STALL_CURRENT_AMPS, which a
        // roller cannot reach once its stator limit has been pulled below it — the detector would
        // go quiet exactly when jams are most likely. So while a reduced limit is in force the
        // threshold drops to this fraction of it instead.
        public static final double STALL_THRESHOLD_FRACTION_OF_LIMIT = 0.8;
    }

    public static final class LEDConstants {
        public static final int CANDLE_ID = 29;
        public static final int LED_COUNT = 60; // 1m strip at 60 LEDs/m
    }

    public static final class VisionConstants {

        // Camera names as configured in PhotonVision
        public static final String BACK_LEFT_CAMERA_NAME  = "Back_Left_Camera";
        public static final String BACK_RIGHT_CAMERA_NAME = "Back_Right_Camera";
        public static final String FRONT_RIGHT_CAMERA_NAME = "Front_Right_Camera";
        public static final String FRONT_LEFT_CAMERA_NAME  = "Front_Left_Camera";


        /**
         * Camera mounting transforms relative to robot center.
         * X: Forward, Y: Left, Z: Up
         * TODO: Update once final camera placement is confirmed.
         */
        public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.3048, 0.1778, 0.3397),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(135))
        );

        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.3048, -0.1778, 0.3397),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(-135))
        );

        // TO DO: update location of front right camera
        public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
            new Translation3d(0.1651, -0.3143, 0.381),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(-45))
        );

        // TO DO: update location of front left camera
        public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
            new Translation3d(0.1651, 0.3143, 0.381),
            new Rotation3d(0.0, Math.toRadians(11), Math.toRadians(45))
        );

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Standard deviations — higher = less trust. Format: [x, y, theta]
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

        // These std devs above are conservatively tuned for real-world camera noise. maple-sim's
        // simulated cameras don't have anywhere near that much noise, so trusting them exactly
        // as little as real cameras makes odometry drift (e.g. from a wall collision) recover
        // unrealistically slowly in sim. Scales the final std dev down (only in simulation) to
        // let simulated vision correct drift faster — tune this if recovery still feels too slow
        // or corrections start looking too twitchy/aggressive.
        public static final double SIM_STD_DEV_SCALE_FACTOR = 0.1;

        public static final double MAX_TAG_DISTANCE_METERS = 6.0;
        public static final double MAX_POSE_AMBIGUITY      = 0.2;
        public static final int    MIN_TAGS_FOR_MULTI_TAG  = 2;

        // Tags to always ignore for pose estimation (tower back tags + outpost tags on both sides)
        public static final Set<Integer> IGNORED_TAG_IDS = Set.of(13, 14, 15, 16, 29, 30, 31, 32);
        //public static final Set<Integer> IGNORED_TAG_IDS = Set.of();
    }
}
