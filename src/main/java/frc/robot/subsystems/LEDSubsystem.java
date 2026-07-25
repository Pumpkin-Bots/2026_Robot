// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    // Indices 0-7 are the CANdle's onboard LEDs; 8 onward are the external strip.
    private static final int LED_START = 0;
    private static final int LED_END   = 7 + Constants.LEDConstants.LED_COUNT; // 67

    // Teleop timing boundaries (seconds remaining, counting down).
    // 125.0 = end of transition period; 105/80/55/30 = shift change boundaries.
    private static final double[] SWITCH_TIMES = {125.0, 105.0, 80.0, 55.0, 30.0};

    // Flash window: how many seconds before a switch to start flashing.
    private static final double FLASH_WINDOW_SECONDS = 5.0;

    // Always flash in the last N seconds of the match regardless of hub state.
    private static final double MATCH_END_FLASH_SECONDS = 10.0;

    // Flash rate ramps from MIN_HZ (far from switch) to MAX_HZ (at switch).
    private static final double FLASH_MIN_HZ = 1.0;
    private static final double FLASH_MAX_HZ = 8.0;

    // Brightness wave applied during solid (non-strobe) states.
    private static final double WAVE_HZ             = 1;  // one full pulse per 2 s
    private static final double WAVE_MIN_BRIGHTNESS = 0.05;

    // Chase pattern: number of LEDs per alternating segment.
    // 10 LEDs → 7 segments across the 68-LED strip, each clearly visible from the field.
    private static final int CHASE_SEGMENT_SIZE = 10;
    // ceil((LED_END - LED_START + 1) / CHASE_SEGMENT_SIZE) = ceil(68/10) = 7
    private static final int NUM_SEGMENTS =
        (LED_END - LED_START + CHASE_SEGMENT_SIZE) / CHASE_SEGMENT_SIZE;

    private static final RGBWColor GREEN  = new RGBWColor(0, 255, 0);
    private static final RGBWColor RED    = new RGBWColor(255, 0, 0);
    private static final RGBWColor WHITE  = new RGBWColor(200, 200, 200);
    private static final RGBWColor OFF    = new RGBWColor(0, 0, 0);
    private static final RGBWColor YELLOW = new RGBWColor(255, 200, 0);

    private final CANdle m_candle;

    private final SolidColor m_solidGreen = new SolidColor(LED_START, LED_END).withColor(GREEN);
    private final SolidColor m_solidRed   = new SolidColor(LED_START, LED_END).withColor(RED);
    private final SolidColor m_solidWhite = new SolidColor(LED_START, LED_END).withColor(WHITE);
    private final SolidColor m_solidOff   = new SolidColor(LED_START, LED_END).withColor(OFF);

    // Reused control request for the brightness wave — withColor() mutates in place.
    private final SolidColor m_waveControl = new SolidColor(LED_START, LED_END);

    // Per-segment SolidColor controls for the chase pattern (created once, reused every toggle).
    // SolidColor with a range-limited startIndex/endIndex only affects that range of LEDs,
    // so calling setControl for each segment in sequence builds up the full pattern.
    private final SolidColor[] m_chaseSegs = new SolidColor[NUM_SEGMENTS];

    // Chase pattern state.  m_lastSentPhase = -1 forces the first send after any mode change.
    private int     m_chasePhase     = 0;
    private int     m_lastSentPhase  = -1;
    private boolean m_lastSentActive = false;
    private double  m_lastToggleTime = 0.0;

    public LEDSubsystem() {
        m_candle = new CANdle(Constants.LEDConstants.CANDLE_ID);

        for (int i = 0; i < NUM_SEGMENTS; i++) {
            int segStart = LED_START + i * CHASE_SEGMENT_SIZE;
            int segEnd   = Math.min(segStart + CHASE_SEGMENT_SIZE - 1, LED_END);
            m_chaseSegs[i] = new SolidColor(segStart, segEnd);
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            double b = waveBrightness();
            m_waveControl.withColor(new RGBWColor((int)(128 * b), 0, (int)(128 * b)));
            m_candle.setControl(m_waveControl);
            m_lastSentPhase = -1;
            return;
        }

        if (DriverStation.isDisabled()) {
            double b = waveBrightness();
            m_waveControl.withColor(new RGBWColor((int)(255 * b), (int)(20 * b), 0));
            m_candle.setControl(m_waveControl);
            m_lastSentPhase = -1;
            return;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        var allianceOpt = DriverStation.getAlliance();
        if (gameData.isEmpty() || allianceOpt.isEmpty()) {
            m_candle.setControl(m_solidWhite);
            m_lastSentPhase = -1;
            return;
        }

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            m_candle.setControl(m_solidOff);
            m_lastSentPhase = -1;
            return;
        }

        char gd = gameData.charAt(0);
        Alliance alliance = allianceOpt.get();
        boolean active  = isHubActive(matchTime, gd, alliance);
        double  flashHz = getFlashHz(matchTime, gd, alliance);

        if (flashHz > 0) {
            applyFlashChase(active, flashHz);
        } else {
            m_lastSentPhase = -1;
            m_candle.setControl(active ? m_solidGreen : m_solidRed);
        }
    }

    /**
     * Individually-addressable chase pattern for flash windows.
     *
     * The strip is divided into CHASE_SEGMENT_SIZE-LED segments. Alternating segments show
     * the hub-status color (green = active, red = inactive) while the others show bright
     * yellow — then on each toggle the two sets swap, creating a "jumping checkerboard"
     * effect. Unlike a simple on/off strobe, the strip is always fully lit with both colors
     * visible, giving maximum brightness and contrast from the driver station.
     *
     * SolidColor controls are range-limited to each segment so calling setControl per segment
     * in sequence builds the full pattern without disturbing other ranges. CAN frames are only
     * sent when the displayed pattern actually changes (at most flashHz × NUM_SEGMENTS per second).
     */
    private void applyFlashChase(boolean active, double flashHz) {
        double now = Timer.getFPGATimestamp();
        if (now - m_lastToggleTime >= 0.5 / flashHz) {
            m_chasePhase     = 1 - m_chasePhase;
            m_lastToggleTime = now;
        }

        // Skip CAN traffic when nothing has changed.
        if (m_chasePhase == m_lastSentPhase && active == m_lastSentActive) return;
        m_lastSentPhase  = m_chasePhase;
        m_lastSentActive = active;

        RGBWColor primaryColor = active ? GREEN : RED;
        for (int i = 0; i < NUM_SEGMENTS; i++) {
            RGBWColor color = ((i + m_chasePhase) % 2 == 0) ? primaryColor : YELLOW;
            m_candle.setControl(m_chaseSegs[i].withColor(color));
        }
    }

    /** Smooth brightness scale oscillating between WAVE_MIN_BRIGHTNESS and 1.0. */
    private double waveBrightness() {
        double sine = 0.5 + 0.5 * Math.sin(2.0 * Math.PI * WAVE_HZ * Timer.getFPGATimestamp());
        return WAVE_MIN_BRIGHTNESS + sine * (1.0 - WAVE_MIN_BRIGHTNESS);
    }

    /**
     * Returns the flash rate in Hz if we are within the flash window of an
     * upcoming switch AND our hub state actually changes at that switch.
     * Also flashes in the last MATCH_END_FLASH_SECONDS regardless of hub state.
     * Returns 0 if no flash is needed.
     */
    private double getFlashHz(double matchTime, char gameData, Alliance alliance) {
        // Always flash at end of match.
        if (matchTime <= MATCH_END_FLASH_SECONDS) {
            double fraction = 1.0 - (matchTime / MATCH_END_FLASH_SECONDS);
            return FLASH_MIN_HZ + fraction * (FLASH_MAX_HZ - FLASH_MIN_HZ);
        }

        for (double switchTime : SWITCH_TIMES) {
            if (matchTime >= switchTime && matchTime <= switchTime + FLASH_WINDOW_SECONDS) {
                // Only flash if our hub state actually changes at this boundary.
                boolean before = isHubActive(switchTime + 0.1, gameData, alliance);
                boolean after  = isHubActive(switchTime - 0.1, gameData, alliance);
                if (before == after) continue;

                double timeUntilSwitch = matchTime - switchTime;
                double fraction = 1.0 - (timeUntilSwitch / FLASH_WINDOW_SECONDS);
                return FLASH_MIN_HZ + fraction * (FLASH_MAX_HZ - FLASH_MIN_HZ);
            }
        }
        return 0;
    }

    /**
     * Returns true if our alliance's hub is active at the given match time.
     *
     * Game data 'R' = Red won autonomous (Red inactive in Shift 1).
     * Game data 'B' = Blue won autonomous (Blue inactive in Shift 1).
     * Shifts alternate: winner inactive in odd shifts, active in even shifts.
     * End game (< 30s): hub is always active.
     */
    private boolean isHubActive(double matchTime, char gameData, Alliance alliance) {
        if (matchTime > 125) return true; // Transition period: both hubs active
        if (matchTime < 30)  return true; // End game: always active

        boolean weAreWinner = (alliance == Alliance.Red  && gameData == 'R')
                           || (alliance == Alliance.Blue && gameData == 'B');

        if (matchTime < 55)  return  weAreWinner;  // Shift 4: winner active
        if (matchTime < 80)  return !weAreWinner;  // Shift 3: winner inactive
        if (matchTime < 105) return  weAreWinner;  // Shift 2: winner active
        return                      !weAreWinner;  // Shift 1: winner inactive
    }
}
