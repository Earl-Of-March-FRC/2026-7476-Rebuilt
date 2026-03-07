// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.Constants.GameModelConstants;

/**
 * Tracks REBUILT 2026 game state including hub activation rules,
 * shift timing, and grace windows. Provides driver visibility via
 * dashboard and ready-to-use getters for mechanism commands.
 */
public class GameModel {

  // ═══════════════════════════════════════════════════════════════════════════
  // ENUMS
  // ═══════════════════════════════════════════════════════════════════════════

  /** Match phases for REBUILT 2026 */
  public enum MatchPhase {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // STATE
  // ═══════════════════════════════════════════════════════════════════════════

  private MatchPhase phase = MatchPhase.DISABLED;
  private boolean hubActive = true;
  private boolean nextHubActive = true;
  private boolean weWonAuto = false;
  private boolean autoWinnerDetected = false;
  private boolean autoWinnerFromFms = false;
  private String rawGameSpecificMessage = "";

  private double graceTimer = 0;

  private double matchTime = 0;
  private char allianceColour = 'B';

  // Timestamp tracking for grace window timing
  private double lastTeleopUpdateTimestampSec = 0.0;

  // For one-time notifications
  private MatchPhase lastNotifiedPhase = null;
  private boolean graceEndingAlerted = false;

  // ═══════════════════════════════════════════════════════════════════════════
  // PUBLIC GETTERS (Bucket 4 - for future mechanism commands)
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * Can we score fuel right now? True if hub is active OR in grace window.
   * Use this in launcher commands to decide whether to launch.
   */
  public boolean canScore() {
    return hubActive || graceTimer > 0;
  }

  /**
   * Is our hub currently active (not counting grace window)?
   */
  public boolean isHubActive() {
    return hubActive;
  }

  /**
   * Is it endgame (last 30 seconds)? Use for climb triggers.
   */
  public boolean isEndgame() {
    return phase == MatchPhase.ENDGAME;
  }

  /**
   * Get current match phase.
   */
  public MatchPhase getPhase() {
    return phase;
  }

  /**
   * Get seconds remaining in grace window (0 if not in grace).
   */
  public double getGraceRemaining() {
    return graceTimer;
  }

  /**
   * Get seconds until the next phase transition.
   */
  public double getTimeToNextShift() {
    if (matchTime > GameModelConstants.AUTO_END) {
      return matchTime - GameModelConstants.AUTO_END;
    } else if (matchTime > GameModelConstants.TRANSITION_END) {
      return matchTime - GameModelConstants.TRANSITION_END;
    } else if (matchTime > GameModelConstants.SHIFT_1_END) {
      return matchTime - GameModelConstants.SHIFT_1_END;
    } else if (matchTime > GameModelConstants.SHIFT_2_END) {
      return matchTime - GameModelConstants.SHIFT_2_END;
    } else if (matchTime > GameModelConstants.SHIFT_3_END) {
      return matchTime - GameModelConstants.SHIFT_3_END;
    } else if (matchTime > GameModelConstants.SHIFT_4_END) {
      return matchTime - GameModelConstants.SHIFT_4_END;
    }
    return matchTime; // Time until match ends
  }

  /**
   * Should we collect fuel but hold it (not launch)?
   * True when hub is inactive and grace has expired.
   */
  public boolean shouldStockpile() {
    return !hubActive && graceTimer <= 0;
  }

  /**
   * Did our alliance win auto (score more fuel)?
   */
  public boolean weWonAuto() {
    return weWonAuto;
  }

  /**
   * Get current match time (seconds remaining).
   */
  public double getMatchTime() {
    return matchTime;
  }

  /**
   * Has the auto winner been detected from FMS?
   */
  public boolean isAutoWinnerDetected() {
    return autoWinnerDetected;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // UPDATE METHODS (called from Robot.java)
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * Call from Robot.autonomousInit()
   */
  public void initAuto() {
    phase = MatchPhase.AUTO;
    hubActive = true;
    weWonAuto = false;
    autoWinnerDetected = false;
    autoWinnerFromFms = false;
    rawGameSpecificMessage = "";
    graceTimer = 0.0;
    graceEndingAlerted = false;
    lastNotifiedPhase = null;
    lastTeleopUpdateTimestampSec = 0.0;
    publishState();
  }

  /**
   * Call from Robot.autonomousPeriodic()
   */
  public void updateAuto() {
    matchTime = DriverStation.getMatchTime() + 140; // Auto is 0-20, teleop is 0-140
    publishState();
  }

  /**
   * Call from Robot.teleopInit()
   */
  public void initTeleop() {
    phase = MatchPhase.TRANSITION;
    hubActive = true;
    weWonAuto = false;
    autoWinnerDetected = false;
    autoWinnerFromFms = false;
    rawGameSpecificMessage = "";
    graceTimer = 0.0;
    lastNotifiedPhase = null;
    graceEndingAlerted = false;
    lastTeleopUpdateTimestampSec = Timer.getFPGATimestamp();
    publishState();
  }

  /**
   * Call from Robot.teleopPeriodic()
   */
  public void updateTeleop() {
    matchTime = DriverStation.getMatchTime();

    // Detect auto winner during TRANSITION. After TRANSITION, freeze to a safe
    // default if still unknown to avoid mid-shift flips when/if the message arrives
    // late.
    if (!autoWinnerDetected) {
      // Try to read the FMS message while it is expected to be available.
      // Also allow a single "last chance" read on the boundary tick right as
      // transition ends.
      detectAutoWinner();
      if (!autoWinnerDetected && matchTime <= GameModelConstants.TRANSITION_END) {
        // Default assumption if we never received the message (practice/sim/DS glitch)
        weWonAuto = false;
        autoWinnerDetected = true;
        autoWinnerFromFms = false;
      }
    }

    // Compute new phase and hub state
    MatchPhase oldPhase = phase;
    boolean oldHubActive = hubActive;

    updatePhaseAndHubState();

    // Grace window logic
    updateGraceWindow(oldHubActive, getTeleopDtSeconds());

    // Alerts on phase changes
    handlePhaseChangeAlerts(oldPhase, oldHubActive);

    // Alert when grace is about to expire
    handleGraceEndingAlert();

    publishState();
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // PRIVATE HELPERS
  // ═══════════════════════════════════════════════════════════════════════════

  private void updatePhaseAndHubState() {
    if (matchTime > GameModelConstants.TRANSITION_END) {
      phase = MatchPhase.TRANSITION;
      hubActive = true;
      if (autoWinnerDetected && weWonAuto) {
        nextHubActive = false;
      }
    } else if (matchTime > GameModelConstants.SHIFT_1_END) {
      phase = MatchPhase.SHIFT_1;
      hubActive = !weWonAuto; // Loser of auto is active first
      nextHubActive = weWonAuto;
    } else if (matchTime > GameModelConstants.SHIFT_2_END) {
      phase = MatchPhase.SHIFT_2;
      hubActive = weWonAuto;
      nextHubActive = !weWonAuto;
    } else if (matchTime > GameModelConstants.SHIFT_3_END) {
      phase = MatchPhase.SHIFT_3;
      hubActive = !weWonAuto;
      nextHubActive = weWonAuto;
    } else if (matchTime > GameModelConstants.SHIFT_4_END) {
      phase = MatchPhase.SHIFT_4;
      hubActive = weWonAuto;
      nextHubActive = true;
    } else {
      phase = MatchPhase.ENDGAME;
      hubActive = true;
    }
  }

  private void updateGraceWindow(boolean oldHubActive, double dtSeconds) {
    // Start grace timer when hub transitions from active to inactive
    if (oldHubActive && !hubActive) {
      graceTimer = GameModelConstants.GRACE_DURATION;
      graceEndingAlerted = false;
    }

    // Count down grace timer
    graceTimer = Math.max(0, graceTimer - dtSeconds);
  }

  private double getTeleopDtSeconds() {
    double nowSec = Timer.getFPGATimestamp();

    // If this is the first call (or we were not initialized), assume nominal loop
    // time.
    if (lastTeleopUpdateTimestampSec <= 0.0) {
      lastTeleopUpdateTimestampSec = nowSec;
      return 0.02;
    }

    double dtSec = nowSec - lastTeleopUpdateTimestampSec;
    lastTeleopUpdateTimestampSec = nowSec;

    // Guard against timestamp glitches or long pauses causing a huge jump.
    // Still respects normal jitter while keeping behavior predictable.
    if (dtSec <= 0.0 || dtSec > 0.1) {
      return 0.02;
    }

    return dtSec;
  }

  private void detectAutoWinner() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceColour = (alliance.get() == Alliance.Red) ? 'R' : 'B';
    }

    rawGameSpecificMessage = DriverStation.getGameSpecificMessage();
    if (!rawGameSpecificMessage.isEmpty()) {
      // Game message contains the alliance letter that WON auto
      weWonAuto = (rawGameSpecificMessage.charAt(0) == allianceColour);
      autoWinnerDetected = true;
      autoWinnerFromFms = true;
    }
    // If message is empty, DON'T lock in - keep trying next cycle
    // This handles late-arriving FMS messages
  }

  private void handlePhaseChangeAlerts(MatchPhase oldPhase, boolean oldHubActive) {
    if (phase != lastNotifiedPhase) {
      if (phase == MatchPhase.ENDGAME) {
        sendAlert("ENDGAME", "30 seconds - Both hubs ACTIVE!", NotificationLevel.INFO);
      } else if (hubActive != oldHubActive || phase != oldPhase) {
        if (hubActive) {
          sendAlert(phase.name(), "Hub ACTIVE - SCORE!", NotificationLevel.INFO);
        } else {
          sendAlert(phase.name(), "Hub INACTIVE - hold fire", NotificationLevel.WARNING);
        }
      }

      // Alert when grace window starts
      if (oldHubActive && !hubActive) {
        sendAlert("GRACE WINDOW", "3 seconds to finish scoring!", NotificationLevel.WARNING);
      }

      lastNotifiedPhase = phase;
    }
  }

  private void handleGraceEndingAlert() {
    if (!graceEndingAlerted && graceTimer > 0 && graceTimer < 1.0 && !hubActive) {
      sendAlert("GRACE ENDING", String.format("%.1fs left to score!", graceTimer), NotificationLevel.ERROR);
      graceEndingAlerted = true;
    }
  }

  private void sendAlert(String title, String message, NotificationLevel level) {
    Elastic.sendNotification(new Notification(level, title, message, 3000));
  }

  private void publishState() {
    SmartDashboard.putString("Game/Phase", phase.name());
    SmartDashboard.putBoolean("Game/Hub State", hubActive);
    SmartDashboard.putBoolean("Game/Next Hub State", nextHubActive);
    SmartDashboard.putBoolean("Game/Can Score", canScore());
    SmartDashboard.putNumber("Game/Grace Remaining", graceTimer);
    SmartDashboard.putNumber("Game/Phase Time", getTimeToNextShift());
    SmartDashboard.putNumber("Game/Match Time", matchTime);
    SmartDashboard.putBoolean("Game/WeWonAuto", weWonAuto);
    SmartDashboard.putBoolean("Game/AutoWinnerDetected", autoWinnerDetected);
    SmartDashboard.putBoolean("Game/AutoWinnerFromFms", autoWinnerFromFms);
    SmartDashboard.putString("Game/RawGameSpecificMessage", rawGameSpecificMessage);
    SmartDashboard.putBoolean("Game/ShouldStockpile", shouldStockpile());
    SmartDashboard.putBoolean("Game/IsEndgame", isEndgame());
  }
}
