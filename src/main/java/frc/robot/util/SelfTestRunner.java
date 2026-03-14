package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.SelfTestConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem.CameraHealthSnapshot;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class SelfTestRunner {
  private enum SelfTestState {
    READY,
    RUNNING,
    PASS,
    WARN,
    FAIL
  }

  private static final String kDashboardPrefix = "Self Test/";
  private static final String kRunKey = kDashboardPrefix + "Run Self Test";
  private static final String kStatusKey = kDashboardPrefix + "Status";
  private static final String kSummaryKey = kDashboardPrefix + "Summary";
  private static final String kNextActionKey = kDashboardPrefix + "Next Action";
  private static final String[] kInstructionLines = {
      "1. Put the robot on blocks or in a clear area and do not touch the controllers.",
      "2. Point the robot so at least one vision camera can see AprilTags.",
      "3. Switch Driver Station to Test mode, then press Run Self Test.",
      "4. Review PASS/WARN/FAIL and fix issues before queueing."
  };

  private final DrivetrainSubsystem drivetrain;
  private final String[] cameraNames;
  private final boolean[] cameraUpdatedDuringRun;
  private final boolean[] cameraSawTagsDuringRun;
  private final boolean[] cameraPoseDuringRun;
  private final double[] cameraBestAreaDuringRun;
  private final double[] cameraBestAmbiguityDuringRun;

  private boolean lastRunRequested = false;
  private SelfTestState state = SelfTestState.READY;
  private double runStartTimestampSec = 0.0;
  private int sampleCount = 0;
  private double maxAbsGyroRateDegPerSec = 0.0;

  public SelfTestRunner(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    int cameraCount = drivetrain.getCameraCount();
    cameraNames = new String[cameraCount];
    cameraUpdatedDuringRun = new boolean[cameraCount];
    cameraSawTagsDuringRun = new boolean[cameraCount];
    cameraPoseDuringRun = new boolean[cameraCount];
    cameraBestAreaDuringRun = new double[cameraCount];
    cameraBestAmbiguityDuringRun = new double[cameraCount];

    for (int i = 0; i < cameraCount; i++) {
      cameraNames[i] = drivetrain.getCameraHealthSnapshot(i).name();
      cameraBestAmbiguityDuringRun[i] = Double.POSITIVE_INFINITY;
    }
  }

  public void enterTestMode() {
    state = SelfTestState.READY;
    lastRunRequested = false;
    resetRunSamples();
    publishInstructions();
    publishStatus(
        "Ready to run the passive self-test.",
        "Keep the robot still, point it at AprilTags, then press Run Self Test.",
        "Not run yet",
        "Not run yet",
        "Not run yet",
        "Not run yet",
        "Not run yet");
    publishCameraStatuses(repeatStatus("Not run yet"));
    SmartDashboard.putBoolean(kRunKey, false);

    Elastic.selectTab("Self Test");
    Elastic.sendNotification(new Notification(
        NotificationLevel.INFO,
        "Self Test Ready",
        "Keep the robot still, point it at visible AprilTags, then press Run Self Test.",
        8000));
    DriverStation.reportWarning(
        "Self Test ready: keep the robot still, point it at visible AprilTags, and press Run Self Test on the dashboard.",
        false);
  }

  public void periodic() {
    boolean runRequested = SmartDashboard.getBoolean(kRunKey, false);
    if (runRequested && !lastRunRequested && state != SelfTestState.RUNNING) {
      startRun();
      SmartDashboard.putBoolean(kRunKey, false);
      runRequested = false;
    }
    lastRunRequested = runRequested;

    if (state == SelfTestState.RUNNING) {
      sampleRun();
      if (Timer.getFPGATimestamp() - runStartTimestampSec >= SelfTestConstants.kRunDurationSeconds) {
        finishRun();
      }
    }
  }

  private void startRun() {
    state = SelfTestState.RUNNING;
    runStartTimestampSec = Timer.getFPGATimestamp();
    resetRunSamples();
    publishStatus(
        "Collecting gyro and vision samples.",
        "Keep the robot still until the self-test finishes.",
        "Running...",
        "Running...",
        "Running...",
        "Running...",
        "Running...");
    publishCameraStatuses(repeatStatus("Collecting data..."));
  }

  private void sampleRun() {
    sampleCount++;
    maxAbsGyroRateDegPerSec = Math.max(maxAbsGyroRateDegPerSec,
        Math.abs(drivetrain.getGyroRateDegreesPerSecond()));

    for (int i = 0; i < cameraNames.length; i++) {
      CameraHealthSnapshot snapshot = drivetrain.getCameraHealthSnapshot(i);
      cameraUpdatedDuringRun[i] |= snapshot.lastUpdateTimestampSec() >= runStartTimestampSec;
      cameraSawTagsDuringRun[i] |= snapshot.lastSawTagTimestampSec() >= runStartTimestampSec;
      cameraPoseDuringRun[i] |= snapshot.lastPoseTimestampSec() >= runStartTimestampSec;
      cameraBestAreaDuringRun[i] = Math.max(cameraBestAreaDuringRun[i], snapshot.bestArea());
      cameraBestAmbiguityDuringRun[i] = Math.min(cameraBestAmbiguityDuringRun[i], snapshot.bestAmbiguity());
    }
  }

  private void finishRun() {
    List<String> failures = new ArrayList<>();
    List<String> warnings = new ArrayList<>();
    double nowSec = Timer.getFPGATimestamp();
    boolean[] freshCameraUpdates = new boolean[cameraNames.length];

    for (int i = 0; i < cameraNames.length; i++) {
      CameraHealthSnapshot snapshot = drivetrain.getCameraHealthSnapshot(i);
      freshCameraUpdates[i] = cameraUpdatedDuringRun[i]
          && snapshot.lastUpdateTimestampSec() >= nowSec - SelfTestConstants.kCameraFreshnessTimeoutSeconds;
    }

    boolean gyroConnected = drivetrain.isGyroConnected();
    boolean gyroStable = sampleCount > 0
        && maxAbsGyroRateDegPerSec <= SelfTestConstants.kGyroStillnessThresholdDegPerSec;
    int updatedCameraCount = countTrue(freshCameraUpdates);
    boolean sawAnyTags = hasAny(cameraSawTagsDuringRun);
    boolean hasAnyPose = hasAny(cameraPoseDuringRun);
    boolean qualityOkay = false;

    for (int i = 0; i < cameraNames.length; i++) {
      if (cameraSawTagsDuringRun[i]
          && cameraBestAreaDuringRun[i] >= PhotonConstants.kMinSingleTagArea
          && cameraBestAmbiguityDuringRun[i] <= PhotonConstants.kAmbiguityDiscardThreshold) {
        qualityOkay = true;
        break;
      }
    }

    if (!gyroConnected) {
      failures.add("Gyro disconnected");
    }
    if (!gyroStable) {
      warnings.add(String.format("Robot moved or gyro drifted (max %.2f deg/s)", maxAbsGyroRateDegPerSec));
    }
    if (cameraNames.length == 0) {
      failures.add("No cameras configured for the active profile");
    } else if (updatedCameraCount != cameraNames.length) {
      failures.add("Missing fresh camera updates from " + describeMissing(freshCameraUpdates));
    }
    if (!sawAnyTags) {
      failures.add("No AprilTags were seen during the run");
    }
    if (sawAnyTags && !hasAnyPose) {
      failures.add("Tags were visible but no valid vision pose was produced");
    }
    if (sawAnyTags && hasAnyPose && !qualityOkay) {
      warnings.add("Visible tags were small or ambiguous");
    }

    String gyroDetail = gyroConnected
        ? formatCheck("PASS", String.format("gyro connected, max rate %.2f deg/s", maxAbsGyroRateDegPerSec))
        : formatCheck("FAIL", "gyro disconnected");
    if (gyroConnected && !gyroStable) {
      gyroDetail = formatCheck("WARN",
          String.format("keep robot still while running (max %.2f deg/s)", maxAbsGyroRateDegPerSec));
    }

    String cameraUpdateDetail;
    String tagVisibilityDetail;
    String visionPoseDetail;
    String visionQualityDetail;
    if (cameraNames.length == 0) {
      cameraUpdateDetail = formatCheck("FAIL", "no cameras configured for the active profile");
      tagVisibilityDetail = formatCheck("FAIL", "no cameras configured for the active profile");
      visionPoseDetail = formatCheck("FAIL", "no cameras configured for the active profile");
      visionQualityDetail = formatCheck("FAIL", "no cameras configured for the active profile");
    } else {
      cameraUpdateDetail = updatedCameraCount == cameraNames.length
          ? formatCheck("PASS", updatedCameraCount + "/" + cameraNames.length + " expected cameras updated")
          : formatCheck("FAIL", "missing fresh updates from " + describeMissing(freshCameraUpdates));
      tagVisibilityDetail = sawAnyTags
          ? formatCheck("PASS", "tags seen by " + describeSeen(cameraSawTagsDuringRun))
          : formatCheck("FAIL", "no cameras saw AprilTags");
      visionPoseDetail = hasAnyPose
          ? formatCheck("PASS", "valid pose from " + describeSeen(cameraPoseDuringRun))
          : formatCheck("FAIL", "no valid vision pose update during the run");
      visionQualityDetail = qualityOkay
          ? formatCheck("PASS", "at least one camera met current area/ambiguity thresholds")
          : formatCheck(sawAnyTags && hasAnyPose ? "WARN" : "FAIL",
              sawAnyTags ? "tags were seen but quality was marginal" : "quality could not be assessed");
    }

    publishCameraStatuses(buildCameraStatuses(freshCameraUpdates));

    if (!failures.isEmpty()) {
      state = SelfTestState.FAIL;
    } else if (!warnings.isEmpty()) {
      state = SelfTestState.WARN;
    } else {
      state = SelfTestState.PASS;
    }

    String summary = switch (state) {
      case PASS -> "PASS - robot looks ready for a quick field-side check.";
      case WARN -> "WARN - robot is mostly healthy, but review the warnings before queueing.";
      case FAIL -> "FAIL - fix the listed issue before relying on the robot.";
      default -> "READY";
    };

    String nextAction = !failures.isEmpty()
        ? failures.get(0)
        : warnings.isEmpty() ? "Self-test passed. Re-run after any hardware change."
            : warnings.get(0);

    publishStatus(summary, nextAction, gyroDetail, cameraUpdateDetail, tagVisibilityDetail, visionPoseDetail,
        visionQualityDetail);

    Elastic.sendNotification(new Notification(
        state == SelfTestState.FAIL ? NotificationLevel.ERROR
            : state == SelfTestState.WARN ? NotificationLevel.WARNING : NotificationLevel.INFO,
        "Self Test " + state.name(),
        nextAction,
        8000));
  }

  private void publishInstructions() {
    for (int i = 0; i < kInstructionLines.length; i++) {
      SmartDashboard.putString(kDashboardPrefix + "Instructions/" + (i + 1), kInstructionLines[i]);
    }
  }

  private void publishStatus(
      String summary,
      String nextAction,
      String gyroDetail,
      String cameraUpdateDetail,
      String tagVisibilityDetail,
      String visionPoseDetail,
      String visionQualityDetail) {
    SmartDashboard.putString(kStatusKey, state.name());
    SmartDashboard.putString(kSummaryKey, summary);
    SmartDashboard.putString(kNextActionKey, nextAction);
    SmartDashboard.putString(kDashboardPrefix + "Checks/Gyro", gyroDetail);
    SmartDashboard.putString(kDashboardPrefix + "Checks/Camera Updates", cameraUpdateDetail);
    SmartDashboard.putString(kDashboardPrefix + "Checks/Tag Visibility", tagVisibilityDetail);
    SmartDashboard.putString(kDashboardPrefix + "Checks/Vision Pose", visionPoseDetail);
    SmartDashboard.putString(kDashboardPrefix + "Checks/Vision Quality", visionQualityDetail);
  }

  private void publishCameraStatuses(String[] statuses) {
    for (int i = 0; i < 3; i++) {
      String status = i < statuses.length ? statuses[i] : "No camera configured";
      SmartDashboard.putString(kDashboardPrefix + "Cameras/" + (i + 1), status);
    }
  }

  private String[] buildCameraStatuses(boolean[] freshCameraUpdates) {
    String[] statuses = new String[cameraNames.length];
    for (int i = 0; i < cameraNames.length; i++) {
      String statusPrefix;
      if (!freshCameraUpdates[i]) {
        statusPrefix = "FAIL";
      } else if (cameraPoseDuringRun[i]) {
        statusPrefix = "PASS";
      } else if (cameraSawTagsDuringRun[i]) {
        statusPrefix = "WARN";
      } else {
        statusPrefix = "INFO";
      }

      StringBuilder detail = new StringBuilder();
      detail.append(cameraNames[i]).append(": ").append(statusPrefix).append(" - ");
      if (!freshCameraUpdates[i]) {
        detail.append("no recent updates");
      } else if (cameraPoseDuringRun[i]) {
        detail.append("valid pose update");
      } else if (cameraSawTagsDuringRun[i]) {
        detail.append("tags seen, but no valid pose");
      } else {
        detail.append("updates only, no tags in view");
      }

      if (cameraSawTagsDuringRun[i]) {
        detail.append(String.format(" (area %.2f, ambiguity %.2f)",
            cameraBestAreaDuringRun[i],
            cameraBestAmbiguityDuringRun[i]));
      }

      statuses[i] = detail.toString();
    }
    return statuses;
  }

  private String[] repeatStatus(String status) {
    String[] statuses = new String[cameraNames.length];
    for (int i = 0; i < statuses.length; i++) {
      statuses[i] = cameraNames[i] + ": " + status;
    }
    return statuses;
  }

  private void resetRunSamples() {
    sampleCount = 0;
    maxAbsGyroRateDegPerSec = 0.0;

    for (int i = 0; i < cameraNames.length; i++) {
      cameraUpdatedDuringRun[i] = false;
      cameraSawTagsDuringRun[i] = false;
      cameraPoseDuringRun[i] = false;
      cameraBestAreaDuringRun[i] = 0.0;
      cameraBestAmbiguityDuringRun[i] = Double.POSITIVE_INFINITY;
    }
  }

  private static boolean hasAny(boolean[] values) {
    return countTrue(values) > 0;
  }

  private static int countTrue(boolean[] values) {
    int count = 0;
    for (boolean value : values) {
      if (value) {
        count++;
      }
    }
    return count;
  }

  private String describeMissing(boolean[] values) {
    List<String> missing = new ArrayList<>();
    for (int i = 0; i < values.length; i++) {
      if (!values[i]) {
        missing.add(cameraNames[i]);
      }
    }
    return String.join(", ", missing);
  }

  private String describeSeen(boolean[] values) {
    List<String> seen = new ArrayList<>();
    for (int i = 0; i < values.length; i++) {
      if (values[i]) {
        seen.add(cameraNames[i]);
      }
    }
    return seen.isEmpty() ? "none" : String.join(", ", seen);
  }

  private static String formatCheck(String status, String detail) {
    return status + " - " + detail;
  }
}
