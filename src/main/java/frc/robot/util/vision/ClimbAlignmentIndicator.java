package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbAlignmentConstants;
import frc.robot.Constants.FieldConstants;

/**
 * Passive climb alignment indicator that publishes GO/NO-GO status and
 * diagnostic data to the driver dashboard. Computes how close the robot is
 * to an ideal Tower climbing position and whether the expected AprilTags
 * are visible with acceptable quality.
 *
 * Does not control any motors or subsystems.
 */
public class ClimbAlignmentIndicator {

  private final Pose2d[] blueClimbPoses;
  private final Pose2d[] redClimbPoses;

  public ClimbAlignmentIndicator() {
    blueClimbPoses = computeClimbPoses(ClimbAlignmentConstants.kBlueTowerTagIds);
    redClimbPoses = computeClimbPoses(ClimbAlignmentConstants.kRedTowerTagIds);
  }

  /**
   * Derive two ideal climb poses (left-post and right-post) from the Tower
   * AprilTag positions in the field layout.
   *
   * @param towerTagIds the two Tower tag IDs for the relevant alliance
   * @return array of two Pose2d: [0] = left post, [1] = right post
   */
  private Pose2d[] computeClimbPoses(int[] towerTagIds) {
    Optional<Pose3d> tagPoseOpt = FieldConstants.kfieldLayout.getTagPose(towerTagIds[0]);
    if (tagPoseOpt.isEmpty()) {
      return new Pose2d[] { new Pose2d(), new Pose2d() };
    }

    Pose3d tagPose3d = tagPoseOpt.get();
    Translation2d towerPos = tagPose3d.toPose2d().getTranslation();

    // Tag yaw gives the direction the tag faces (toward field center)
    double tagYaw = tagPose3d.getRotation().getZ();
    Translation2d facingDir = new Translation2d(Math.cos(tagYaw), Math.sin(tagYaw));

    // Perpendicular: 90° counter-clockwise from facing direction
    Translation2d perpDir = new Translation2d(-facingDir.getY(), facingDir.getX());

    // Base position: standoff distance along the tag facing direction
    Translation2d basePos = towerPos.plus(
        facingDir.times(ClimbAlignmentConstants.kStandoffDistanceMeters));

    // Robot heading: face the Tower (opposite of tag facing direction)
    Rotation2d robotHeading = new Rotation2d(tagYaw + Math.PI);

    Translation2d leftPos = basePos.plus(
        perpDir.times(ClimbAlignmentConstants.kHookLateralOffsetMeters));
    Translation2d rightPos = basePos.plus(
        perpDir.times(-ClimbAlignmentConstants.kHookLateralOffsetMeters));

    return new Pose2d[] {
        new Pose2d(leftPos, robotHeading),
        new Pose2d(rightPos, robotHeading)
    };
  }

  /**
   * Evaluate climb alignment and publish results to SmartDashboard and
   * AdvantageScope. Call once per periodic cycle from DrivetrainSubsystem.
   *
   * @param currentPose the robot's current field-relative pose
   * @param cameras     all PhotonCamera instances on the robot
   */
  public void update(Pose2d currentPose, PhotonCamera[] cameras) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    Pose2d[] climbPoses = isRed ? redClimbPoses : blueClimbPoses;
    int[] expectedTagIds = isRed
        ? ClimbAlignmentConstants.kRedTowerTagIds
        : ClimbAlignmentConstants.kBlueTowerTagIds;

    // --- Nearest ideal pose ---
    double distLeft = currentPose.getTranslation().getDistance(climbPoses[0].getTranslation());
    double distRight = currentPose.getTranslation().getDistance(climbPoses[1].getTranslation());
    boolean leftCloser = distLeft <= distRight;

    Pose2d idealPose = leftCloser ? climbPoses[0] : climbPoses[1];
    String nearestPost = leftCloser ? "Left" : "Right";

    // --- Pose error ---
    double translationError = currentPose.getTranslation().getDistance(idealPose.getTranslation());
    double headingErrorDeg = Math.toDegrees(
        currentPose.getRotation().minus(idealPose.getRotation()).getRadians());

    // --- Tag visibility across all cameras ---
    List<Integer> seenExpectedTags = new ArrayList<>();
    double bestAmbiguity = Double.MAX_VALUE;
    double bestArea = 0;

    for (PhotonCamera camera : cameras) {
      PhotonPipelineResult result = camera.getLatestResult();
      if (!result.hasTargets()) {
        continue;
      }
      for (PhotonTrackedTarget target : result.getTargets()) {
        for (int expectedId : expectedTagIds) {
          if (target.fiducialId == expectedId && !seenExpectedTags.contains(expectedId)) {
            seenExpectedTags.add(expectedId);
            bestAmbiguity = Math.min(bestAmbiguity, target.poseAmbiguity);
            bestArea = Math.max(bestArea, target.area);
          }
        }
      }
    }

    int tagsExpected = expectedTagIds.length;
    int tagsSeen = seenExpectedTags.size();

    // --- Tag quality ---
    String tagQuality;
    if (tagsSeen == 0) {
      tagQuality = "None";
    } else if (bestAmbiguity < ClimbAlignmentConstants.kMaxAmbiguity
        && bestArea > ClimbAlignmentConstants.kMinTagArea) {
      tagQuality = "Good";
    } else if (bestAmbiguity < ClimbAlignmentConstants.kMaxAmbiguity * 2) {
      tagQuality = "Marginal";
    } else {
      tagQuality = "Poor";
    }

    // --- Alignment score (0-100) ---
    double poseComponent = Math.max(0, 1.0 - translationError / 2.0)
        * Math.max(0, 1.0 - Math.abs(headingErrorDeg) / 30.0);
    double tagComponent = (double) tagsSeen / tagsExpected;
    double qualityComponent = tagQuality.equals("Good") ? 1.0
        : tagQuality.equals("Marginal") ? 0.5 : 0.0;

    double alignmentScore = Math.round(poseComponent * 50 + tagComponent * 30 + qualityComponent * 20);
    alignmentScore = Math.max(0, Math.min(100, alignmentScore));

    // --- GO / NO-GO ---
    boolean positionOk = translationError < ClimbAlignmentConstants.kTranslationToleranceMeters;
    boolean headingOk = Math.abs(headingErrorDeg) < ClimbAlignmentConstants.kHeadingToleranceDegrees;
    boolean tagsOk = tagsSeen >= 1;
    boolean isGo = positionOk && headingOk && tagsOk;

    String reason;
    if (isGo) {
      reason = "GO: Aligned to " + nearestPost + " Post";
    } else {
      List<String> issues = new ArrayList<>();
      if (!positionOk) {
        issues.add(String.format("%.2fm off", translationError));
      }
      if (!headingOk) {
        issues.add(String.format("heading %.1f\u00B0 out", Math.abs(headingErrorDeg)));
      }
      if (!tagsOk) {
        issues.add("no Tower tags visible");
      }
      reason = "NO-GO: " + String.join(", ", issues);
    }

    // ── Primary outputs (keep) ──
    SmartDashboard.putBoolean("Climb/GO", isGo);
    SmartDashboard.putString("Climb/Reason", reason);

    // ── Diagnostic outputs — trim these after testing is complete ──
    SmartDashboard.putNumber("Climb/AlignmentScore", alignmentScore);
    SmartDashboard.putString("Climb/NearestPost", nearestPost);
    SmartDashboard.putNumber("Climb/TranslationErrorMeters", translationError);
    SmartDashboard.putNumber("Climb/HeadingErrorDeg", headingErrorDeg);
    SmartDashboard.putString("Climb/TagsSeenRatio", tagsSeen + "/" + tagsExpected);
    SmartDashboard.putString("Climb/TagQuality", tagQuality);

    // ── AdvantageScope logging (always keep for post-match analysis) ──
    Logger.recordOutput("Climb/GO", isGo);
    Logger.recordOutput("Climb/Reason", reason);
    Logger.recordOutput("Climb/AlignmentScore", alignmentScore);
    Logger.recordOutput("Climb/NearestPost", nearestPost);
    Logger.recordOutput("Climb/TranslationErrorMeters", translationError);
    Logger.recordOutput("Climb/HeadingErrorDeg", headingErrorDeg);
    Logger.recordOutput("Climb/TagsSeen", tagsSeen);
    Logger.recordOutput("Climb/TagsExpected", tagsExpected);
    Logger.recordOutput("Climb/TagQuality", tagQuality);
    Logger.recordOutput("Climb/IdealPose", idealPose);
  }
}
