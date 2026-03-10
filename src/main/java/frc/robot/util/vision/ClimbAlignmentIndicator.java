package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbAlignmentConstants;
import frc.robot.Constants.FieldConstants;

public class ClimbAlignmentIndicator {

  // Enums
  public enum TagQuality {
    NONE(0.0),
    POOR(0.0),
    MARGINAL(0.5),
    GOOD(1.0);

    // Weight applied to the quality component of the alignment score.
    public final double scoreWeight;

    TagQuality(double scoreWeight) {
      this.scoreWeight = scoreWeight;
    }

    /**
     * Classify tag quality from raw photon metrics.
     *
     * @param tagsSeen  number of expected tower tags currently visible
     * @param ambiguity best (lowest) pose ambiguity across visible tags
     * @param area      best (highest) tag area across visible tags
     */
    public static TagQuality classify(int tagsSeen, double ambiguity, double area) {
      if (tagsSeen == 0)
        return NONE;
      if (ambiguity < ClimbAlignmentConstants.kMaxAmbiguity
          && area > ClimbAlignmentConstants.kMinTagArea)
        return GOOD;
      if (ambiguity < ClimbAlignmentConstants.kMaxAmbiguity * 2)
        return MARGINAL;
      return POOR;
    }
  }

  public enum PostSide {
    LEFT, RIGHT;

    // Human-readable label for elastic display.
    public String label() {
      return name().charAt(0) + name().substring(1).toLowerCase(); // Left/Right
    }
  }

  public enum AlignmentStatus {
    GO, NO_GO;

    public boolean isGo() {
      return this == GO;
    }
  }

  // One alignment evaluation cycle.
  public record AlignmentResult(
      AlignmentStatus status,
      PostSide nearestPost,
      double translationErrorMeters,
      double headingErrorDegrees,
      int tagsSeen,
      int tagsExpected,
      TagQuality tagQuality,
      double alignmentScore,
      Pose2d idealPose,
      String reason) {
  }

  // Fields
  // [0] = left post, [1] = right post for each alliance.
  private final Pose2d[] blueClimbPoses;
  private final Pose2d[] redClimbPoses;

  public ClimbAlignmentIndicator() {
    blueClimbPoses = computeClimbPoses(ClimbAlignmentConstants.kBlueTowerTagIds);
    redClimbPoses = computeClimbPoses(ClimbAlignmentConstants.kRedTowerTagIds);
  }

  /**
   * Derive two ideal climb poses from the two Tower AprilTag positions.
   * Each tag independently gives one post position so that left ≠ right
   * even when the tags are at different heights / lateral offsets.
   *
   * @param towerTagIds two Tower tag IDs for the relevant alliance: [0] ->
   *                    left-post tag, [1] -> right-post tag
   * @return [0] = left post pose, [1] = right post pose
   */
  private Pose2d[] computeClimbPoses(int[] towerTagIds) {
    Pose2d[] poses = new Pose2d[2];
    for (int i = 0; i < 2; i++) {
      Optional<Pose3d> tagPoseOpt = FieldConstants.kfieldLayout.getTagPose(towerTagIds[i]);
      if (tagPoseOpt.isEmpty()) {
        poses[i] = new Pose2d();
        continue;
      }

      Pose3d tagPose3d = tagPoseOpt.get();
      Translation2d towerPos = tagPose3d.toPose2d().getTranslation();

      double tagYaw = tagPose3d.getRotation().getZ();
      Translation2d facingDir = new Translation2d(Math.cos(tagYaw), Math.sin(tagYaw));

      // Stand off along the tag's facing direction
      Translation2d standoffPos = towerPos.plus(
          facingDir.times(ClimbAlignmentConstants.kStandoffDistanceMeters));

      // Robot faces the tower (opposite of tag-facing direction)
      Rotation2d robotHeading = new Rotation2d(tagYaw + Math.PI);

      poses[i] = new Pose2d(standoffPos, robotHeading);
    }
    return poses;
  }

  /**
   * Evaluate climb alignment and publish results to SmartDashboard /
   * AdvantageScope. Call once per periodic cycle from DrivetrainSubsystem.
   *
   * @param currentPose field-relative robot pose
   * @param cameras     all PhotonCamera instances on the robot
   * @return the full alignment result for this cycle
   */
  public AlignmentResult update(Pose2d currentPose, PhotonCamera[] cameras) {
    AlignmentResult result = evaluate(currentPose, cameras);
    publish(result);
    return result;
  }

  private AlignmentResult evaluate(Pose2d currentPose, PhotonCamera[] cameras) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    Pose2d[] climbPoses = isRed ? redClimbPoses : blueClimbPoses;
    int[] expectedTagIds = isRed
        ? ClimbAlignmentConstants.kRedTowerTagIds
        : ClimbAlignmentConstants.kBlueTowerTagIds;

    // Nearest post
    double distLeft = currentPose.getTranslation().getDistance(climbPoses[PostSide.LEFT.ordinal()].getTranslation());
    double distRight = currentPose.getTranslation().getDistance(climbPoses[PostSide.RIGHT.ordinal()].getTranslation());
    PostSide nearestPost = distLeft <= distRight ? PostSide.LEFT : PostSide.RIGHT;
    Pose2d idealPose = climbPoses[nearestPost.ordinal()];

    // Pose error
    double translationError = currentPose.getTranslation().getDistance(idealPose.getTranslation());

    // Normalize heading error to [-180, 180]
    double headingErrorDeg = MathUtil.inputModulus(
        currentPose.getRotation().minus(idealPose.getRotation()).getDegrees(),
        -180.0, 180.0);

    // Tag visibility
    List<Integer> seenExpectedTags = new ArrayList<>();
    double bestAmbiguity = Double.MAX_VALUE;
    double bestArea = 0;

    for (PhotonCamera camera : cameras) {
      PhotonPipelineResult result = camera.getLatestResult();
      if (!result.hasTargets())
        continue;

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

    int tagsSeen = seenExpectedTags.size();
    int tagsExpected = expectedTagIds.length;
    TagQuality tagQuality = TagQuality.classify(tagsSeen, bestAmbiguity, bestArea);

    // Alignment score (0–100)
    // Pose component: decays linearly with translation error (zero at 2 m) and
    // heading error (zero at 30 deg). Clamped to [0, 1].
    double poseFactor = Math.max(0.0, 1.0 - translationError / 2.0)
        * Math.max(0.0, 1.0 - Math.abs(headingErrorDeg) / 30.0);
    double tagFactor = tagsExpected > 0 ? (double) tagsSeen / tagsExpected : 0.0;

    double rawScore = poseFactor * 50.0
        + tagFactor * 30.0
        + tagQuality.scoreWeight * 20.0;
    double alignmentScore = MathUtil.clamp(Math.round(rawScore), 0, 100);

    // GO / NO-GO
    boolean positionOk = translationError < ClimbAlignmentConstants.kTranslationToleranceMeters;
    boolean headingOk = Math.abs(headingErrorDeg) < ClimbAlignmentConstants.kHeadingToleranceDegrees;
    boolean tagsOk = tagsSeen >= 1;

    AlignmentStatus status = (positionOk && headingOk && tagsOk)
        ? AlignmentStatus.GO
        : AlignmentStatus.NO_GO;

    String reason = buildReason(status, nearestPost, positionOk, headingOk, tagsOk,
        translationError, headingErrorDeg);

    return new AlignmentResult(
        status, nearestPost, translationError, headingErrorDeg,
        tagsSeen, tagsExpected, tagQuality, alignmentScore, idealPose, reason);
  }

  private static String buildReason(
      AlignmentStatus status, PostSide post,
      boolean positionOk, boolean headingOk, boolean tagsOk,
      double translationError, double headingErrorDeg) {

    if (status == AlignmentStatus.GO) {
      return "GO: Aligned to " + post.label() + " Post";
    }

    List<String> issues = new ArrayList<>();
    if (!positionOk)
      issues.add(String.format("%.2fm off", translationError));
    if (!headingOk)
      issues.add(String.format("heading %.1f° out", Math.abs(headingErrorDeg)));
    if (!tagsOk)
      issues.add("no Tower tags visible");
    return "NO-GO: " + String.join(", ", issues);
  }

  // Elastic and logger publishing
  private static void publish(AlignmentResult r) {
    // Primary outputs
    SmartDashboard.putBoolean("Climb/GO", r.status().isGo());
    SmartDashboard.putString("Climb/Reason", r.reason());

    // Diagnostics; trim after testing
    SmartDashboard.putNumber("Climb/AlignmentScore", r.alignmentScore());
    SmartDashboard.putString("Climb/NearestPost", r.nearestPost().label());
    SmartDashboard.putNumber("Climb/TranslationErrorMeters", r.translationErrorMeters());
    SmartDashboard.putNumber("Climb/HeadingErrorDeg", r.headingErrorDegrees());
    SmartDashboard.putString("Climb/TagsSeenRatio", r.tagsSeen() + "/" + r.tagsExpected());
    SmartDashboard.putString("Climb/TagQuality", r.tagQuality().name());

    // AdvantageScope; always keep
    Logger.recordOutput("Climb/GO", r.status().isGo());
    Logger.recordOutput("Climb/Reason", r.reason());
    Logger.recordOutput("Climb/AlignmentScore", r.alignmentScore());
    Logger.recordOutput("Climb/NearestPost", r.nearestPost().name());
    Logger.recordOutput("Climb/TranslationErrorMeters", r.translationErrorMeters());
    Logger.recordOutput("Climb/HeadingErrorDeg", r.headingErrorDegrees());
    Logger.recordOutput("Climb/TagsSeen", r.tagsSeen());
    Logger.recordOutput("Climb/TagsExpected", r.tagsExpected());
    Logger.recordOutput("Climb/TagQuality", r.tagQuality().name());
    Logger.recordOutput("Climb/IdealPose", r.idealPose());
  }
}