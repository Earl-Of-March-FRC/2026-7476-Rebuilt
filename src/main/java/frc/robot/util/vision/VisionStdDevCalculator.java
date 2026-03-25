package frc.robot.util.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.PhotonConstants;

/**
 * Utility class for calculating dynamic vision measurement standard deviations.
 * Standard deviations increase with distance, ambiguity, and decrease with more
 * targets.
 *
 * <p>
 * For multi-tag targeting (MULTI_TAG_PNP_ON_COPROCESSOR), this calculator
 * adjusts trust levels based on:
 * <ul>
 * <li>Average distance to all visible tags</li>
 * <li>Number of tags in the measurement</li>
 * <li>Maximum ambiguity among all tags</li>
 * <li>Per-camera {@code dynamicStdDevScaleFactor} (from
 * {@link CameraProfile})</li>
 * </ul>
 *
 * <p>
 * The per-camera scale factor is the primary knob for reducing jitter from
 * cameras that are noisy or mounted at awkward angles (e.g. the Launcher
 * camera). A value of 1.0 keeps default behaviour; values above 1.0 make that
 * camera progressively less trusted.
 */
public class VisionStdDevCalculator {

  /**
   * Calculates dynamic standard deviations based on the estimated pose and
   * targets used. Uses global base std devs from {@link PhotonConstants}.
   *
   * @param estimatedPose The estimated robot pose from vision
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose) {
    return calculateStdDevs(estimatedPose, 1.0);
  }

  /**
   * Calculates dynamic standard deviations with a per-camera scale factor
   * applied on top of the distance / ambiguity / multi-tag logic.
   *
   * <p>
   * This overload is the one called from {@code DrivetrainSubsystem} when
   * {@code kUseDynamicStandardDeviations} is true but no per-camera base std devs
   * are available beyond the global constants.
   *
   * @param estimatedPose            The estimated robot pose from vision
   * @param dynamicStdDevScaleFactor Per-camera multiplier (&ge; 1.0 to reduce
   *                                 trust)
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose,
      double dynamicStdDevScaleFactor) {

    int numTargets = estimatedPose.targetsUsed.size();

    double xyStdDev = PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double thetaStdDev = PhotonConstants.kVisionBaseThetaStdDev.in(Radians);

    double avgDistance = 0.0;
    double maxAmbiguity = 0.0;

    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      avgDistance += distance;
      maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
    }

    if (numTargets > 0) {
      avgDistance /= numTargets;
    }

    // Distance-based scaling
    double distanceMultiplier = calculateDistanceMultiplier(avgDistance);
    xyStdDev *= distanceMultiplier;
    thetaStdDev *= distanceMultiplier;

    // Multi-tag bonus
    if (numTargets >= 2) {
      double multiTagBonus = 1.0 / Math.sqrt(numTargets);
      xyStdDev *= multiTagBonus;
      thetaStdDev *= multiTagBonus;
    }

    // Ambiguity penalty — smooth exponential, not a hard step
    double ambiguityMultiplier = calculateAmbiguityMultiplier(maxAmbiguity);
    xyStdDev *= ambiguityMultiplier;
    thetaStdDev *= ambiguityMultiplier;

    // Per-camera scale factor — the primary knob for jittery cameras
    xyStdDev *= dynamicStdDevScaleFactor;
    thetaStdDev *= dynamicStdDevScaleFactor;

    // Cap to prevent extreme values
    xyStdDev = Math.min(xyStdDev, PhotonConstants.kVisionMaxStdDev);
    thetaStdDev = Math.min(thetaStdDev, PhotonConstants.kVisionMaxStdDev);

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  /**
   * Calculates standard deviations with custom base values from a camera profile.
   * The camera's base std devs act as a scaling factor relative to the global
   * base, and the profile's {@code dynamicStdDevScaleFactor} is applied on top.
   *
   * <p>
   * For example: if Camera A has base std dev 0.3 and Camera B has 0.9, Camera B
   * is trusted 3× less. If Camera B also has {@code dynamicStdDevScaleFactor}
   * of 2.0 its measurements are trusted 6× less than Camera A at the same
   * distance.
   *
   * @param estimatedPose The estimated robot pose from vision
   * @param cameraProfile The full camera profile (provides both base std devs
   *                      and the dynamic scale factor)
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose,
      CameraProfile cameraProfile) {
    return calculateStdDevs(estimatedPose, cameraProfile.standardDeviation(),
        cameraProfile.dynamicStdDevScaleFactor());
  }

  /**
   * Calculates standard deviations with custom base values and an explicit scale
   * factor. This is the full-control overload.
   *
   * @param estimatedPose            The estimated robot pose from vision
   * @param cameraBaseStdDevs        Base std devs for this camera [x, y, theta]
   * @param dynamicStdDevScaleFactor Per-camera multiplier applied after all other
   *                                 adjustments
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose,
      Vector<N3> cameraBaseStdDevs,
      double dynamicStdDevScaleFactor) {

    // Get the base dynamic result (uses global base std devs internally)
    Vector<N3> dynamicStdDevs = calculateStdDevs(estimatedPose, dynamicStdDevScaleFactor);

    // Scale by the ratio of this camera's base to the global base so cameras
    // with higher base std devs remain proportionally less trusted
    double xScale = cameraBaseStdDevs.get(0) / PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double yScale = cameraBaseStdDevs.get(1) / PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double thetaScale = cameraBaseStdDevs.get(2) / PhotonConstants.kVisionBaseThetaStdDev.in(Radians);

    return VecBuilder.fill(
        Math.min(dynamicStdDevs.get(0) * xScale, PhotonConstants.kVisionMaxStdDev),
        Math.min(dynamicStdDevs.get(1) * yScale, PhotonConstants.kVisionMaxStdDev),
        Math.min(dynamicStdDevs.get(2) * thetaScale, PhotonConstants.kVisionMaxStdDev));
  }

  /**
   * Backwards-compatible overload that accepts base std devs without a scale
   * factor. Equivalent to calling
   * {@link #calculateStdDevs(EstimatedRobotPose, Vector, double)} with a scale
   * factor of 1.0.
   *
   * @param estimatedPose     The estimated robot pose from vision
   * @param cameraBaseStdDevs Base std devs for this camera [x, y, theta]
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose,
      Vector<N3> cameraBaseStdDevs) {
    return calculateStdDevs(estimatedPose, cameraBaseStdDevs, 1.0);
  }

  /**
   * Calculates the distance multiplier based on average distance to targets.
   *
   * <p>
   * Uses a quadratic curve because vision pose error grows roughly with the
   * square of distance: the projected tag shrinks proportionally to distance,
   * so a fixed pixel-level corner-detection error maps to a pose error that
   * scales as d². A linear curve under-penalises far measurements and
   * over-penalises close ones.
   *
   * <p>
   * The curve is anchored so that:
   * <ul>
   * <li>At 0 m the multiplier is {@code kVisionCloseMultiplier} (< 1.0,
   * extra trust very close up)</li>
   * <li>At {@code kVisionCloseDistance} the multiplier passes through
   * 1.0</li>
   * <li>Beyond {@code kVisionFarDistance} it continues growing
   * quadratically, scaled by {@code kVisionDistanceScaleFactor}</li>
   * </ul>
   *
   * @param avgDistance Average distance to all targets in meters
   * @return Multiplier to apply to standard deviations (1.0 = no change)
   */
  private static double calculateDistanceMultiplier(double avgDistance) {
    double closeDistance = PhotonConstants.kVisionCloseDistance.in(Meters);

    // Quadratic: multiplier = scaleFactor * d^2 + 1
    // Anchored so that multiplier == 1.0 at d == closeDistance:
    // scaleFactor * closeDistance^2 + offset == 1.0
    // => offset = 1.0 - scaleFactor * closeDistance^2

    // kVisionDistanceScaleFactor controls how steeply trust decays with
    // distance. The default value in PhotonConstants is tuned so that at
    // kVisionFarDistance the multiplier is roughly 3–4x.
    double k = PhotonConstants.kVisionDistanceScaleFactor;
    double offset = 1.0 - k * closeDistance * closeDistance;
    double multiplier = k * avgDistance * avgDistance + offset;

    // Never go below the minimum trust floor (avoids negative or tiny values
    // at very short range where the quadratic dips below kVisionCloseMultiplier)
    return Math.max(multiplier, PhotonConstants.kVisionCloseMultiplier);
  }

  /**
   * Converts a raw ambiguity value [0, 1] into a std dev penalty multiplier.
   *
   * <p>
   * Rather than a hard step (was: 1.0 or 1.5), this uses an exponential so the
   * penalty grows smoothly as ambiguity approaches 1. At
   * {@code kVisionHighAmbiguityThreshold} the multiplier equals
   * {@code kVisionHighAmbiguityMultiplier}; below that it approaches 1.0
   * asymptotically.
   *
   * @param ambiguity Raw PhotonVision pose ambiguity in [0, 1]
   * @return Penalty multiplier (&ge; 1.0)
   */
  private static double calculateAmbiguityMultiplier(double ambiguity) {
    if (ambiguity <= 0.0) {
      return 1.0;
    }

    // Exponential: penalty = exp(k * ambiguity) normalised so that
    // penalty == kVisionHighAmbiguityMultiplier at the threshold.
    //
    // k = ln(targetMultiplier) / threshold
    double threshold = PhotonConstants.kVisionHighAmbiguityThreshold;
    double targetMultiplier = PhotonConstants.kVisionHighAmbiguityMultiplier;
    double k = Math.log(targetMultiplier) / threshold;

    return Math.exp(k * ambiguity);
  }
}