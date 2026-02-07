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
 * For multi-tag targeting (MULTI_TAG_PNP_ON_COPROCESSOR), this calculator
 * adjusts
 * trust levels based on:
 * - Average distance to all visible tags
 * - Number of tags in the measurement
 * - Maximum ambiguity among all tags
 */
public class VisionStdDevCalculator {

  /**
   * Calculates dynamic standard deviations based on the estimated pose and
   * targets used.
   * 
   * For multi-tag measurements:
   * - Uses average distance to all tags (multi-tag already provides better
   * accuracy)
   * - Applies bonus based on number of tags visible
   * - Penalizes if any tag has high ambiguity
   * 
   * @param estimatedPose The estimated robot pose from vision
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose) {
    int numTargets = estimatedPose.targetsUsed.size();

    // Start with base standard deviations from constants
    double xyStdDev = PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double thetaStdDev = PhotonConstants.kVisionBaseThetaStdDev.in(Radians);

    // Calculate average distance and maximum ambiguity
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

    // Apply distance scaling
    // Closer = more trust (lower std dev), Farther = less trust (higher std dev)
    double distanceMultiplier = calculateDistanceMultiplier(avgDistance);

    xyStdDev *= distanceMultiplier;
    thetaStdDev *= distanceMultiplier;

    // Apply multi-tag bonus
    // More tags = more confidence, std dev decreases
    // Note: For multi-tag targeting, even 1 "target" may represent a multi-tag
    // solution
    if (numTargets >= 2) {
      // Multiple tags: reduce std dev by square root of number of tags
      double multiTagBonus = 1.0 / Math.sqrt(numTargets);
      xyStdDev *= multiTagBonus;
      thetaStdDev *= multiTagBonus;
    }

    // Apply ambiguity penalty if any tag has high ambiguity
    if (maxAmbiguity > PhotonConstants.kVisionHighAmbiguityThreshold) {
      xyStdDev *= PhotonConstants.kVisionHighAmbiguityMultiplier;
      thetaStdDev *= PhotonConstants.kVisionHighAmbiguityMultiplier;
    }

    // Cap the standard deviations to prevent extreme values
    xyStdDev = Math.min(xyStdDev, PhotonConstants.kVisionMaxStdDev);
    thetaStdDev = Math.min(thetaStdDev, PhotonConstants.kVisionMaxStdDev);

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  /**
   * Calculates the distance multiplier based on average distance to targets.
   * 
   * @param avgDistance Average distance to all targets in meters
   * @return Multiplier to apply to standard deviations (1.0 = no change)
   */
  private static double calculateDistanceMultiplier(double avgDistance) {
    double closeDistance = PhotonConstants.kVisionCloseDistance.in(Meters);
    double farDistance = PhotonConstants.kVisionFarDistance.in(Meters);

    if (avgDistance < closeDistance) {
      // Very close: reduce std dev linearly
      // At 0m: 0.5x multiplier, at closeDistance: 1.0x multiplier
      return 0.5 + (avgDistance / closeDistance) * 0.5;
    } else if (avgDistance > farDistance) {
      // Very far: increase std dev based on distance scale factor
      // Beyond farDistance, std dev increases linearly
      return 1.0 + PhotonConstants.kVisionDistanceScaleFactor * (avgDistance - farDistance);
    } else {
      // Medium distance: scale linearly between close and far
      // At closeDistance: 1.0x, at farDistance: 1.5x
      double normalizedDistance = (avgDistance - closeDistance) / (farDistance - closeDistance);
      return 1.0 + normalizedDistance * 0.5;
    }
  }

  /**
   * Calculates standard deviations with custom base values from camera profile.
   * Useful for cameras with different trust levels.
   * 
   * The camera's base standard deviations are used as a scaling factor.
   * For example, if Camera 1 has base std dev of 0.3 and Camera 2 has 0.9,
   * Camera 2's measurements will always be trusted 3x less than Camera 1's,
   * but both will still adjust dynamically based on measurement quality.
   * 
   * @param estimatedPose     The estimated robot pose from vision
   * @param cameraBaseStdDevs The base standard deviations for this specific
   *                          camera
   * @return Vector of standard deviations [x, y, theta]
   */
  public static Vector<N3> calculateStdDevs(EstimatedRobotPose estimatedPose, Vector<N3> cameraBaseStdDevs) {
    Vector<N3> dynamicStdDevs = calculateStdDevs(estimatedPose);

    // Scale the dynamic result by the ratio of camera's base to global base
    double xScale = cameraBaseStdDevs.get(0) / PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double yScale = cameraBaseStdDevs.get(1) / PhotonConstants.kVisionBaseXYStdDev.in(Meters);
    double thetaScale = cameraBaseStdDevs.get(2) / PhotonConstants.kVisionBaseThetaStdDev.in(Radians);

    return VecBuilder.fill(
        dynamicStdDevs.get(0) * xScale,
        dynamicStdDevs.get(1) * yScale,
        dynamicStdDevs.get(2) * thetaScale);
  }
}