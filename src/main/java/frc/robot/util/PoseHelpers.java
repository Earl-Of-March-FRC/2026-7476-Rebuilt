package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.vision.CameraProfile;

public class PoseHelpers {

  /**
   * Returns true if the robot is within the field boundaries
   * Note that this method does not consider the width of the robot. The height of
   * the pose is also ignored.
   * 
   * @param pose A robot pose
   * @return True if the pose is within the field boundaries
   */
  public static boolean isInField(Pose3d pose) {
    return pose.getX() >= 0 && pose.getX() <= FieldConstants.kFieldLengthX.in(Meters) && pose.getY() >= 0
        && pose.getY() <= FieldConstants.kFieldWidthY.in(Meters);
  }

  /**
   * Returns true if the robot is within the field boundaries
   * Note that this method does not consider the width of the robot. The height of
   * the pose is also ignored.
   * 
   * @param pose A robot pose
   * @return True if the pose is within the field boundaries
   */
  public static boolean isInField(Pose2d pose) {
    return isInField(new Pose3d(pose));
  }

  /**
   * Test if a robot pose is on the ground
   * 
   * @param pose           A robot pose
   * @param heighTolerance The max distance from the ground the robot can be (both
   *                       sides)
   * @return True if the pose is on the ground
   */
  public static boolean isOnGround(Pose3d pose, Distance heighTolerance) {
    // return pose.getZ() <= heighTolerance && pose.getZ() >= -heighTolerance;
    return true;
  }

  /**
   * Calculates the distance between two poses
   * 
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @return The distance between the two poses, in meters
   */
  public static double distanceBetween(Pose3d pose1, Pose3d pose2) {
    return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2)
        + Math.pow(pose1.getZ() - pose2.getZ(), 2));
  }

  /**
   * Calculates the distance between two poses
   * 
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @return The distance between the two poses, in meters
   */
  public static double distanceBetween(Pose2d pose1, Pose2d pose2) {
    return distanceBetween(new Pose3d(pose1), new Pose3d(pose2));
  }

  /**
   * Converts a Pose2d to a Pose3d
   * 
   * @param pose A Pose2d
   * @return The Pose3d equivalent
   */
  public static Pose2d toPose2d(Pose3d pose) {
    return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getZ()));
  }

  /**
   * Takes a list of Translation2ds and returns the index of the one closest to
   * the given pose
   * 
   * @param poseTranslation2ds The list of Translation2ds
   * @param pose               The pose to measure distances against
   * @return The index of the nearest one
   */
  public static int nearestTranslation2dIndex(Translation2d[] poseTranslation2ds, Pose2d pose) {
    int nearestIndex = 0;

    Translation2d currTranslation = pose.getTranslation();
    for (int i = 0; i < poseTranslation2ds.length; i++) {
      double d1 = currTranslation.getDistance(poseTranslation2ds[i]);
      double d2 = currTranslation.getDistance(poseTranslation2ds[nearestIndex]);

      if (d1 < d2) {
        nearestIndex = i;
      }
    }
    return nearestIndex;
  }

  /**
   * Takes a list of Translation2ds and returns the one closest to
   * the given pose
   * 
   * @param poseTranslation2ds The list of Translation2ds
   * @param pose               The pose to measure distances against
   * @return The nearest one
   */
  public static Translation2d nearestTranslation2d(Translation2d[] poseTranslation2ds, Pose2d pose) {
    return poseTranslation2ds[nearestTranslation2dIndex(poseTranslation2ds, pose)];
  }

  /**
   * Returns the Y coordinate of the center of the trench nearest the given pose
   * 
   * @param pose The current pose
   * @return The Y coordinate of the nearest trench
   */
  public static Distance nearestTrenchY(Pose2d pose) {
    // Default to top
    if (pose.getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      return FieldConstants.kEdgeToTrenchCenter;
    } else {
      return FieldConstants.kFieldWidthY.minus(FieldConstants.kEdgeToTrenchCenter);
    }
  }

  /**
   * Returns the Y coordinate of the center of the bump nearest the given pose
   * 
   * @param pose The current pose
   * @return The Y coordinate of the nearest trench
   */
  public static Distance nearestBumpY(Pose2d pose) {
    // Default to top
    if (pose.getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      return FieldConstants.kEdgeToBumpCenter;
    } else {
      return FieldConstants.kFieldWidthY.minus(FieldConstants.kEdgeToBumpCenter);
    }
  }

  /**
   * Calculate a camera's global position (its position relative to the field)
   * 
   * @param globalBotPose Current robot field-relative position
   * @param cameraProfile Selected camera profile
   * @return Field-relative camera pose
   */
  public static Pose3d calculateGlobalCameraPose(Pose3d globalBotPose, CameraProfile cameraProfile) {
    return globalBotPose.transformBy(
        cameraProfile.getRobotToCameraTransform());
  }
}
