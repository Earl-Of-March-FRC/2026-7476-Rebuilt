package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

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
    return pose.getX() >= 0 && pose.getX() <= FieldConstants.kFieldLengthX && pose.getY() >= 0
        && pose.getY() <= FieldConstants.kFieldWidthY;
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
  public static boolean isOnGround(Pose3d pose, double heighTolerance) {
    return pose.getZ() <= heighTolerance && pose.getZ() >= -heighTolerance;
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
   * Test if a robot is on the blue side of the field
   * Note that this method does not consider the width of the robot. The robot can
   * be outside the field boundaries.
   * 
   * @param pose A robot pose
   * @return True if the robot is on the blue side of the field
   */
  public static boolean isOnBlueSide(Pose3d pose) {
    double robotX = pose.getX();
    return robotX - FieldConstants.kBargeX < 0;
  }

  /**
   * Test if a robot is on the blue side of the field
   * Note that this method does not consider the width of the robot. The robot can
   * be outside the field boundaries.
   * 
   * @param pose A robot pose
   * @return True if the robot is on the blue side of the field
   */
  public static boolean isOnBlueSide(Pose2d pose) {
    return isOnBlueSide(new Pose3d(pose));
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

}
