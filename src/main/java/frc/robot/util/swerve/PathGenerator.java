package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Helper class to generate paths on the fly
 */
public class PathGenerator {
  private final DrivetrainSubsystem driveSub;

  /**
   * Initialize a path generator for a drivetrain subsystem
   * 
   * @param driveSub The drivetrain to use in commands
   */
  public PathGenerator(DrivetrainSubsystem driveSub) {
    this.driveSub = driveSub;
  }

  /**
   * Create a command to drive to the nearest trench and cross to the other side
   * 
   * @param endVelocity the desried end velocity after crossing
   * @return The command to schedule
   */
  public Command crossNearestTrench(LinearVelocity endVelocity) {
    PathPlannerPath trenchPath = crossTrenchPath(endVelocity, nearestIndex(FieldConstants.kTrenchPathWaypoints));

    return AutoBuilder.pathfindThenFollowPath(trenchPath, SwerveConfig.kPathfindingConstraints);
  }

  /**
   * Generate a path that drives over a trench
   * 
   * @param endVelocity Desired end velocity
   * @param trenchID    Encodes which trench to cross, and from which side the
   *                    robot
   *                    start (IDs set in FieldConstants)
   * @return A path to cross the trench
   */
  private PathPlannerPath crossTrenchPath(LinearVelocity endVelocity, int trenchID) {
    // Calculate The heading with which to cross the trench
    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kTrenchHeadingRestriction.in(Degrees)), false);

    // Define the start and end poses
    int nearestTrenchIndex = trenchID;
    Pose2d start = new Pose2d(FieldConstants.kTrenchPathWaypoints[nearestTrenchIndex],
        targetHeading);
    Pose2d end = new Pose2d(
        FieldConstants.kTrenchPathWaypoints[nearestTrenchIndex < 4 ? nearestTrenchIndex + 4 : nearestTrenchIndex - 4],
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kBumpConstraints,
        null,
        new GoalEndState(endVelocity, targetHeading));

    return path;
  }

  /**
   * Create a command to drive to the nearest bump and cross to the other side
   * 
   * @param endVelocity the desried end velocity after crossing
   * @return The command to schedule
   */
  public Command crossNearestBump(LinearVelocity endVelocity) {
    PathPlannerPath bumpPath = crossBumpPath(endVelocity, nearestIndex(FieldConstants.kBumpPathWaypoints));

    return AutoBuilder.pathfindThenFollowPath(bumpPath, SwerveConfig.kPathfindingConstraints);
  }

  /**
   * Generate a path that drives over a bump
   * 
   * @param endVelocity Desired end velocity
   * @param bumpID      Encodes which bump to cross, and from which side the robot
   *                    start (IDs set in FieldConstants)
   * @return A path to cross the bump
   */
  private PathPlannerPath crossBumpPath(LinearVelocity endVelocity, int bumpID) {
    // Calculate The heading with which to cross the bump
    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kBumpHeadingRestriction.in(Degrees)), true);

    // Define the start and end poses
    int nearestBumpIndex = bumpID;
    Pose2d start = new Pose2d(FieldConstants.kBumpPathWaypoints[nearestBumpIndex],
        targetHeading);
    Pose2d end = new Pose2d(
        FieldConstants.kBumpPathWaypoints[nearestBumpIndex < 4 ? nearestBumpIndex + 4 : nearestBumpIndex - 4],
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kBumpConstraints,
        null,
        new GoalEndState(endVelocity, targetHeading));

    return path;
  }

  /**
   * Helper method
   * Takes a list of Translation2ds and returns the index of the one closest to
   * the bot
   * 
   * @param poseTranslation2ds The list of Translation2ds
   * @return The index of the nearest one
   */
  private int nearestIndex(Translation2d[] poseTranslation2ds) {
    int nearestIndex = 0;

    Translation2d currTranslation = driveSub.getPose().getTranslation();
    for (int i = 0; i < poseTranslation2ds.length; i++) {
      double d1 = currTranslation.getDistance(poseTranslation2ds[i]);
      double d2 = currTranslation.getDistance(poseTranslation2ds[nearestIndex]);

      if (d1 < d2) {
        nearestIndex = i;
      }
    }
    return nearestIndex;
  }
}
