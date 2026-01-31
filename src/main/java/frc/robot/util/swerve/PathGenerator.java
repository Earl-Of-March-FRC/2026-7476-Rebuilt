package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * Helper class to generate paths on the fly
 */
public class PathGenerator {
  private static DrivetrainSubsystem driveSub;

  public static void setDrivetrain(DrivetrainSubsystem driveSubsystem) {
    driveSub = driveSubsystem;
  }

  /**
   * Create a command to drive to the nearest trench and cross to the other side
   * 
   * @param endVelocity the desried end velocity after crossing
   * @return The command to schedule
   */
  public static Command crossNearestTrench(LinearVelocity endVelocity) {
    PathPlannerPath trenchPath = crossTrenchPath(endVelocity,
        nearestTranslation2dIndex(FieldConstants.kTrenchPathWaypoints));

    // return AutoBuilder.pathfindThenFollowPath(trenchPath,
    // DriveConstants.kPathfindingConstraints).until(() -> (driveSub
    // .getCurrentBotZone() == driveSub
    // .getPoseZone(trenchPath.getPathPoses().get(trenchPath.getPathPoses().size() -
    // 1))));

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
  private static PathPlannerPath crossTrenchPath(LinearVelocity endVelocity, int trenchID) {
    // Calculate The heading with which to cross the trench
    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kTrenchHeadingRestriction.in(Degrees)), false);

    // Define the start and end poses
    Pose2d start = new Pose2d(FieldConstants.kTrenchPathWaypoints[trenchID],
        targetHeading);
    Pose2d end = new Pose2d(
        FieldConstants.kTrenchPathWaypoints[trenchID < 4 ? trenchID + 4 : trenchID - 4],
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kTrenchConstraints,
        new IdealStartingState(DriveConstants.kTrenchLinearVelocity, targetHeading),
        new GoalEndState(endVelocity, targetHeading));

    return path;
  }

  /**
   * Create a command to drive to the nearest bump and cross to the other side
   * 
   * @param endVelocity the desried end velocity after crossing
   * @return The command to schedule
   */
  public static Command crossNearestBump(LinearVelocity endVelocity) {
    PathPlannerPath bumpPath = crossBumpPath(endVelocity, nearestTranslation2dIndex(FieldConstants.kBumpPathWaypoints));

    // return AutoBuilder.pathfindThenFollowPath(bumpPath,
    // DriveConstants.kPathfindingConstraints).until(() -> (driveSub
    // .getCurrentBotZone() ==
    // driveSub.getPoseZone(bumpPath.getPathPoses().get(bumpPath.getPathPoses().size()
    // - 1))));

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
  private static PathPlannerPath crossBumpPath(LinearVelocity endVelocity, int bumpID) {
    // Calculate The heading with which to cross the bump
    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kBumpHeadingRestriction.in(Degrees)), true);

    // Define the start and end poses
    Pose2d start = new Pose2d(FieldConstants.kBumpPathWaypoints[bumpID],
        targetHeading);
    Pose2d end = new Pose2d(
        FieldConstants.kBumpPathWaypoints[bumpID < 4 ? bumpID + 4 : bumpID - 4],
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kBumpConstraints,
        new IdealStartingState(DriveConstants.kBumpLinearVelocity, targetHeading),
        new GoalEndState(endVelocity, targetHeading));

    return path;
  }

  /**
   * Create a command to go from the neutral zone to the launch zone going over
   * the bump, will return
   * a command that does nothing and ends immediately if not currently in neutral
   * zone
   * 
   * @param endVelocity Desired end velocity
   * @return
   */
  public static Command driveToLaunchZoneCommandBump(LinearVelocity endVelocity) {
    if (driveSub.getCurrentBotZone() != FieldZones.Neutral) {
      return new InstantCommand();
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    if (alliance.isPresent()) {
      SmartDashboard.putString("Drivetrain/Alliance",
          isBlueAlliance ? "Blue" : "Red");
    } else {
      SmartDashboard.putString("Drivetrain/Alliance",
          "Unknown");
    }

    Translation2d[] neutralBumpTranslation2ds = isBlueAlliance
        ? new Translation2d[] { FieldConstants.kBumpPathWaypoints[4], FieldConstants.kBumpPathWaypoints[5] }
        : new Translation2d[] { FieldConstants.kBumpPathWaypoints[6], FieldConstants.kBumpPathWaypoints[7] };

    Translation2d startTranslation2d = nearestTranslation2d(neutralBumpTranslation2ds);
    Translation2d inLaunchZoneTranslation2d = new Translation2d(
        isBlueAlliance
            ? FieldConstants.kAcceptedLaunchingZone.minus(SwerveConfig.kBumperWidth).in(Meters)
            : FieldConstants.kFieldLengthX
                .minus(FieldConstants.kAcceptedLaunchingZone.minus(SwerveConfig.kBumperWidth)).in(Meters),
        startTranslation2d.getY());

    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kBumpHeadingRestriction.in(Degrees)), true);

    // Define the start and end poses
    Pose2d start = new Pose2d(startTranslation2d,
        targetHeading);
    Pose2d end = new Pose2d(inLaunchZoneTranslation2d,
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kBumpConstraints,
        new IdealStartingState(DriveConstants.kBumpLinearVelocity, targetHeading),
        new GoalEndState(endVelocity, targetHeading));

    return AutoBuilder.pathfindThenFollowPath(path, SwerveConfig.kPathfindingConstraints)
        .until(() -> (driveSub.getCurrentBotZone() == FieldZones.Launch));
  }

  /**
   * Create a command to go from the neutral zone to the launch zone going over
   * the trench, will return
   * a command that does nothing and ends immediately if not currently in neutral
   * zone
   * 
   * @param endVelocity Desired end velocity
   * @return
   */
  public static Command driveToLaunchZoneCommandTrench(LinearVelocity endVelocity) {
    if (driveSub.getCurrentBotZone() != FieldZones.Neutral) {
      return new InstantCommand();
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    if (alliance.isPresent()) {
      SmartDashboard.putString("Drivetrain/Alliance",
          isBlueAlliance ? "Blue" : "Red");
    } else {
      SmartDashboard.putString("Drivetrain/Alliance",
          "Unknown");
    }

    Translation2d[] neutralTrenchTranslation2ds = isBlueAlliance
        ? new Translation2d[] { FieldConstants.kTrenchPathWaypoints[4], FieldConstants.kTrenchPathWaypoints[5] }
        : new Translation2d[] { FieldConstants.kTrenchPathWaypoints[6], FieldConstants.kTrenchPathWaypoints[7] };

    Translation2d startTranslation2d = nearestTranslation2d(neutralTrenchTranslation2ds);
    Translation2d inLaunchZoneTranslation2d = new Translation2d(
        isBlueAlliance
            ? FieldConstants.kAcceptedLaunchingZone.minus(SwerveConfig.kBumperWidth).in(Meters)
            : FieldConstants.kFieldLengthX
                .minus(FieldConstants.kAcceptedLaunchingZone.minus(SwerveConfig.kBumperWidth)).in(Meters),
        startTranslation2d.getY());

    Rotation2d targetHeading = driveSub
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kTrenchHeadingRestriction.in(Degrees)), true);

    // Define the start and end poses
    Pose2d start = new Pose2d(startTranslation2d,
        targetHeading);
    Pose2d end = new Pose2d(inLaunchZoneTranslation2d,
        targetHeading);

    // Convert to waypoint list
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(start, end);

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        DriveConstants.kTrenchConstraints,
        new IdealStartingState(DriveConstants.kTrenchLinearVelocity, targetHeading),
        new GoalEndState(endVelocity, targetHeading));

    return AutoBuilder.pathfindThenFollowPath(path, SwerveConfig.kPathfindingConstraints)
        .until(() -> (driveSub.getCurrentBotZone() == FieldZones.Launch));
  }

  /**
   * Helper method
   * Takes a list of Translation2ds and returns the index of the one closest to
   * the bot
   * 
   * @param poseTranslation2ds The list of Translation2ds
   * @return The index of the nearest one
   */
  private static int nearestTranslation2dIndex(Translation2d[] poseTranslation2ds) {
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

  /**
   * Helper method
   * Takes a list of Translation2ds and returns the one closest to
   * the bot
   * 
   * @param poseTranslation2ds The list of Translation2ds
   * @return The nearest one
   */
  private static Translation2d nearestTranslation2d(Translation2d[] poseTranslation2ds) {
    return poseTranslation2ds[nearestTranslation2dIndex(poseTranslation2ds)];
  }

}
