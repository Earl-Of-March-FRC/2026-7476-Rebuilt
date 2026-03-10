package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.PoseHelpers;
import java.util.Arrays;

/**
 * Helper class to generate paths on the fly
 */
public class PathGenerator {
  // Do not access driveSub directly, use drive() to avoid NPE
  private static DrivetrainSubsystem driveSub;
  private static boolean configured = false;

  public static void setDrivetrain(DrivetrainSubsystem driveSubsystem) {
    if (configured) {
      throw new IllegalStateException("PathGenerator already configured");
    }
    driveSub = Objects.requireNonNull(driveSubsystem, "Drivetrain cannot be null");
    configured = true;
  }

  private static DrivetrainSubsystem drive() {
    if (!configured) {
      throw new IllegalStateException(
          "PathGenerator used before setDrivetrain() was called");
    }
    return driveSub;
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

    // return pathfindThenFollowPathNoFlip(trenchPath,
    // DriveConstants.kPathfindingConstraints).until(() -> (drive()
    // .getCurrentBotZone() == drive()
    // .getPoseZone(trenchPath.getPathPoses().get(trenchPath.getPathPoses().size() -
    // 1))));

    return pathfindThenFollowPathNoFlip(trenchPath, SwerveConfig.kPathfindingConstraints);
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
    Rotation2d targetHeading = drive()
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

    // return pathfindThenFollowPathNoFlip(bumpPath,
    // DriveConstants.kPathfindingConstraints).until(() -> (drive()
    // .getCurrentBotZone() ==
    // drive().getPoseZone(bumpPath.getPathPoses().get(bumpPath.getPathPoses().size()
    // - 1))));

    return pathfindThenFollowPathNoFlip(bumpPath, SwerveConfig.kPathfindingConstraints);
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
    Rotation2d targetHeading = drive()
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
    if (drive().getCurrentBotZone() != FieldZones.Neutral) {
      return new InstantCommand();
    }

    boolean isBlueAlliance = PoseHelpers.getAlliance() == Alliance.Blue;

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

    Rotation2d targetHeading = drive()
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

    return pathfindThenFollowPathNoFlip(path, SwerveConfig.kPathfindingConstraints)
        .until(() -> (drive().getCurrentBotZone() == FieldZones.Launch));
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
    if (drive().getCurrentBotZone() != FieldZones.Neutral) {
      return new InstantCommand();
    }

    boolean isBlueAlliance = PoseHelpers.getAlliance() == Alliance.Blue;

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

    Rotation2d targetHeading = drive()
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

    return pathfindThenFollowPathNoFlip(path, SwerveConfig.kPathfindingConstraints)
        .until(() -> (drive().getCurrentBotZone() == FieldZones.Launch));
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

    Translation2d currTranslation = drive().getPose().getTranslation();
    for (int i = 0; i < poseTranslation2ds.length; i++) {
      double d1 = currTranslation.getDistance(poseTranslation2ds[i]);
      double d2 = currTranslation.getDistance(poseTranslation2ds[nearestIndex]);

      if (d1 < d2) {
        nearestIndex = i;
      }
    }
    return nearestIndex;
  }

  public static Command crossBumpAuto(Translation2d[] bumpWaypoints) {
    Rotation2d targetHeading = drive()
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kBumpHeadingRestriction.in(Degrees)), true);

    int bumpID = nearestTranslation2dIndex(bumpWaypoints);
    Pose2d start = new Pose2d(FieldConstants.kBumpPathWaypoints[bumpID], targetHeading);
    Pose2d end = new Pose2d(FieldConstants.kBumpPathWaypoints[bumpID < 4 ? bumpID + 4 : bumpID - 4], targetHeading);

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    int directionMultiplier;
    BooleanSupplier isCrossingFinished;

    if (start.getX() < end.getX()) {
      directionMultiplier = isBlueAlliance ? 1 : -1;
      isCrossingFinished = () -> drive().getPose().getX() >= end.getX();
    } else {
      directionMultiplier = isBlueAlliance ? -1 : 1;
      isCrossingFinished = () -> drive().getPose().getX() <= end.getX();
    }

    return AutoBuilder.pathfindToPose(start, SwerveConfig.kPathfindingConstraints)
        .andThen(new DriveLockedHeadingAndYCmd(driveSub, () -> 1.0 * directionMultiplier, // ?
            () -> PoseHelpers.nearestBumpY(start), targetHeading))
        .until(isCrossingFinished);
  }

  public static Command crossTrenchAuto(Translation2d[] trenchWaypoints) {
    Rotation2d targetHeading = drive()
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kTrenchHeadingRestriction.in(Degrees)), false);

    int trenchID = nearestTranslation2dIndex(trenchWaypoints);
    Pose2d start = new Pose2d(FieldConstants.kTrenchPathWaypoints[trenchID], targetHeading);
    Pose2d end = new Pose2d(FieldConstants.kTrenchPathWaypoints[trenchID < 4 ? trenchID + 4 : trenchID - 4],
        targetHeading);

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    int directionMultiplier;
    BooleanSupplier isCrossingFinished;

    if (start.getX() < end.getX()) {
      directionMultiplier = isBlueAlliance ? 1 : -1;
      isCrossingFinished = () -> drive().getPose().getX() >= end.getX();
    } else {
      directionMultiplier = isBlueAlliance ? -1 : 1;
      isCrossingFinished = () -> drive().getPose().getX() <= end.getX();
    }

    return AutoBuilder.pathfindToPose(start, SwerveConfig.kPathfindingConstraints)
        .andThen(new DriveLockedHeadingAndYCmd(driveSub, () -> 1.0 * directionMultiplier, // ?
            () -> PoseHelpers.nearestTrenchY(start), targetHeading))
        .until(isCrossingFinished);
  }

  /**
   * Load the L1 climb path and return a pathfinding command for it.
   * If loading fails the method returns an InstantCommand so callers can
   * schedule it without null checks.
   */
  public static Command loadL1ClimbCommand() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    int nearestPathIndex;

    if (isBlueAlliance) {
      nearestPathIndex = nearestTranslation2dIndex(
          Arrays.copyOfRange(AutoConstants.climbPathWaypoints, 0, 2));
    } else {
      nearestPathIndex = nearestTranslation2dIndex(
          Arrays.copyOfRange(AutoConstants.climbPathWaypoints, 2, 4));
    }

    // Load the path we want to pathfind to and follow
    PathPlannerPath nearestClimbPath = AutoConstants.climbPaths[nearestPathIndex];

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindThenFollowPath(nearestClimbPath, AutoConstants.L1ClimbConstraints);
  }

  /**
   * Creates a command to cross from the neutral zone to our alliance side
   * depending on the passed in crossing method (either bump or trench).
   * The command ends immediately if the robot is not in neutral zone (already on
   * alliance side)
   * 
   * @param endVelocity    Desired end velocity
   * @param crossingMethod Either "Bump" or "Trench"
   * @return A path to cross from netural zone to alliance side
   */
  public static Command crossToAllianceSide(LinearVelocity endVelocity, String crossingMethod) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    if (drive().getCurrentBotZone() != FieldZones.Neutral) {
      return new InstantCommand();
    }

    int neutralZoneWaypointId;
    PathPlannerPath crossingPath;

    if (isBlueAlliance) {
      neutralZoneWaypointId = 4;
    } else {
      neutralZoneWaypointId = 6;
    }

    if (crossingMethod.equalsIgnoreCase("Bump")) {
      int nearestBumpInNeutralZone = nearestTranslation2dIndex(
          Arrays.copyOfRange(FieldConstants.kBumpPathWaypoints, neutralZoneWaypointId, neutralZoneWaypointId + 2));
      crossingPath = crossBumpPath(endVelocity, neutralZoneWaypointId + nearestBumpInNeutralZone);
    } else {
      int nearestTrenchInNeutralZone = nearestTranslation2dIndex(
          Arrays.copyOfRange(FieldConstants.kTrenchPathWaypoints, neutralZoneWaypointId, neutralZoneWaypointId + 2));
      crossingPath = crossTrenchPath(endVelocity, neutralZoneWaypointId + nearestTrenchInNeutralZone);
    }

    return pathfindThenFollowPathNoFlip(crossingPath, SwerveConfig.kPathfindingConstraints);
  }

  /**
   * Creates a sequential command that finds the shortest path to the climb zone
   * 
   * @param endVelocity    Desired end velocity
   * @param crossingMethod Either "Bump" or "Trench"
   * @return A command group that runs the shortest path to the clime zone
   */
  public static SequentialCommandGroup findL1ClimbPath(LinearVelocity endVelocity, String crossingMethod) {
    return new SequentialCommandGroup(
        Commands.defer(
            () -> crossToAllianceSide(endVelocity, crossingMethod),
            Set.of(driveSub)),
        Commands.defer(
            () -> loadL1ClimbCommand(),
            Set.of(driveSub)));
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

  /**
   * Autobuilder pathfinding will always apply the shouldFlip supplier when using
   * pathfindThenFollowPath(), even if the path.preventFlipping = true. The only
   * way to circumvent this is to manually compose pathfindToPose() and followPath
   * 
   * @param path        The path to pathfind to, and then follow
   * @param constraints The pathfinding constraints
   * @return The command that will pathfind and then follow path
   */
  private static Command pathfindThenFollowPathNoFlip(PathPlannerPath path, PathConstraints constraints) {
    Pose2d pathStartPose = path.getPathPoses().get(0);
    LinearVelocity pathStartVelocity = path.getIdealStartingState().velocity();

    path.preventFlipping = true;

    return AutoBuilder.pathfindToPose(pathStartPose, constraints, pathStartVelocity)
        .andThen(AutoBuilder.followPath(path));

  }
}