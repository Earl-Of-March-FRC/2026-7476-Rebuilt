package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.commands.drivetrain.DriveStopCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
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

  /**
   * Create a command to drive through the nearest bump (without pathfinding)
   * 
   * @param trenchWaypoints A list of waypoints to consider when finding the
   *                        nearest bump
   * @return The command to schedule
   */
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

  /**
   * Create a command to drive through the nearest trench (without pathfinding)
   * 
   * @param trenchWaypoints A list of waypoints to consider when finding the
   *                        nearest trench
   * @return The command to schedule
   */
  public static Command crossTrenchAuto(Translation2d[] trenchWaypoints) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    Rotation2d targetHeading = drive()
        .getNearestTargetAngle(Rotation2d.fromDegrees(DriveConstants.kTrenchHeadingRestriction.in(Degrees)), false);

    int trenchID = nearestTranslation2dIndex(trenchWaypoints);
    Pose2d start = new Pose2d(FieldConstants.kTrenchPathWaypoints[trenchID], targetHeading);

    Distance endX = isBlueAlliance ? AutoConstants.kAutoNeutralZoneX
        : FieldConstants.kFieldLengthX.minus(AutoConstants.kAutoNeutralZoneX);
    Pose2d end = new Pose2d(endX.in(Meters), start.getY(), targetHeading);

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
   * Gets the corresponding path for the tower side
   * 
   * @param side Tower side relative to the driverstation
   * @return {@code PathPlannerPath} containing waypoints to the tower
   * @apiNote These paths are not on the fly. They are the premade paths made
   *          using the Path Planner app
   */
  public static PathPlannerPath getTowerPathFromSide(TowerSide side) {
    return switch (side) {
      case Left -> AutoConstants.depotClimbVeryCurvedPath;
      case Right -> AutoConstants.outpostClimbVeryCurvedPath;
    };
  }

  /**
   * Basic command that drives to the first waypoint of the L1 path
   * 
   * @param side Side of the tower relative to the driverstation
   * @return Command
   */
  public static Command driveToTowerFrontAuto(TowerSide side) {
    PathPlannerPath selectedPath = getTowerPathFromSide(side);

    if (selectedPath == null) {
      return new PrintCommand("Selected path for driveToTowerFrontAuto was null.");
    }

    List<Pose2d> waypoints = selectedPath.getPathPoses();
    if (waypoints.isEmpty()) {
      return new PrintCommand("Selected path for driveToTowerFrontAuto was empty.");
    }

    Pose2d firstWaypoint = waypoints.get(0);
    Pose2d desiredPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), Rotation2d.kZero);

    Logger.recordOutput("Commands/PathGenerator/driveToTowerFrontAuto/DesiredPose", desiredPose);

    return AutoBuilder.pathfindToPoseFlipped(desiredPose, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
  }

  /**
   * Basic command that drives to the last waypoint of the L1 path
   * 
   * @param side Side of the tower relative to the driverstation
   * @return Command
   */
  public static Command driveToTowerSideAuto(TowerSide side) {
    PathPlannerPath selectedPath = getTowerPathFromSide(side);

    if (selectedPath == null) {
      return new PrintCommand("Selected path for driveToTowerSideAuto was null.");
    }

    List<Pose2d> waypoints = selectedPath.getPathPoses();
    if (waypoints.isEmpty()) {
      return new PrintCommand("Selected path for driveToTowerSideAuto was empty.");
    }

    Pose2d lastWaypoint = waypoints.get(waypoints.size() - 1);
    Pose2d desiredClimbPose = new Pose2d(lastWaypoint.getX(), lastWaypoint.getY(), Rotation2d.kZero);

    Logger.recordOutput("Commands/PathGenerator/driveToTowerSideAuto/DesiredPose", desiredClimbPose);

    return AutoBuilder.pathfindThenFollowPath(selectedPath, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
  }

  /**
   * Creates a command that drives to the start of the depot intake path
   * 
   * @return Command
   */
  public static Command driveToDepotAuto() {
    PathPlannerPath depotPath = AutoConstants.depotPath;
    if (depotPath == null) {
      return new PrintCommand("Depot path for driveToDepotAuto was null.");
    }

    List<Pose2d> waypoints = depotPath.getPathPoses();
    if (waypoints.isEmpty()) {
      return new PrintCommand("Depot path for driveToDepotAuto was empty.");
    }

    Rotation2d targetRotation = Rotation2d.kZero;
    Pose2d firstWaypoint = waypoints.get(0);
    Pose2d desiredPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), targetRotation);

    Logger.recordOutput("Commands/PathGenerator/driveToDepotAuto/DesiredPose", desiredPose);

    return AutoBuilder.pathfindToPoseFlipped(desiredPose, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
  }

  /**
   * Creates a command that drives to the neutral zone through the trench during
   * auto
   * 
   * @return Command
   */
  public static Command driveToNeutralZoneTrenchAuto() {
    PathPlannerPath neutralZonePath;

    boolean isBlueAlliance = PoseHelpers.getAlliance() == Alliance.Blue;

    if (drive().getPose().getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      neutralZonePath = isBlueAlliance ? AutoConstants.neutralZoneTrenchOutpost : AutoConstants.neutralZoneTrenchDepot;
    } else {
      neutralZonePath = isBlueAlliance ? AutoConstants.neutralZoneTrenchDepot : AutoConstants.neutralZoneTrenchOutpost;
    }

    if (neutralZonePath == null) {
      return new PrintCommand("Path for driveToNeutralZoneTrenchAuto was null.");
    }

    return AutoBuilder.pathfindThenFollowPath(neutralZonePath, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
  }

  /**
   * Creates a command that drives to the neutral zone through the bump during
   * auto
   * 
   * @return Command
   */
  public static Command driveToNeutralZoneBumpAuto() {
    PathPlannerPath neutralZonePath;

    boolean isBlueAlliance = PoseHelpers.getAlliance() == Alliance.Blue;

    if (drive().getPose().getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      neutralZonePath = isBlueAlliance ? AutoConstants.neutralZoneBumpOutpost : AutoConstants.neutralZoneBumpDepot;
    } else {
      neutralZonePath = isBlueAlliance ? AutoConstants.neutralZoneBumpDepot : AutoConstants.neutralZoneBumpOutpost;
    }

    if (neutralZonePath == null) {
      return new PrintCommand("Path for driveToNeutralZoneBumpAuto was null.");
    }

    return AutoBuilder.pathfindThenFollowPath(neutralZonePath, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
  }

  /**
   * Creates a command that drives to the launch pose during auto
   * 
   * @return Command
   */
  public static Command driveToLaunchPoseAuto() {
    boolean isBlueAlliance = PoseHelpers.getAlliance() == Alliance.Blue;

    Pose2d startPose = drive().getPose();

    Distance desiredX = FieldConstants.kAllianceZoneXLength.minus(AutoConstants.kAutoLaunchDistanceFromHubX);
    desiredX = isBlueAlliance ? desiredX : FieldConstants.kFieldLengthX.minus(desiredX);

    // : FieldConstants.kFieldLengthX.minus(AutoConstants.autoLaunchDistance);
    Distance desiredY = Meters.of(startPose.getY());
    // : FieldConstants.kFieldWidthY.minus(Meters.of(startPose.getY()));
    Logger.recordOutput("Commands/PathGenerator/driveToLaunchPoseAuto/StartPoseY", desiredY);

    Pose2d desiredPose = new Pose2d(desiredX, desiredY, startPose.getRotation());

    return AutoBuilder.pathfindToPose(desiredPose, AutoConstants.L1ClimbConstraints)
        .andThen(new DriveStopCmd(drive()));
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