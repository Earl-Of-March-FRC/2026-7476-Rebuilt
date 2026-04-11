// NEW CODE

package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.Constants.PassConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers.LaunchSetpoints;
import frc.robot.util.launcher.LaunchHelpers.ShotPrediction;
import frc.robot.util.swerve.FieldZones;

/**
 * Helpers for zone-aware ball passing.
 *
 * <p>
 * <b>Zone behaviour:</b>
 * <ul>
 * <li><b>Neutral zone:</b> Pass to the nearer alliance-side bump pose that is
 * not blocked by our hub. Falls back to the geometrically nearer pose if both
 * are blocked.</li>
 * <li><b>Enemy zone:</b> Dump the ball to the far back of the alliance zone so
 * a robot can collect it.</li>
 * </ul>
 *
 * <p>
 * All setpoint math flows through {@link LaunchHelpers#calculateSetpoints} so
 * that hub shots and pass shots share the same heading derivation and lead
 * correction logic. The only difference is the RPM model: pass shots use
 * {@link LaunchHelpers#calculatePhysicsRpm} (projectile physics at the pass
 * release angle), while hub shots use the empirical curve.
 */
public final class ZonePassHelpers {

  private ZonePassHelpers() {
    throw new UnsupportedOperationException("ZonePassHelpers is a static utility class");
  }

  /**
   * Selects the best pass target for the current zone and returns complete
   * launch setpoints (flywheel RPM + bot heading).
   *
   * <p>
   * Both the target selection and the setpoint math update every call, so this
   * is safe to call every command cycle.
   *
   * @param applyDrivetrainLead If {@code true}, the heading is lead-corrected
   *                            for the robot's current velocity
   * @return Setpoints ready to hand to the launcher and heading controller
   */
  public static LaunchSetpoints calculatePassSetpoints(boolean applyDrivetrainLead) {
    Translation3d target = selectPassTarget();

    Pose2d botPose = LaunchHelpers.driveSubsystem().getPose();
    ChassisSpeeds fieldSpeeds = applyDrivetrainLead
        ? LaunchHelpers.driveSubsystem().getChassisSpeedsFieldRelative()
        : new ChassisSpeeds();

    Translation2d robotPos = botPose.getTranslation();
    Translation2d toTarget2d = target.toTranslation2d().minus(robotPos);

    Translation2d robotVelocity = new Translation2d(
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);

    double releaseHeight = LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters);
    double angle = LauncherAndIntakeConstants.kBallReleaseAngle().in(Radians);
    double cosA = Math.cos(angle);
    double sinA = Math.sin(angle);
    double g = PhysicsConstants.kGravityMps2;

    // Root solve for t
    double tLow = 0.05;
    double tHigh = 3.0;
    double t = 1.0;

    Translation2d launcherVelocity = new Translation2d();
    double adjustedBallSpeed = 0;

    for (int i = 0; i < 25; i++) {
      t = (tLow + tHigh) / 2.0;

      // Solve horizontal velocity needed
      launcherVelocity = toTarget2d.div(t).minus(robotVelocity);
      double requiredVh = launcherVelocity.getNorm();

      // Convert to full velocity using fixed angle
      adjustedBallSpeed = requiredVh / cosA;
      double vz = adjustedBallSpeed * sinA;

      // Compute actual airtime from vertical motion
      double computedT = (vz + Math.sqrt(vz * vz + 2 * g * releaseHeight)) / g;

      // Binary search condition
      if (computedT > t) {
        tLow = t;
      } else {
        tHigh = t;
      }
    }
    AngularVelocity rpm = LauncherAndIntakeConstants.linearVelocityToAngularVelocity(
        MetersPerSecond.of(adjustedBallSpeed));

    double launchYaw = Math.atan2(
        launcherVelocity.getY(),
        launcherVelocity.getX());

    Rotation2d botHeading = Rotation2d.fromRadians(launchYaw)
        .minus(LauncherAndIntakeConstants.kLauncherBotHeading);

    LaunchSetpoints setpoints = new LaunchSetpoints(rpm, botHeading);

    Translation2d totalVelocity = launcherVelocity.plus(robotVelocity);
    Translation2d landing2d = robotPos.plus(totalVelocity.times(t));

    Translation3d landing = new Translation3d(
        landing2d.getX(),
        landing2d.getY(),
        target.getZ());

    // Validation
    double distanceToTarget = landing.toTranslation2d().getDistance(target.toTranslation2d());

    boolean outOfField = !PoseHelpers.isInField(
        new Pose2d(landing.toTranslation2d(), new Rotation2d()));

    double hubRadius = FieldConstants.kHubInsideWidth.div(2).in(Meters);

    boolean blockedByOwnHub = LaunchHelpers.isTrajectoryBlockedByHub(
        robotPos, landing.toTranslation2d(),
        PoseHelpers.getAllianceHubtTranslation2d(), hubRadius);

    boolean blockedByEnemyHub = LaunchHelpers.isTrajectoryBlockedByHub(
        robotPos, landing.toTranslation2d(),
        PoseHelpers.getEnemyHubTranslation2d(), hubRadius);

    boolean willScore = distanceToTarget < PassConstants.kPassLandingTolerance.in(Meters)
        && !outOfField && !blockedByOwnHub && !blockedByEnemyHub;

    ShotPrediction prediction = new ShotPrediction(
        willScore, landing, blockedByOwnHub, blockedByEnemyHub,
        outOfField, distanceToTarget, t);

    LaunchHelpers.logShotPrediction(prediction, "ZonePass/Prediction");

    return setpoints;
  }

  /**
   * Returns the 3-D field-frame position of the currently selected pass target.
   * Useful for visualisation and logging without re-computing setpoints.
   */
  public static Translation3d selectPassTarget() {
    boolean isBlue = PoseHelpers.getAlliance() == Alliance.Blue;
    Translation2d robotPos = LaunchHelpers.driveSubsystem().getPose().getTranslation();

    double allianceZoneX = FieldConstants.kAllianceZoneXLength.in(Meters);
    double fieldLength = FieldConstants.kFieldLengthX.in(Meters);
    double robotX = robotPos.getX();

    boolean inNeutralZone = robotX > allianceZoneX && robotX < fieldLength - allianceZoneX;

    Translation2d target2d;
    double targetHeight;
    String zoneLabel;

    if (inNeutralZone) {
      target2d = selectNeutralTarget(robotPos, isBlue);
      targetHeight = PassConstants.kPassTargetHeight.in(Meters);
      zoneLabel = FieldZones.Neutral.toString();
    } else {
      // target2d = dumpTarget(isBlue);
      // targetHeight = PassConstants.kDumpTargetHeight.in(Meters);
      target2d = selectNeutralTarget(robotPos, isBlue);
      targetHeight = PassConstants.kPassTargetHeight.in(Meters);
      zoneLabel = FieldZones.Enemy.toString();
    }

    Translation3d target3d = new Translation3d(target2d.getX(), target2d.getY(), targetHeight);

    Logger.recordOutput("ZonePass/Zone", zoneLabel);
    Logger.recordOutput("ZonePass/Target3d", target3d);

    return target3d;
  }

  /**
   * Predicts whether a pass shot will reach the intended target, and whether it
   * is blocked by either hub or leaves the field.
   *
   * <p>
   * Pass landing tolerance is much looser than a hub shot — a robot can chase a
   * ball that lands within {@link PassConstants#kPassLandingTolerance} of the
   * intended target. The shot is considered successful if it lands within that
   * radius, stays in the field, and is not blocked by either hub.
   *
   * @param target         The intended 3-D field-frame pass target
   * @param rpm            The flywheel speed being commanded
   * @param desiredHeading The bot heading being commanded
   * @return Full shot prediction including obstruction and landing analysis
   */
  public static ShotPrediction predictPassShot(
      Translation3d target, AngularVelocity rpm, Rotation2d desiredHeading) {

    Pose2d botPose = LaunchHelpers.driveSubsystem().getPose();
    Translation3d landing = LaunchHelpers.predictPassLanding(
        botPose, rpm, desiredHeading);

    double distanceToTarget = landing.toTranslation2d()
        .getDistance(target.toTranslation2d());

    boolean outOfField = !PoseHelpers.isInField(
        new Pose2d(landing.toTranslation2d(), new Rotation2d()));

    double hubRadius = FieldConstants.kHubInsideWidth.div(2).in(Meters);

    boolean blockedByOwnHub = LaunchHelpers.isTrajectoryBlockedByHub(
        botPose.getTranslation(),
        landing.toTranslation2d(),
        PoseHelpers.getAllianceHubtTranslation2d(),
        hubRadius);

    boolean blockedByEnemyHub = LaunchHelpers.isTrajectoryBlockedByHub(
        botPose.getTranslation(),
        landing.toTranslation2d(),
        PoseHelpers.getEnemyHubTranslation2d(),
        hubRadius);

    boolean willScore = distanceToTarget < PassConstants.kPassLandingTolerance.in(Meters)
        && !outOfField
        && !blockedByOwnHub
        && !blockedByEnemyHub;

    double airTimeSec = LaunchHelpers.calculateAirTime(
        Meters.of(target.getZ()), rpm).in(Seconds);

    // DEBUG
    Logger.recordOutput("ZonePass/Prediction/Debug/LandingX", landing.getX());
    Logger.recordOutput("ZonePass/Prediction/Debug/LandingY", landing.getY());
    Logger.recordOutput("ZonePass/Prediction/Debug/Landing3d", landing);
    Logger.recordOutput("ZonePass/Prediction/Debug/TargetX", target.getX());
    Logger.recordOutput("ZonePass/Prediction/Debug/TargetY", target.getY());
    Logger.recordOutput("ZonePass/Prediction/Debug/DistanceToTarget", distanceToTarget);
    Logger.recordOutput("ZonePass/Prediction/Debug/OutOfField", outOfField);
    Logger.recordOutput("ZonePass/Prediction/Debug/BlockedByOwnHub", blockedByOwnHub);
    Logger.recordOutput("ZonePass/Prediction/Debug/BlockedByEnemyHub", blockedByEnemyHub);
    Logger.recordOutput("ZonePass/Prediction/Debug/Tolerance",
        PassConstants.kPassLandingTolerance.in(Meters));
    Logger.recordOutput("ZonePass/Prediction/Debug/AirTimeSec", airTimeSec);
    Logger.recordOutput("ZonePass/Prediction/Debug/RobotVx",
        LaunchHelpers.driveSubsystem().getChassisSpeedsFieldRelative().vxMetersPerSecond);
    Logger.recordOutput("ZonePass/Prediction/Debug/RobotVy",
        LaunchHelpers.driveSubsystem().getChassisSpeedsFieldRelative().vyMetersPerSecond);

    return new ShotPrediction(
        willScore, landing, blockedByOwnHub, blockedByEnemyHub,
        outOfField, distanceToTarget, airTimeSec);
  }

  /**
   * Picks the nearer bump-pass pose that has a clear line of sight past our hub.
   * Falls back to the geometrically nearer pose if both are blocked.
   *
   * @param robotPos Robot XY position (field frame, Blue coordinates)
   * @param isBlue   {@code true} when on Blue alliance
   * @return The chosen 2-D pass target
   */
  private static Translation2d selectNeutralTarget(Translation2d robotPos, boolean isBlue) {
    Translation2d[] targets = bumpPassTargets(isBlue);

    double dist0 = robotPos.getDistance(targets[0]);
    double dist1 = robotPos.getDistance(targets[1]);
    boolean target0IsNearer = dist0 <= dist1;

    Translation2d nearer = target0IsNearer ? targets[0] : targets[1];
    Translation2d further = target0IsNearer ? targets[1] : targets[0];

    Translation2d ownHub = PoseHelpers.getAllianceHubtTranslation2d();
    double hubRadius = PassConstants.kHubLOSRadius.in(Meters);

    boolean nearerBlocked = LaunchHelpers.isTrajectoryBlockedByHub(robotPos, nearer, ownHub, hubRadius);
    boolean furtherBlocked = LaunchHelpers.isTrajectoryBlockedByHub(robotPos, further, ownHub, hubRadius);

    Logger.recordOutput("ZonePass/NeutralZone/NearerTargetBlocked", nearerBlocked);
    Logger.recordOutput("ZonePass/NeutralZone/FurtherTargetBlocked", furtherBlocked);

    if (!nearerBlocked) {
      Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "Nearer");
      return nearer;
    }
    if (!furtherBlocked) {
      Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "Further");
      return further;
    }

    // Both blocked: best-effort fallback to the geometrically closer target
    Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "NearerFallback (both blocked)");
    return nearer;
  }

  /** Returns the two bump-pass targets for the given alliance. */
  private static Translation2d[] bumpPassTargets(boolean isBlue) {
    if (isBlue) {
      return PassConstants.kBlueBumpPassTargets;
    }
    double fl = FieldConstants.kFieldLengthX.in(Meters);
    double fw = FieldConstants.kFieldWidthY.in(Meters);
    return new Translation2d[] {
        new Translation2d(fl - PassConstants.kBlueBumpPassTargets[0].getX(),
            fw - PassConstants.kBlueBumpPassTargets[0].getY()),
        new Translation2d(fl - PassConstants.kBlueBumpPassTargets[1].getX(),
            fw - PassConstants.kBlueBumpPassTargets[1].getY())
    };
  }

  /** Returns the enemy-zone dump target for the given alliance. */
  private static Translation2d dumpTarget(boolean isBlue) {
    if (isBlue) {
      return new Translation2d(
          PassConstants.kBlueDumpTargetX.in(Meters),
          PassConstants.kBlueDumpTargetY.in(Meters));
    }
    double fl = FieldConstants.kFieldLengthX.in(Meters);
    double fw = FieldConstants.kFieldWidthY.in(Meters);
    return new Translation2d(
        fl - PassConstants.kBlueDumpTargetX.in(Meters),
        fw - PassConstants.kBlueDumpTargetY.in(Meters));
  }
}