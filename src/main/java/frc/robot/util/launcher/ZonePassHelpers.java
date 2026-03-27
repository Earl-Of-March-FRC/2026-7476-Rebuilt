package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PassConstants;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers.LaunchSetpoints;
import frc.robot.util.swerve.FieldZones;

/**
 * Helpers for zone-aware ball passing.
 *
 * <p>
 * Behaviour depends on the robot's current field zone:
 * <ul>
 * <li><b>Neutral zone</b>: Pass to the nearer of the two alliance-side bump
 * poses, provided the hub does not block the shot (line-of-sight check).
 * If both are blocked, fall back to the nearer one anyway.</li>
 * <li><b>Enemy zone</b>: Dump the ball to the far back of the alliance zone
 * so a robot can collect it.</li>
 * </ul>
 */
public final class ZonePassHelpers {

  private ZonePassHelpers() {
    throw new UnsupportedOperationException("Launcher Utility class");
  }

  /**
   * Selects the best pass target and returns full launch setpoints
   * (flywheel RPM + bot heading) for the current robot state.
   *
   * @param applyLead whether to lead the shot for drivetrain velocity
   * @return setpoints ready to feed into the launcher and drivetrain heading
   *         controller
   */
  public static LaunchSetpoints calculateZonePassSetpoints(boolean applyLead) {
    Translation3d target = selectPassTarget();
    Translation3d targetBotRelative = toTargetBotRelative(target);
    LaunchSetpoints setpoints = LaunchHelpers.calculateLaunchSetpoints(targetBotRelative, applyLead);

    Logger.recordOutput("ZonePass/Setpoints/FlywheelRPM", setpoints.flywheelSpeed().in(RPM));
    Logger.recordOutput("ZonePass/Setpoints/BotHeadingDeg", setpoints.botHeading().getDegrees());
    Logger.recordOutput("ZonePass/Setpoints/ApplyLead", applyLead);

    return setpoints;
  }

  /**
   * Returns the 3-D field-frame position of the currently chosen pass target.
   * Useful for logging / visualisation.
   */
  public static Translation3d selectPassTarget() {
    boolean isBlue = PoseHelpers.getAlliance() == Alliance.Blue;

    Translation2d robotPos = LaunchHelpers.drive().getPose().getTranslation();

    double allianceZoneX = FieldConstants.kAllianceZoneXLength.in(Meters);
    double fieldLength = FieldConstants.kFieldLengthX.in(Meters);
    double robotX = robotPos.getX();

    boolean inNeutralZone = robotX > allianceZoneX
        && robotX < fieldLength - allianceZoneX;

    Translation2d target2d;
    double targetHeight;
    String zoneLabel;

    if (inNeutralZone) {
      target2d = selectNeutralZoneTarget(robotPos, isBlue);
      targetHeight = PassConstants.kPassTargetHeight.in(Meters);
      zoneLabel = FieldZones.Neutral.toString();
    } else {
      // Enemy zone (or any unrecognised zone); dump shot to back of alliance zone.
      target2d = allianceDumpTarget(isBlue);
      targetHeight = PassConstants.kDumpTargetHeight.in(Meters);
      zoneLabel = FieldZones.Enemy.toString();
    }

    Translation3d target3d = new Translation3d(target2d.getX(), target2d.getY(), targetHeight);

    Logger.recordOutput("ZonePass/Zone", zoneLabel);
    Logger.recordOutput("ZonePass/Target3d", target3d);
    Logger.recordOutput("ZonePass/Target2d/X", target2d.getX());
    Logger.recordOutput("ZonePass/Target2d/Y", target2d.getY());
    Logger.recordOutput("ZonePass/TargetHeight", targetHeight);
    Logger.recordOutput("ZonePass/IsBlueAlliance", isBlue);
    Logger.recordOutput("ZonePass/RobotX", robotX);

    return target3d;
  }

  /**
   * Picks the nearer bump-pass pose that has line-of-sight past the hub.
   * Falls back to the geometrically nearer target if both are blocked.
   *
   * @param robotPos current robot XY position (field frame, Blue coordinates)
   * @param isBlue   true when on Blue alliance
   * @return chosen 2-D pass target
   */
  private static Translation2d selectNeutralZoneTarget(
      Translation2d robotPos, boolean isBlue) {

    Translation2d[] targets = allianceBumpPassTargets(isBlue);

    double dist0 = robotPos.getDistance(targets[0]);
    double dist1 = robotPos.getDistance(targets[1]);
    boolean t0Closer = dist0 <= dist1;

    Translation2d nearerTarget = t0Closer ? targets[0] : targets[1];
    Translation2d furtherTarget = t0Closer ? targets[1] : targets[0];

    Translation2d hubCenter = PoseHelpers.getAllianceHubtTranslation2d();
    double hubRadius = PassConstants.kHubLOSRadius.in(Meters);

    boolean nearerBlocked = segmentIntersectsCircle(robotPos, nearerTarget, hubCenter, hubRadius);
    boolean furtherBlocked = segmentIntersectsCircle(robotPos, furtherTarget, hubCenter, hubRadius);

    Logger.recordOutput("ZonePass/NeutralZone/DistToTarget0", dist0);
    Logger.recordOutput("ZonePass/NeutralZone/DistToTarget1", dist1);
    Logger.recordOutput("ZonePass/NeutralZone/NearerTargetIndex", t0Closer ? 0 : 1);
    Logger.recordOutput("ZonePass/NeutralZone/NearerBlocked", nearerBlocked);
    Logger.recordOutput("ZonePass/NeutralZone/FurtherBlocked", furtherBlocked);
    Logger.recordOutput("ZonePass/NeutralZone/HubCenterX", hubCenter.getX());
    Logger.recordOutput("ZonePass/NeutralZone/HubCenterY", hubCenter.getY());
    Logger.recordOutput("ZonePass/NeutralZone/HubLOSRadius", hubRadius);

    // Prefer the nearer target if it has LOS; otherwise try the further one.
    if (!nearerBlocked) {
      Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "Nearer");
      return nearerTarget;
    }
    if (!furtherBlocked) {
      Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "Further");
      return furtherTarget;
    }

    // Both blocked: Default to geometrically nearer target as best effort.
    Logger.recordOutput("ZonePass/NeutralZone/ChosenTarget", "NearerFallback(BothBlocked)");
    return nearerTarget;
  }

  /**
   * Returns the two bump-pass targets for the current alliance.
   * Red targets are the Blue targets mirrored across the field centre.
   */
  private static Translation2d[] allianceBumpPassTargets(boolean isBlue) {
    if (isBlue) {
      return PassConstants.kBlueBumpPassTargets;
    }
    double fieldLength = FieldConstants.kFieldLengthX.in(Meters);
    double fieldWidth = FieldConstants.kFieldWidthY.in(Meters);
    return new Translation2d[] {
        new Translation2d(
            fieldLength - PassConstants.kBlueBumpPassTargets[0].getX(),
            fieldWidth - PassConstants.kBlueBumpPassTargets[0].getY()),
        new Translation2d(
            fieldLength - PassConstants.kBlueBumpPassTargets[1].getX(),
            fieldWidth - PassConstants.kBlueBumpPassTargets[1].getY())
    };
  }

  /**
   * Returns the enemy-zone dump target for the current alliance.
   * Red is derived by mirroring the Blue constant.
   */
  private static Translation2d allianceDumpTarget(boolean isBlue) {
    if (isBlue) {
      return new Translation2d(
          PassConstants.kBlueDumpTargetX.in(Meters),
          PassConstants.kBlueDumpTargetY.in(Meters));
    }
    double fieldLength = FieldConstants.kFieldLengthX.in(Meters);
    double fieldWidth = FieldConstants.kFieldWidthY.in(Meters);
    return new Translation2d(
        fieldLength - PassConstants.kBlueDumpTargetX.in(Meters),
        fieldWidth - PassConstants.kBlueDumpTargetY.in(Meters));
  }

  /**
   * Returns {@code true} if the line segment from {@code p1} to {@code p2}
   * passes within {@code radius} of {@code center} (i.e. the hub blocks LOS).
   *
   * <p>
   * Uses the standard parametric closest-point-on-segment formula so only
   * the actual path of the ball (not the infinite extension of the line) is
   * tested.
   *
   * @param p1     segment start (robot position)
   * @param p2     segment end (pass target)
   * @param center circle centre (hub XY position)
   * @param radius effective hub LOS radius
   * @return true if the segment is blocked by the circle
   */
  static boolean segmentIntersectsCircle(
      Translation2d p1, Translation2d p2,
      Translation2d center, double radius) {

    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    double lenSq = dx * dx + dy * dy;

    if (lenSq < 1e-12) {
      // Degenerate zero-length segment: Check whether the point is inside.
      double ddx = p1.getX() - center.getX();
      double ddy = p1.getY() - center.getY();
      return (ddx * ddx + ddy * ddy) <= (radius * radius);
    }

    // t element of [0,1] is the parameter of the closest point on the segment to
    // center.
    double t = ((center.getX() - p1.getX()) * dx
        + (center.getY() - p1.getY()) * dy) / lenSq;
    t = Math.max(0.0, Math.min(1.0, t));

    double closestX = p1.getX() + t * dx;
    double closestY = p1.getY() + t * dy;

    double distX = center.getX() - closestX;
    double distY = center.getY() - closestY;

    return (distX * distX + distY * distY) <= (radius * radius);
  }

  /**
   * Converts a field-frame 3-D target into a bot-relative 3-D vector.
   * This is the form expected by
   * {@link LaunchHelpers#calculateLaunchSetpoints(Translation3d, boolean)}.
   */
  private static Translation3d toTargetBotRelative(Translation3d fieldTarget) {
    Translation2d botPos = LaunchHelpers.drive().getPose().getTranslation();
    Translation2d delta = fieldTarget.toTranslation2d().minus(botPos);

    Translation3d botRelative = new Translation3d(delta.getX(), delta.getY(), fieldTarget.getZ());

    Logger.recordOutput("ZonePass/TargetBotRelative/X", botRelative.getX());
    Logger.recordOutput("ZonePass/TargetBotRelative/Y", botRelative.getY());
    Logger.recordOutput("ZonePass/TargetBotRelative/Z", botRelative.getZ());
    Logger.recordOutput("ZonePass/TargetBotRelative/DistanceMeters",
        botRelative.toTranslation2d().getNorm());

    return botRelative;
  }
}