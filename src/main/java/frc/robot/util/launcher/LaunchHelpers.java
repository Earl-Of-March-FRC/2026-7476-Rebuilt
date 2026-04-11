// NEW CODE
package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.PoseHelpers;

/**
 * Static helpers for launcher physics, RPM calculation, and shot prediction.
 *
 * <ul>
 * <li>All public methods that need the drivetrain or launcher subsystem use the
 * singleton references set by {@link #setSubsystems}. Methods that accept
 * explicit parameters are preferred for testability and reuse.</li>
 * <li>RPM calculation is intentionally separated from heading calculation so
 * that pass shots (physics-derived RPM) and hub shots (empirical curve RPM)
 * can both flow through the same {@link #calculateSetpoints} entry point.</li>
 * <li>Shot-prediction logging is consolidated in
 * {@link #logShotPrediction}. Call it once per command cycle from the
 * relevant command's {@code execute()} to keep the subsystem periodic clean.
 * </li>
 * </ul>
 */
public final class LaunchHelpers {

  /**
   * Everything a launcher command needs to fire at a target.
   *
   * @param flywheelSpeed RPM the flywheel should spin at
   * @param botHeading    Field-relative heading the robot should face so the
   *                      launcher points at the target
   */
  public record LaunchSetpoints(AngularVelocity flywheelSpeed, Rotation2d botHeading, boolean isShotSafe) {
    /** Convenience constructor for hub shots where safety is not checked here. */
    public LaunchSetpoints(AngularVelocity flywheelSpeed, Rotation2d botHeading) {
      this(flywheelSpeed, botHeading, true); // hub shots gate elsewhere via willHitHub()
    }
  }

  /**
   * Detailed prediction of where the ball will land and whether it will score.
   *
   * @param willScore         True when the ball is predicted to enter the hub
   * @param predictedLanding  Field-frame 3-D position where the ball lands
   * @param blockedByOwnHub   True when the trajectory passes through our own hub
   *                          structure
   * @param blockedByEnemyHub True when the trajectory passes through the enemy
   *                          hub structure
   * @param outOfField        True when the predicted landing is outside field
   *                          boundaries
   * @param distanceToLanding XY distance from predicted landing to the intended
   *                          target
   * @param airTimeSeconds    Estimated flight time in seconds
   */
  public record ShotPrediction(
      boolean willScore,
      Translation3d predictedLanding,
      boolean blockedByOwnHub,
      boolean blockedByEnemyHub,
      boolean outOfField,
      double distanceToLanding,
      double airTimeSeconds) {
  }

  private static LauncherAndIntakeSubsystem launcherSub;
  private static DrivetrainSubsystem driveSub;
  private static boolean configured = false;

  private LaunchHelpers() {
    throw new UnsupportedOperationException("LaunchHelpers is a static utility class");
  }

  /** Call exactly once from {@code RobotContainer} before any helper is used. */
  public static void setSubsystems(DrivetrainSubsystem drivetrain,
      LauncherAndIntakeSubsystem launcher) {
    if (configured) {
      throw new IllegalStateException("LaunchHelpers.setSubsystems() called more than once");
    }
    driveSub = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
    launcherSub = Objects.requireNonNull(launcher, "launcher must not be null");
    configured = true;
  }

  /**
   * Read-only accessor for other utilities that need the drivetrain reference.
   */
  public static DrivetrainSubsystem driveSubsystem() {
    requireConfigured();
    return driveSub;
  }

  /**
   * Unified entry point: calculates the flywheel speed and bot heading needed to
   * hit {@code target} from the robot's current position.
   *
   * <p>
   * Both hub shots and pass shots go through this method. The only difference is
   * how {@code rpmForTarget} is calculated — pass shots use physics-derived RPM
   * (see {@link #calculatePhysicsRpm}), hub shots use the empirical lookup curve
   * (see {@link #calculateHubRpm}).
   *
   * @param targetFieldFrame    3-D target position in field coordinates
   * @param rpmForTarget        The flywheel RPM needed to reach that target —
   *                            callers decide which RPM model fits their shot
   * @param applyDrivetrainLead If true, the heading is adjusted to compensate
   *                            for the robot's current velocity (lead correction)
   * @return Setpoints ready to hand to the launcher and heading controller
   */
  public static LaunchSetpoints calculateSetpoints(
      Translation3d targetFieldFrame,
      AngularVelocity rpmForTarget,
      boolean applyDrivetrainLead) {

    requireConfigured();

    // Vector from robot to target in field frame
    Translation2d robotXY = driveSub.getPose().getTranslation();
    Translation2d toTarget2d = targetFieldFrame.toTranslation2d().minus(robotXY);

    if (!applyDrivetrainLead) {
      // Simple case: just aim straight at the target
      Rotation2d heading = toTarget2d.getAngle()
          .minus(LauncherAndIntakeConstants.kLauncherBotHeading);
      return new LaunchSetpoints(rpmForTarget, heading);
    }

    // Lead correction: back out the bot's XY velocity contribution so the
    // ball's resultant velocity points at the target.
    ChassisSpeeds fieldSpeeds = driveSub.getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond, 0);

    Translation3d launchVelocity = calculateLaunchVelocityVector(rpmForTarget);

    // Re-aim the 2-D component of the launch vector toward the target,
    // keeping the vertical component (pitch) unchanged.
    double horizontalMagnitude = launchVelocity.toTranslation2d().getNorm();
    Translation2d reaimed2d = new Translation2d(horizontalMagnitude, toTarget2d.getAngle());
    Translation3d reaimed = new Translation3d(
        reaimed2d.getX(), reaimed2d.getY(), launchVelocity.getZ());

    // Subtract the drivetrain's contribution: V_launcher = V_total − V_bot
    Translation3d adjustedLaunch = reaimed.minus(botVelocity);

    Rotation2d heading = adjustedLaunch.toTranslation2d().getAngle()
        .minus(LauncherAndIntakeConstants.kLauncherBotHeading);

    AngularVelocity adjustedRpm = Constants.LauncherAndIntakeConstants.linearVelocityToAngularVelocity(
        MetersPerSecond.of(adjustedLaunch.getNorm()));

    return new LaunchSetpoints(adjustedRpm, heading);
  }

  /**
   * Convenience wrapper: calculates hub setpoints from the robot's current
   * position using the empirical RPM curve.
   *
   * @param applyDrivetrainLead Whether to apply drivetrain lead correction
   * @return Hub launch setpoints
   */
  public static LaunchSetpoints calculateHubSetpoints(boolean applyDrivetrainLead) {
    requireConfigured();
    Translation3d hubTarget = PoseHelpers.getAllianceHubtTranslation3d();
    AngularVelocity rpm = calculateHubRpm();
    return calculateSetpoints(hubTarget, rpm, applyDrivetrainLead);
  }

  /**
   * Hub RPM using the empirical polynomial curve tuned for the hub height.
   * Adjusts the equivalent distance when the target is at a height other than
   * {@link frc.robot.Constants.FieldConstants#kHubHeight}.
   */
  public static AngularVelocity calculateHubRpm() {
    requireConfigured();
    return calculateHubRpm(driveSub.getHubDistance(), FieldConstants.kHubHeight);
  }

  /**
   * Hub RPM using the empirical polynomial curve. Use this when you know the
   * horizontal distance and target height explicitly.
   *
   * @param horizontalDistance XY distance to the target
   * @param targetHeight       Z height of the target
   * @return Ideal flywheel speed
   */
  public static AngularVelocity calculateHubRpm(Distance horizontalDistance, Distance targetHeight) {
    // The empirical curve was fit at hub height. For other heights we solve for
    // the equivalent horizontal distance that produces the same launch RPM.
    AngularVelocity omega0 = LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(horizontalDistance);

    if (targetHeight.isEquivalent(FieldConstants.kHubHeight)) {
      return omega0;
    }

    Translation3d initialVelocity = calculateLaunchVelocityVector(omega0);
    double V2d = initialVelocity.toTranslation2d().getNorm();
    double Vz = initialVelocity.getZ();

    // Solve for horizontal range at targetHeight using standard kinematics:
    // h = Vz*x/V2d − 0.5*g*(x/V2d)^2 => quadratic in x
    double a = -PhysicsConstants.kGravityMps2 / 2.0 / (V2d * V2d);
    double b = Vz / V2d;
    double c = -targetHeight.in(Meters);
    double discriminant = b * b - 4 * a * c;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      // DriverStation.reportError(
      // "calculateHubRpm: discriminant is NaN or negative (" + discriminant + ")",
      // false);
      return RPM.zero();
    }

    double equivalentDistanceMeters = (-b + Math.sqrt(discriminant)) / (2.0 * a);
    return LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(Meters.of(equivalentDistanceMeters));
  }

  /**
   * Physics-derived RPM for a pass shot to a <em>ground-level</em> target using
   * the fixed pass release angle
   * ({@link LauncherAndIntakeConstants#kPassReleaseAngle}).
   *
   * <p>
   * This is more accurate than the empirical hub curve for off-angle shots
   * because it directly solves the projectile equations.
   *
   * @param horizontalDistance XY-plane distance from robot to target
   * @return Flywheel speed needed to reach that distance
   */
  public static AngularVelocity calculatePhysicsRpm(Distance horizontalDistance) {
    double d = horizontalDistance.in(Meters);
    double releaseHeight = LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters);
    double angle = LauncherAndIntakeConstants.kBallReleaseAngle().in(Radians);
    double g = PhysicsConstants.kGravityMps2;

    double cosA = Math.cos(angle);
    double sinA = Math.sin(angle);
    double tanA = sinA / cosA;

    // Solve: V^2 = (g * d^2) / (2 * cos^2A * (d*tanA + releaseHeight))
    double denominator = 2 * cosA * cosA * (d * tanA + releaseHeight);
    if (denominator <= 0) {
      // DriverStation.reportError(
      // "calculatePhysicsRpm: invalid denominator " + denominator, false);
      return RPM.of(1000);
    }

    double ballSpeedMps = Math.sqrt((g * d * d) / denominator);
    return LauncherAndIntakeConstants.linearVelocityToAngularVelocity(MetersPerSecond.of(ballSpeedMps));
  }

  /**
   * Calculates the launcher's contribution to the ball velocity, in field frame,
   * for the given flywheel speed.
   *
   * <p>
   * Does not include the robot's translational velocity. To get the
   * full resultant velocity, add the robot velocity from
   * {@link #calculateResultantVelocityVector(AngularVelocity)}.
   *
   * @param rpm Flywheel speed
   * @return Ball launch velocity (m/s) as a field-frame {@link Translation3d}
   */
  public static Translation3d calculateLaunchVelocityVector(AngularVelocity rpm) {
    requireConfigured();
    double wheelLinearMps = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * rpm.in(RadiansPerSecond);
    double ballSpeedMps = wheelLinearMps * LauncherAndIntakeConstants.kWheelSlipCoefficient;

    // Robot-relative: forward component along the release angle, zero lateral
    Translation3d robotRelative = new Translation3d(
        Math.cos(LauncherAndIntakeConstants.kBallReleaseAngle().in(Radians)),
        0,
        Math.sin(LauncherAndIntakeConstants.kBallReleaseAngle().in(Radians)))
        .times(ballSpeedMps)
        .rotateBy(new Rotation3d(LauncherAndIntakeConstants.kLauncherBotHeading));

    // Rotate into field frame using the robot's current heading
    return robotRelative.rotateBy(new Rotation3d(driveSub.getPose().getRotation()));
  }

  /**
   * Calculates the ball's full resultant velocity (launcher + robot drivetrain)
   * in field frame, for the given flywheel speed.
   *
   * @param rpm Flywheel speed
   * @return Resultant ball velocity as a field-frame {@link Translation3d}
   */
  public static Translation3d calculateResultantVelocityVector(AngularVelocity rpm) {
    requireConfigured();
    ChassisSpeeds fieldSpeeds = driveSub.getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond, 0);
    return calculateLaunchVelocityVector(rpm).plus(botVelocity);
  }

  /**
   * Predicts where the ball will land at the given target height.
   *
   * @param targetHeight Z height of the intended target
   * @param botPose      Robot pose at launch time
   * @param rpm          Flywheel speed
   * @return Field-frame 3-D landing position
   */
  public static Translation3d predictBallLanding(Distance targetHeight, Pose2d botPose, AngularVelocity rpm) {
    Translation3d initialVelocity = calculateResultantVelocityVector(rpm);
    Time airTime = calculateAirTime(targetHeight, rpm);

    Translation2d horizontalTravel = initialVelocity
        .times(airTime.in(Seconds))
        .toTranslation2d();

    Translation2d landingXY = botPose.getTranslation().plus(horizontalTravel);
    return new Translation3d(landingXY.getX(), landingXY.getY(), targetHeight.in(Meters));
  }

  /**
   * Predicts where the ball will land using the robot's <b>current</b> state
   * (pose and flywheel speed).
   *
   * @param targetHeight Z height of the intended target
   * @return Field-frame 3-D landing position
   */
  public static Translation3d predictBallLanding(Distance targetHeight) {
    requireConfigured();
    return predictBallLanding(targetHeight, driveSub.getPose(), launcherSub.getVelocity());
  }

  /**
   * Predicts the pass endpoint for debug/visualization. Uses the pass release
   * angle rather than the hub angle. Useful for AdvantageScope trajectory views.
   *
   * @param botPose        Robot pose at launch time
   * @param rpm            Flywheel speed
   * @param desiredHeading Field-relative desired bot heading
   * @return Field-frame 3-D landing position (z = 0, ground-plane shot)
   */
  public static Translation3d predictPassLanding(Pose2d botPose, AngularVelocity rpm,
      Rotation2d desiredHeading) {
    requireConfigured();

    double wheelLinearMps = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * rpm.in(RadiansPerSecond);
    double ballSpeedMps = wheelLinearMps * LauncherAndIntakeConstants.kWheelSlipCoefficient;

    double angle = LauncherAndIntakeConstants.kBallReleaseAngle().in(Radians);
    double Vz = ballSpeedMps * Math.sin(angle);
    double Vh = ballSpeedMps * Math.cos(angle);
    double releaseHeight = LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters);
    double g = PhysicsConstants.kGravityMps2;

    double discriminant = Vz * Vz + 2 * g * releaseHeight;
    if (discriminant < 0) {
      return new Translation3d(botPose.getX(), botPose.getY(), 0);
    }
    double t = (Vz + Math.sqrt(discriminant)) / g;

    double launchYaw = desiredHeading.getRadians()
        + LauncherAndIntakeConstants.kLauncherBotHeading.getRadians();

    // Robot velocity always included; zero when stationary, correct when moving
    ChassisSpeeds fieldSpeeds = driveSub.getChassisSpeedsFieldRelative();

    return new Translation3d(
        botPose.getX() + (Vh * Math.cos(launchYaw) + fieldSpeeds.vxMetersPerSecond) * t,
        botPose.getY() + (Vh * Math.sin(launchYaw) + fieldSpeeds.vyMetersPerSecond) * t,
        0);
  }

  /**
   * Builds a {@link ShotPrediction} for a hub shot using the robot's current
   * state. Checks whether the trajectory is blocked, out of field, or on-target.
   *
   * <p>
   * Call {@link #logShotPrediction} to publish results to NetworkTables /
   * AdvantageScope.
   *
   * @return Full shot prediction for the current launcher state
   */
  public static ShotPrediction predictHubShot() {
    requireConfigured();
    Translation3d hubTarget = PoseHelpers.getAllianceHubtTranslation3d();
    Translation3d landing = predictBallLanding(FieldConstants.kHubHeight);

    double airTimeSec = calculateAirTime(FieldConstants.kHubHeight, launcherSub.getVelocity()).in(Seconds);
    double distanceToHub = landing.toTranslation2d()
        .getDistance(hubTarget.toTranslation2d());

    boolean outOfField = !PoseHelpers.isInField(new edu.wpi.first.math.geometry.Pose2d(
        landing.toTranslation2d(), new Rotation2d()));

    boolean blockedByOwnHub = isTrajectoryBlockedByHub(
        driveSub.getPose().getTranslation(),
        landing.toTranslation2d(),
        PoseHelpers.getAllianceHubtTranslation2d(),
        FieldConstants.kHubInsideWidth.div(2).in(Meters));

    // Check enemy hub only if we are on the opposite side of the field
    boolean blockedByEnemyHub = isTrajectoryBlockedByHub(
        driveSub.getPose().getTranslation(),
        landing.toTranslation2d(),
        PoseHelpers.getEnemyHubTranslation2d(),
        FieldConstants.kHubInsideWidth.div(2).in(Meters));

    boolean willScore = distanceToHub < FieldConstants.kHubInsideWidth.div(2).in(Meters);

    return new ShotPrediction(
        willScore, landing, blockedByOwnHub, blockedByEnemyHub,
        outOfField, distanceToHub, airTimeSec);
  }

  /**
   * Convenience boolean: returns {@code true} if the current shot is predicted
   * to score in the hub. Equivalent to {@code predictHubShot().willScore()}.
   */
  public static boolean willHitHub() {
    return predictHubShot().willScore();
  }

  /**
   * Publishes a {@link ShotPrediction} to AdvantageScope and NetworkTables.
   * Call this once per command cycle from whichever command is actively
   * launching.
   *
   * @param prediction The prediction to log, from {@link #predictHubShot()}
   * @param prefix     Logging prefix (e.g. {@code "Commands/HubShot"})
   */
  public static void logShotPrediction(ShotPrediction prediction, String prefix) {
    Logger.recordOutput(prefix + "/WillScore", prediction.willScore());
    Logger.recordOutput(prefix + "/PredictedLanding", prediction.predictedLanding());
    Logger.recordOutput(prefix + "/BlockedByOwnHub", prediction.blockedByOwnHub());
    Logger.recordOutput(prefix + "/BlockedByEnemyHub", prediction.blockedByEnemyHub());
    Logger.recordOutput(prefix + "/OutOfField", prediction.outOfField());
    Logger.recordOutput(prefix + "/DistanceToTargetMeters", prediction.distanceToLanding());
    Logger.recordOutput(prefix + "/AirTimeSeconds", prediction.airTimeSeconds());
  }

  /**
   * Returns {@code true} if the robot is closer to the hub than the minimum safe
   * launch distance along the XY plane.
   */
  public static boolean isTooCloseToHub() {
    requireConfigured();
    double dist = driveSub.getPose().getTranslation()
        .getDistance(PoseHelpers.getAllianceHubtTranslation2d());
    boolean tooClose = Meters.of(dist).lt(LauncherAndIntakeConstants.kMinLaunchDistance);
    Logger.recordOutput("Launcher/TooClose/DistanceMeters", dist);
    Logger.recordOutput("Launcher/TooClose/IsTooClose", tooClose);
    return tooClose;
  }

  /**
   * Calculates how long a ball will be in the air before reaching
   * {@code targetHeight}.
   *
   * @param targetHeight Z height of the target
   * @param rpm          Flywheel speed
   * @return Predicted flight time
   */
  public static Time calculateAirTime(Distance targetHeight, AngularVelocity rpm) {
    Translation3d initialVelocity = calculateResultantVelocityVector(rpm);
    double Vz = initialVelocity.getZ();
    double g = PhysicsConstants.kGravityMps2;
    double deltaHeight = targetHeight.minus(LauncherAndIntakeConstants.kBallReleaseHeight).in(Meters);
    double discriminant = Vz * Vz - 2 * g * deltaHeight;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      // DriverStation.reportError(
      // "calculateAirTime: discriminant is NaN or negative (" + discriminant + ")",
      // false);
      return Seconds.zero();
    }

    double t = (Vz + Math.sqrt(discriminant)) / g;
    if (Double.isNaN(t) || t < 0) {
      // DriverStation.reportError(
      // String.format("calculateAirTime: non-positive flight time t=%.4fs", t),
      // false);
      return Seconds.zero();
    }

    return Seconds.of(t);
  }

  /**
   * Returns {@code true} if the straight-line path from {@code start} to
   * {@code end} on the XY plane passes within {@code hubRadius} of
   * {@code hubCenter} — i.e. the hub would block the shot.
   *
   * <p>
   * Uses the standard parametric closest-point-on-segment formula so only the
   * actual trajectory segment is tested, not its infinite extension.
   *
   * @param start     Robot XY position (field frame)
   * @param end       Predicted landing XY position (field frame)
   * @param hubCenter Hub center XY position
   * @param hubRadius Hub physical radius + safety margin
   * @return {@code true} when the hub blocks the shot
   */
  public static boolean isTrajectoryBlockedByHub(
      Translation2d start,
      Translation2d end,
      Translation2d hubCenter,
      double hubRadius) {

    double effectiveRadius = hubRadius + Constants.LauncherAndIntakeConstants.kEpsilonBuffer.in(Meters);

    // Vector from start to end
    Translation2d segment = end.minus(start);

    // Vector from start to hub center
    Translation2d toCenter = hubCenter.minus(start);

    double segmentLengthSquared = segment.getNorm() * segment.getNorm();

    // Handle degenerate case (start == end)
    if (segmentLengthSquared < 1e-9) {
      return start.getDistance(hubCenter) <= effectiveRadius;
    }

    // Project center onto segment (parameter t from 0 -> 1)
    double t = toCenter.dot(segment) / segmentLengthSquared;

    // Clamp t to segment bounds
    t = Math.max(0.0, Math.min(1.0, t));

    // Closest point on the segment to the hub center
    Translation2d closestPoint = start.plus(segment.times(t));

    // Distance from hub center to that closest point
    double distance = closestPoint.getDistance(hubCenter);

    return distance <= effectiveRadius;
  }

  private static void requireConfigured() {
    if (!configured) {
      throw new IllegalStateException(
          "LaunchHelpers used before setSubsystems() was called");
    }
  }
}