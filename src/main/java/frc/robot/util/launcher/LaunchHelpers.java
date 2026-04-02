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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.PoseHelpers;

public class LaunchHelpers {

  /**
   * Record for the two values that define launch setpoints: flywheel speed
   * and bot heading.
   *
   * @param flywheelSpeed The flywheel speed to launch at
   * @param botHeading    The bot heading to aim towards the target
   */
  public record LaunchSetpoints(AngularVelocity flywheelSpeed, Rotation2d botHeading) {
  }

  // Do not access these directly, use launcher() and drive() to avoid NPE :skull.
  private static LauncherAndIntakeSubsystem launcherAndIntakeSub;
  private static DrivetrainSubsystem driveSub;
  private static boolean configured = false;

  public static void setSubsystems(DrivetrainSubsystem driveSubsystem,
      LauncherAndIntakeSubsystem launcherAndIntakeSubsystem) {
    if (configured) {
      throw new IllegalStateException("LauncherHelpers already configured");
    }
    launcherAndIntakeSub = Objects.requireNonNull(launcherAndIntakeSubsystem, "Launcher cannot be null");
    driveSub = Objects.requireNonNull(driveSubsystem, "Drivetrain cannot be null");
    configured = true;
  }

  static LauncherAndIntakeSubsystem launcher() {
    if (!configured) {
      throw new IllegalStateException("LauncherHelpers used before setSubsystems() was called");
    }
    return launcherAndIntakeSub;
  }

  static DrivetrainSubsystem drive() {
    if (!configured) {
      throw new IllegalStateException("LauncherHelpers used before setSubsystems() was called");
    }
    return driveSub;
  }

  /**
   * Public read-only accessor for the drivetrain. This is needed by subsystems
   * that
   * live outside this package (e.g. vision utilities) but must not call drive()
   * directly.
   */
  public static DrivetrainSubsystem driveSubsystem() {
    return drive();
  }

  /**
   * Predicts the endpoint of a ball launched with the given parameters.
   *
   *
   * @param targetHeight         Height of the target (final z of the ball)
   * @param botPose              Bot pose at launch time
   * @param wheelAngularVelocity Flywheel speed
   * @return 3-D field-frame endpoint of the ball
   */
  public static Translation3d predictBallEndpoint(Distance targetHeight, Pose2d botPose,
      AngularVelocity wheelAngularVelocity) {

    Translation3d ballInitialVelocityMPS = calculateBallResultantVelocityVector(wheelAngularVelocity);
    Time airTime = calculateBallAirTime(targetHeight, wheelAngularVelocity);

    // Find 2-D translation of flight path
    Translation2d ball2DTranslationMeters = ballInitialVelocityMPS.times(airTime.in(Seconds)).toTranslation2d();

    // // Rotate to launch in the correct direction using the supplied bot pose
    // ball2DTranslationMeters = new Translation2d(
    // ball2DTranslationMeters.getNorm(),
    // botPose.getRotation().minus(LauncherAndIntakeConstants.kLauncherBotHeading));

    // Add to current pose, and add back the z component
    return new Translation3d(botPose.getTranslation().plus(ball2DTranslationMeters))
        .plus(new Translation3d(0, 0, targetHeight.in(Meters)));
  }

  /**
   * Predicts the endpoint of a ball launched with the <b>current</b> launch
   * parameters.
   *
   * @param targetHeight Height of the target
   * @return 3-D field-frame endpoint of the ball
   */
  public static Translation3d predictBallEndpoint(Distance targetHeight) {
    return predictBallEndpoint(targetHeight, drive().getPose(), launcher().getVelocity());
  }

  /**
   * Predicts whether a ball launched with the given parameters will hit the
   * provided target within a certain tolerance.
   */
  public static boolean willHitTarget(Translation3d target, Pose2d botPose,
      Distance tolerance, AngularVelocity wheelAngularVelocity) {
    Translation3d ballEndpoint = predictBallEndpoint(target.getMeasureZ(), botPose, wheelAngularVelocity);
    return target.getDistance(ballEndpoint) < tolerance.in(Meters);
  }

  /**
   * Predicts whether a ball launched with the <b>current</b> parameters will
   * hit the provided target within tolerance.
   */
  public static boolean willHitTarget(Translation3d target, Distance tolerance) {
    return willHitTarget(target, drive().getPose(), tolerance, launcher().getVelocity());
  }

  /**
   * Predicts whether a ball launched with the <b>current</b> parameters will
   * score in the hub.
   */
  public static boolean willHitHub() {
    return willHitTarget(PoseHelpers.getAllianceHubtTranslation3d(), FieldConstants.kHubInsideWidth);
  }

  /**
   * Checks if the robot is too close to the hub to score fuel.
   *
   * @return {@code true} if the robot X distance to the hub is less than
   *         {@link LauncherAndIntakeConstants#kMinLaunchDistance}
   */
  public static boolean isTooCloseToHub() {
    double dist = drive().getPose().getTranslation().getDistance(PoseHelpers.getAllianceHubtTranslation2d());
    boolean tooClose = Meters.of(dist).lt(LauncherAndIntakeConstants.kMinLaunchDistance);
    Logger.recordOutput("Launcher/TooClose/Dist", dist);
    Logger.recordOutput("Launcher/TooClose/IsTooClose", tooClose);
    return tooClose;
  }

  /**
   * Calculates the correct RPM to launch at a target located at the given
   * horizontal distance and height.
   *
   * @param targetDistance Horizontal (XY-plane) distance to the target
   * @param targetHeight   Z height of the target
   * @return Ideal flywheel speed
   */
  public static AngularVelocity calculateWheelRPM(Distance targetDistance, Distance targetHeight) {

    // kDistanceToRPMCurve is tuned for the hub height. For targets at a
    // different height we invert the trajectory kinematics to find the
    // equivalent horizontal distance that maps to the same RPM.
    AngularVelocity omega0 = LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(targetDistance);

    if (targetHeight.isEquivalent(FieldConstants.kHubHeight)) {
      return omega0;
    }

    Translation3d initialVelocity = calculateBallLaunchVelocityVector(omega0);

    double V2d = initialVelocity.toTranslation2d().getNorm();
    double Vz = initialVelocity.getZ();

    // Solve z(x) = targetHeight for x using kinematics (no air-resistance model)
    double a = -9.81 / 2.0 / (V2d * V2d);
    double b = Vz / V2d;
    double c = -targetHeight.in(Meters);

    double discriminant = b * b - 4 * a * c;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      String msg = "calculateWheelRPM: discriminant is NaN or negative: " + discriminant;
      DriverStation.reportError(msg, false);
      return RPM.zero();
    }

    double newDistanceMeters = (-b + Math.sqrt(discriminant)) / (2.0 * a);
    return LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(Meters.of(newDistanceMeters));
  }

  /**
   * Calculates RPM for a ground-level pass using a flat launch angle.
   *
   * @param horizontalDistance XY-plane distance to the target
   * @return Required flywheel speed
   */
  public static AngularVelocity calculatePassRPM(Distance horizontalDistance) {
    double d = horizontalDistance.in(Meters);
    double releaseHeight = LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters);
    double angle = LauncherAndIntakeConstants.kPassReleaseAngle.in(Radians);
    double g = 9.81;

    double cosA = Math.cos(angle);
    double sinA = Math.sin(angle);
    double tanA = sinA / cosA;

    // V^2 = (g * d^2) / (2 * cos^2A * (d*tanA + releaseHeight))
    double denominator = 2 * cosA * cosA * (d * tanA + releaseHeight);

    if (denominator <= 0) {
      DriverStation.reportError("calculatePassRPM: invalid denominator " + denominator, false);
      return RPM.of(1000);
    }

    double vBallMPS = Math.sqrt((g * d * d) / denominator);
    return calculateRequiredAngularVelocity(MetersPerSecond.of(vBallMPS));
  }

  /**
   * Predicts the endpoint of a pass shot using the pass release angle.
   * Used only for debug visualization in AdvantageScope.
   *
   * @param botPose              Bot pose at launch time
   * @param wheelAngularVelocity Flywheel speed
   * @param desiredHeading       Bot heading
   * @return 3-D field-frame endpoint of the ball
   */
  public static Translation3d predictPassEndpoint(Pose2d botPose,
      AngularVelocity wheelAngularVelocity, Rotation2d desiredHeading) {
    double wheelLinearVelocityMPS = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * wheelAngularVelocity.in(RadiansPerSecond);
    double ballLinearVelocityMPS = wheelLinearVelocityMPS
        * LauncherAndIntakeConstants.kWheelSlipCoefficient;

    double angle = LauncherAndIntakeConstants.kPassReleaseAngle.in(Radians);
    double VzMPS = ballLinearVelocityMPS * Math.sin(angle);
    double VhMPS = ballLinearVelocityMPS * Math.cos(angle);
    double releaseHeight = LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters);
    double g = 9.81;

    double a = 0.5 * g;
    double b = -VzMPS;
    double c = -releaseHeight;
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
      return new Translation3d(botPose.getX(), botPose.getY(), 0);
    }

    double t = (-b + Math.sqrt(discriminant)) / (2 * a);

    // desiredHeading already accounts for kLauncherBotHeading offset
    // so the actual launch direction in field frame is:
    double actualLaunchYaw = desiredHeading.getRadians()
        + LauncherAndIntakeConstants.kLauncherBotHeading.getRadians();

    double dx = VhMPS * Math.cos(actualLaunchYaw) * t;
    double dy = VhMPS * Math.sin(actualLaunchYaw) * t;

    Logger.recordOutput("ZonePass/Debug/AirTime", t);
    Logger.recordOutput("ZonePass/Debug/VhMPS", VhMPS);
    Logger.recordOutput("ZonePass/Debug/ActualLaunchYawDeg", Math.toDegrees(actualLaunchYaw));

    return new Translation3d(botPose.getX() + dx, botPose.getY() + dy, 0);
  }

  /**
   * Calculates the correct RPM for launching at the hub from the <b>current</b>
   * distance.
   */
  public static AngularVelocity calculateLaunchRPM() {
    return calculateWheelRPM(drive().getHubDistance(), FieldConstants.kHubHeight);
  }

  /**
   * Calculates how long a ball will be in the air given the target height and
   * flywheel speed.
   *
   * @param targetHeight         Height of the target
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball air time
   */
  public static Time calculateBallAirTime(Distance targetHeight,
      AngularVelocity wheelAngularVelocity) {

    Translation3d ballInitialVelocity = calculateBallResultantVelocityVector(wheelAngularVelocity);
    double VzMPS = ballInitialVelocity.getZ();

    double g = 9.81;
    double deltaHeightMeters = targetHeight.minus(LauncherAndIntakeConstants.kBallReleaseHeight).in(Meters);
    double discriminant = VzMPS * VzMPS - 2 * g * deltaHeightMeters;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      String msg = "calculateBallAirTime: discriminant is NaN or negative: " + discriminant;
      DriverStation.reportError(msg, false);
      return Seconds.zero();
    }

    double tSeconds = (VzMPS + Math.sqrt(discriminant)) / g;

    if (Double.isNaN(tSeconds) || tSeconds < 0) {
      String msg = String.format("calculateBallAirTime: non-positive flight time t=%.6f. Returning 0s.", tSeconds);
      DriverStation.reportError(msg, false);
      return Seconds.zero();
    }

    return Seconds.of(tSeconds);
  }

  /**
   * Calculates how long a ball will be in the air with the <b>current</b>
   * flywheel speed.
   *
   * @param targetHeight Height of the target
   * @return Ball air time
   */
  public static Time calculateBallAirTime(Distance targetHeight) {
    return calculateBallAirTime(targetHeight, launcher().getVelocity());
  }

  /**
   * Calculates the launch velocity vector of the ball in field coordinates for
   * a given flywheel speed.
   *
   * <p>
   * This is the launcher contribution only and does <em>not</em> include
   * drivetrain velocity.
   *
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball launch velocity (m/s) as a field-frame {@link Translation3d}
   */
  public static Translation3d calculateBallLaunchVelocityVector(
      AngularVelocity wheelAngularVelocity) {

    double wheelLinearVelocityMPS = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * wheelAngularVelocity.in(RadiansPerSecond);

    double ballLinearVelocityMPS = wheelLinearVelocityMPS * LauncherAndIntakeConstants.kWheelSlipCoefficient;

    // Robot-relative launch direction
    Translation3d launchVelocity = new Translation3d(
        Math.cos(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians)),
        0,
        Math.sin(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians)))
        .times(ballLinearVelocityMPS)
        .rotateBy(new Rotation3d(LauncherAndIntakeConstants.kLauncherBotHeading));

    // Field relative coordinates
    launchVelocity = launchVelocity.rotateBy(new Rotation3d(drive().getPose().getRotation()));

    return launchVelocity;
  }

  /**
   * Calculates the launch velocity vector for the <b>current</b> flywheel speed.
   *
   * <p>
   * This is the launcher contribution only (no drivetrain velocity).
   */
  public static Translation3d calculateBallLaunchVelocityVector() {
    return calculateBallLaunchVelocityVector(launcher().getVelocity());
  }

  /**
   * Calculates the resultant ball velocity (launcher + drivetrain) for the given
   * flywheel speed, in field coordinates.
   *
   * @param wheelAngularVelocity Flywheel speed
   * @return Resultant ball velocity (m/s) as a field-frame {@link Translation3d}
   */
  public static Translation3d calculateBallResultantVelocityVector(
      AngularVelocity wheelAngularVelocity) {

    ChassisSpeeds speeds = drive().getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);

    return calculateBallLaunchVelocityVector(wheelAngularVelocity).plus(botVelocity);
  }

  /**
   * Calculates the resultant ball velocity for the <b>current</b> flywheel speed,
   * in field coordinates.
   */
  public static Translation3d calculateBallResultantVelocityVector() {
    return calculateBallResultantVelocityVector(launcher().getVelocity());
  }

  /**
   * Calculates the angular velocity required to produce a given ball linear
   * launch speed.
   *
   * @param ballLaunchVelocity Desired ball linear velocity
   * @return Required flywheel speed
   */
  public static AngularVelocity calculateRequiredAngularVelocity(LinearVelocity ballLaunchVelocity) {
    double wheelLinearVelocityMPS = ballLaunchVelocity.in(MetersPerSecond)
        / LauncherAndIntakeConstants.kWheelSlipCoefficient
        / LauncherAndIntakeConstants.kWheelRadius.in(Meters);
    return RadiansPerSecond.of(wheelLinearVelocityMPS);
  }

  /**
   * Calculates the required flywheel speed and bot heading to hit an arbitrary
   * 3-D target, optionally applying lead correction for drivetrain velocity.
   *
   * @param targetBotRelative 3-D vector from the robot to the target
   *                          (field-frame translation, bot origin)
   * @param applyLead         whether to compensate for drivetrain velocity
   * @return {@link LaunchSetpoints} containing flywheel RPM and bot heading
   */
  public static LaunchSetpoints calculateLaunchSetpoints(
      Translation3d targetBotRelative, boolean applyLead) {

    Translation2d targetBotRelative2d = targetBotRelative.toTranslation2d();

    AngularVelocity flywheelSpeed = calculateWheelRPM(
        Meters.of(targetBotRelative2d.getNorm()),
        targetBotRelative.getMeasureZ());

    if (!applyLead) {
      return new LaunchSetpoints(
          flywheelSpeed,
          targetBotRelative2d.getAngle()
              .minus(LauncherAndIntakeConstants.kLauncherBotHeading));
    }

    ChassisSpeeds currentChassisSpeeds = drive().getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(
        currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond, 0);

    // Velocity needed to reach the target
    Translation3d requiredVelocity = calculateBallLaunchVelocityVector(flywheelSpeed);

    // Override 2-D direction to face target; preserve Z (pitch)
    requiredVelocity = new Translation3d(
        new Translation2d(requiredVelocity.toTranslation2d().getNorm(),
            targetBotRelative2d.getAngle()))
        .plus(new Translation3d(0, 0, requiredVelocity.getZ()));

    // Subtract drivetrain contribution: Vlaunch = Vtotal - Vbot
    Translation3d launchVelocity = requiredVelocity.minus(botVelocity);

    Rotation2d desiredHeading = launchVelocity.toTranslation2d().getAngle()
        .minus(LauncherAndIntakeConstants.kLauncherBotHeading);

    return new LaunchSetpoints(
        calculateRequiredAngularVelocity(
            MetersPerSecond.of(launchVelocity.getNorm())),
        desiredHeading);
  }

  /**
   * Calculates the launch setpoints to hit the hub, optionally with lead
   * correction.
   *
   * @param applyLead whether to compensate for drivetrain velocity
   * @return {@link LaunchSetpoints} for the hub
   */
  public static LaunchSetpoints calculateHubLaunchSetpoints(boolean applyLead) {
    return calculateLaunchSetpoints(drive().getHubTranslation3dBotRelative(), applyLead);
  }
}