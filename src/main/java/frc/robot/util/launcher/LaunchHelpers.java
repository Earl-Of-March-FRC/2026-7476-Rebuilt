package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.PoseHelpers;

public class LaunchHelpers {

  /**
   * Container for the two values that define launch setpoints: flywheel speed and
   * bot heading
   * 
   * @param flywheelSpeed The flywheel speed to launch at
   * @param botHeading    The bot heading to launch towerds
   */
  public record LaunchSetpoints(AngularVelocity flywheelSpeed, Rotation2d botHeading) {
  }

  // Do not access launcherAndIntakeSub directly, use Launcher() to avoid NPE
  private static LauncherAndIntakeSubsystem launcherAndIntakeSub;
  private static DrivetrainSubsystem driveSub;
  private static boolean configured = false;
  private static Distance minLaunchDistance = null;

  public static void setSubsystems(DrivetrainSubsystem driveSubsystem,
      LauncherAndIntakeSubsystem launcherAndIntakeSubsystem) {
    if (configured) {
      throw new IllegalStateException("LauncherHelpers already configured");
    }
    launcherAndIntakeSub = Objects.requireNonNull(launcherAndIntakeSubsystem, "Launcher cannot be null");
    driveSub = Objects.requireNonNull(driveSubsystem, "Drivetrain cannot be null");
    configured = true;
  }

  private static LauncherAndIntakeSubsystem launcher() {
    if (!configured) {
      throw new IllegalStateException(
          "LauncherHelpers used before setSubsystems() was called");
    }
    return launcherAndIntakeSub;
  }

  private static DrivetrainSubsystem drive() {
    if (!configured) {
      throw new IllegalStateException(
          "LauncherHelpers used before setSubsystems() was called");
    }
    return driveSub;
  }

  /**
   * Predicts the endpoint of a ball launched with the provided launch parameters
   * 
   * @param targetHeight         The height of the target (final height of the
   *                             ball)
   * @param botPose              Bot Pose2d
   * @param wheelAngularVelocity Flywheel speed
   * @return The Translation2d of the ball endpoint
   */
  public static Translation3d predictBallEndpoint(Distance targetHeight, Pose2d botPose,
      AngularVelocity wheelAngularVelocity) {

    Translation3d ballInitialVelocityMPS = calculateBallResultantVelocityVector(wheelAngularVelocity);
    Time airTime = calculateBallAirTime(targetHeight, wheelAngularVelocity);

    // Find 2d translation of flight path
    Translation2d ball2DTranslationMeters = ballInitialVelocityMPS.times(airTime.in(Seconds)).toTranslation2d();

    // Rotate to launch in the correct direction
    ball2DTranslationMeters = new Translation2d(
        ball2DTranslationMeters.getNorm(),
        drive().getPose().getRotation().minus(LauncherAndIntakeConstants.kLauncherBotHeading));

    // Add to current pose, and add back the z component
    return new Translation3d(drive().getPose().getTranslation().plus(ball2DTranslationMeters))
        .plus(new Translation3d(0, 0, targetHeight.in(Meters)));
  }

  /**
   * Predicts the endpoint of a ball launched with the <b>current</b> launch
   * parameters
   * 
   * @param targetHeight The height of the target (final height of the ball)
   * @return The Translation2d of the ball endpoint
   */
  public static Translation3d predictBallEndpoint(Distance targetHeight) {
    return predictBallEndpoint(targetHeight, drive().getPose(), launcher().getVelocity());
  }

  /**
   * Predicts whether a ball launched with the provided launch parameters will hit
   * the provided target within a certain tolerance.
   * 
   * @param target               Target Translation3d
   * @param botPose              Bot Pose2d
   * @param tolerance            Tolerance of the target
   * @param wheelAngularVelocity Flywheel speed
   * @return True if the ball with hit the target within tolerances, false
   *         otherwise
   */
  public static boolean willHitTarget(Translation3d target, Pose2d botPose, Distance tolerance,
      AngularVelocity wheelAngularVelocity) {

    Translation3d ballEndpoint = predictBallEndpoint(target.getMeasureZ(), botPose, wheelAngularVelocity);

    return target.getDistance(ballEndpoint) < tolerance.in(Meters);
  }

  /**
   * Predicts whether a ball launched with the <b>current</b> launch parameters
   * will hit the provided target within a certain tolerance.
   * 
   * @param target    Target Translation3d
   * @param tolerance Tolerance of the target
   * @return True if the ball with hit the target within tolerances, false
   *         otherwise
   */
  public static boolean willHitTarget(Translation3d target, Distance tolerance) {
    return willHitTarget(target, drive().getPose(), tolerance, launcher().getVelocity());
  }

  /**
   * Predicts whether a ball launched with the <b>current</b> launch parameters
   * will score in the hub.
   * 
   * @return True if the ball will score
   */
  public static boolean willHitHub() {
    return willHitTarget(PoseHelpers.getAllianceHubtTranslation3d(), FieldConstants.kHubInsideWidth);
  }

  /**
   * Checks if the robot is too close to the hub to score fuel
   * 
   * @return {@code true} if the robot is so close to the hub that
   *         {@link #calculateWheelRPM} would return zero (i.e. the ball cannot
   *         physically reach hub height from this distance regardless of flywheel
   *         speed).
   *         <p>
   *         Use this in driving commands to trigger a "back away from hub"
   *         behaviour
   *         before trying to shoot.
   */
  public static boolean isTooCloseToHub() {
    double xDist = Math.abs(
        drive().getPose().getX() - PoseHelpers.getAllianceHubtTranslation2d().getX());
    boolean tooClose = Meters.of(xDist).lt(LauncherAndIntakeConstants.kMinLaunchDistance);
    Logger.recordOutput("Launcher/TooClose/XDist", xDist);
    Logger.recordOutput("Launcher/TooClose/IsTooClose", tooClose);
    return tooClose;
  }

  /**
   * Calculate the correct RPM to launching at the hub from a certain distance
   * 
   * @param targetDistance The distance from the target in the X-Y plane
   * @param targetHeight   The height of the target
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculateWheelRPM(Distance targetDistance, Distance targetHeight) {

    // kDistanceToRPMCurve is tuned for a 6ft target (the hub), we can calulate the
    // RPM for other heights if we ignore air resistance

    // Required RPM for a hub height target
    AngularVelocity omega0 = LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(targetDistance);

    if (targetHeight.isEquivalent(FieldConstants.kHubHeight)) {
      return omega0;
    }

    // if the target is a different height, solve for height as a function of 2d
    // displacement, and find the distance where the ball will be at hub height.
    // This distance will result in the correct RPM to reach a height of 0 after the
    // orignal target distance

    Translation3d intitialVelocity = calculateBallLaunchVelocityVector(omega0);

    double V2d = intitialVelocity.toTranslation2d().getNorm();
    double Vz = intitialVelocity.getZ();

    // Model z(x) as a quadratic (no air resistance or spin) and solve for the
    // coefficients with kinematics, then solve z(x) = targetHeight.in(Meters) =>
    // z(x) - targetHeight.in(Meters) = 0

    double a = -9.81 / 2.0 / (V2d * V2d);
    double b = Vz / V2d;
    double c = -targetHeight.in(Meters);

    double discriminant = b * b - 4 * a * c;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      String msg = "Calculate Wheel RPM: Discriminant is NAN or less than 0: discriminant = " + discriminant;
      DriverStation.reportError(msg, false);
      return RPM.zero();
    }

    // Take larger root
    double newDistanceMeters = (-b + Math.sqrt(discriminant)) / (2.0 * a);

    return LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(Meters.of(newDistanceMeters));
  }

  /**
   * Calculate the correct RPM for launching at the hub from the <b>current</b>
   * distance
   * 
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculateLaunchRPM() {
    return calculateWheelRPM(drive().getHubDistance(), FieldConstants.kHubHeight);
  }

  /**
   * Calculate the correct RPM for passing from the <b>current</b>
   * distance
   * 
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculatePassRPM() {
    boolean isBlue = PoseHelpers.getAlliance() == Alliance.Blue;

    Distance passTargetX = isBlue ? FieldConstants.kAllianceZoneXLength.div(2)
        : FieldConstants.kFieldLengthX.minus(FieldConstants.kAllianceZoneXLength);

    Distance passTargetDistance = drive().getPose().getMeasureX().minus(passTargetX);
    // Take absolute value
    passTargetDistance = Meters.of(Math.abs(passTargetDistance.in(Meters)));

    return calculateWheelRPM(passTargetDistance, Meters.zero());
  }

  /**
   * Calculate how long the ball will be in the air with the provided wheel
   * velocity
   * 
   * @param targetHeight         The height of the target
   * @param wheelAngularVelocity Wheel speed
   * @return The airtime of the ball
   */
  public static Time calculateBallAirTime(Distance targetHeight, AngularVelocity wheelAngularVelocity) {
    Translation3d ballInitialVelocityVectorMPS = calculateBallResultantVelocityVector(wheelAngularVelocity);
    double VzMPS = ballInitialVelocityVectorMPS.getZ();

    double g = 9.81;
    double deltaHeightMeters = targetHeight.minus(LauncherAndIntakeConstants.kBallReleaseHeight).in(Meters);
    double discriminant = VzMPS * VzMPS - 2 * g * deltaHeightMeters;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      String msg = "Calculate Ball Air Time: Discriminant is NAN or less than 0: discriminant = " + discriminant;
      DriverStation.reportError(msg, false);
      return Seconds.zero();
    }

    double tSeconds = (VzMPS + Math.sqrt(discriminant)) / g;

    if (Double.isNaN(tSeconds) || tSeconds < 0) {
      String msg = String.format("Computed non-positive flight time: t=%.6f. Returning 0s.", tSeconds);
      DriverStation.reportError(msg, false);
      return Seconds.of(0);
    }

    return Seconds.of(tSeconds);
  }

  /**
   * Calculate how long the ball will be in the air with the <b>current</b> wheel
   * speed
   * 
   * @param targetHeight The height of the target
   * @return The airtime of the ball
   */
  public static Time calculateBallAirTime(Distance targetHeight) {
    return calculateBallAirTime(targetHeight, launcher().getVelocity());
  }

  /**
   * Calculates the launch velocity of the ball with the given flywheel speed, in
   * field coordinates
   * 
   * This is only the launcher contribution, and does not account for bot velocity
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallLaunchVelocityVector(AngularVelocity wheelAngularVelocity) {
    double wheelLinearVelocityMPS = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * wheelAngularVelocity.in(RadiansPerSecond);

    double ballLinearVelocityMPS = wheelLinearVelocityMPS * LauncherAndIntakeConstants.kWheelSlipCoefficient;

    // Robot relative coordinates
    Translation3d launchVelocity = new Translation3d(Math.cos(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians)),
        0, Math.sin(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians))).times(ballLinearVelocityMPS)
        // Rotate to launch in the correct direction
        .rotateBy(new Rotation3d(LauncherAndIntakeConstants.kLauncherBotHeading));

    // Field relative coordinates
    launchVelocity = launchVelocity.rotateBy(new Rotation3d(drive().getPose().getRotation().unaryMinus()));

    return launchVelocity;
  }

  /**
   * Calculates the launch velocity of the ball with the <b>current</b> flywheel
   * speed, in field coordinates
   * 
   * This is only the launcher contribution, and does not account for bot velocity
   * 
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallLaunchVelocityVector() {
    return calculateBallLaunchVelocityVector(calculateLaunchRPM());
  }

  /**
   * Calculates the launch velocity of the ball with the given flywheel speed, in
   * field coordinates
   * 
   * This takes into acoount the velocity of the bot
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallResultantVelocityVector(AngularVelocity wheelAngularVelocity) {
    // Get bot speeds
    ChassisSpeeds speeds = drive().getChassisSpeedsFieldRelative();

    Translation3d botVelocity = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);

    return calculateBallLaunchVelocityVector().plus(botVelocity);
  }

  /**
   * Calculates the launch velocity of the ball with the <b>current</b> flywheel
   * speed, in
   * field coordinates
   * 
   * This takes into acoount the velocity of the bot
   * 
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallResultantVelocityVector() {
    return calculateBallResultantVelocityVector(launcher().getVelocity());
  }

  /**
   * Calculates the required flywheel speed to launch the ball at a certain
   * velocity
   * 
   * 
   * @param ballLaunchVelocity Desired ball linear velocity
   * @return Required flywheel speed
   */
  public static AngularVelocity calculateRequiredAngularVelocity(LinearVelocity ballLaunchVelocity) {
    double wheelLinearVelocityMPS = ballLaunchVelocity.in(MetersPerSecond)
        / LauncherAndIntakeConstants.kWheelSlipCoefficient / LauncherAndIntakeConstants.kWheelRadius.in(Meters);

    return RadiansPerSecond.of(wheelLinearVelocityMPS);
  }

  /**
   * Calculate the launch setpoints (flywheel speed and bot heading) to hit a
   * target, optionally applying lead to account for drivetrain velocity
   * 
   * @param targetBotRelative The target translation relative to the bot
   * @param applyLead         Whether to apply lead to account for drivetrain
   *                          velocity
   * @return The launch setpoints to hit the target
   */
  public static LaunchSetpoints calculateLaunchSetpoints(Translation3d targetBotRelative, boolean applyLead) {
    Translation2d targetBotRelative2d = targetBotRelative.toTranslation2d();

    AngularVelocity flywheelSpeed = calculateWheelRPM(Meters.of(targetBotRelative2d.getNorm()),
        targetBotRelative.getMeasureZ());

    if (!applyLead) {
      return new LaunchSetpoints(flywheelSpeed,
          targetBotRelative2d.getAngle().minus(LauncherAndIntakeConstants.kLauncherBotHeading));
    }

    // Get current velocity
    ChassisSpeeds currentChassisSpeeds = drive().getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond));

    // Logger.recordOutput("Debug/botVelocity", botVelocity);

    // Get required intial velocity to hit target
    // This is
    Translation3d requiredVelocity = calculateBallLaunchVelocityVector(flywheelSpeed);

    // Override pitch to face target
    // Project to 2d, override rotation, then add back z component
    requiredVelocity = new Translation3d(
        new Translation2d(requiredVelocity.toTranslation2d().getNorm(), targetBotRelative2d.getAngle()))
        .plus(new Translation3d(0, 0, requiredVelocity.getZ()));

    // Logger.recordOutput("Debug/requiredVelocity", requiredVelocity);

    // Calculate required contribution from launching (Vtotal = Vlaunch + Vbot =>
    // Vlaunch = Vtotal - Vbot)
    Translation3d launchVelocity = requiredVelocity.minus(botVelocity);

    // Logger.recordOutput("Debug/launchVelocity", launchVelocity);

    Rotation2d desiredHeading = launchVelocity.toTranslation2d().getAngle()
        .minus(LauncherAndIntakeConstants.kLauncherBotHeading);

    // Logger.recordOutput("Debug/desiredHeading", desiredHeading);

    return new LaunchSetpoints(calculateRequiredAngularVelocity(MetersPerSecond.of(launchVelocity.getNorm())),
        desiredHeading);
  }

  /**
   * Calculate the launch setpoints (flywheel speed and bot heading) to hit the
   * hub, optionally applying lead to account for drivetrain velocity
   * 
   * @param applyLead Whether to apply lead to account for drivetrain
   *                  velocity
   * @return The launch setpoints to hit the target
   */
  public static LaunchSetpoints calculateHubLaunchSetpoints(boolean applyLead) {
    return calculateLaunchSetpoints(drive().getHubTranslation3dBotRelative(), applyLead);
  }
}