package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
   * Calculate the correct RPM to shoot at the hub from a certain distance
   * 
   * @param targetDistance The distance from the hub in the X-Y plane
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculateWheelRPM(Distance targetDistance) {
    // Use lookup table first, then polynomial regression
    // int lookupIndex = -1;
    // for (int i = 0; i < LauncherAndIntakeConstants.kLaunchDistancesLookup.length;
    // i++) {
    // Distance tableEntry = LauncherAndIntakeConstants.kLaunchDistancesLookup[i];
    // double distance = Math.abs(tableEntry.in(Meters) -
    // targetDistance.in(Meters));
    // double minDistance = 0;

    // if (distance < LauncherAndIntakeConstants.kLaunchLookupTolerance.in(Meters)
    // &&
    // distance < minDistance) {
    // lookupIndex = i;
    // }
    // }

    return LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(targetDistance);
  }

  /**
   * Calculate the correct RPM to shoot at the hub from the <b>current</b>
   * distance
   * 
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculateWheelRPM() {
    return calculateWheelRPM(drive().getHubDistance());
  }

  /**
   * Calculate how long the ball will be in the air with the provided wheel
   * velocity
   * 
   * @param targetHeight         The height of the target
   * @param wheelAngularVelocity Wheel speed
   * @return
   */
  public static Time calculateBallAirTime(Distance targetHeight, AngularVelocity wheelAngularVelocity) {
    Translation3d ballLaunchVelocityVectorMPS = calculateBallResultantVelocityVector(wheelAngularVelocity);
    double VzMPS = ballLaunchVelocityVectorMPS.getZ();

    double g = 9.81;
    double deltaHeightMeters = targetHeight.minus(LauncherAndIntakeConstants.kBallReleaseHeight).in(Meters);
    double discriminant = VzMPS * VzMPS - 2 * g * deltaHeightMeters;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      return Seconds.of(0);
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
   * @return
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
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallLaunchVelocityVector() {
    return calculateBallLaunchVelocityVector(calculateWheelRPM());
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
   * @param wheelAngularVelocity Flywheel speed
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
  public static LaunchSetpoints calculateLaunchSetpoints(Translation2d targetBotRelative, boolean applyLead) {
    AngularVelocity flywheelSpeed = calculateWheelRPM(Meters.of(targetBotRelative.getNorm()));

    if (!applyLead) {
      return new LaunchSetpoints(flywheelSpeed,
          targetBotRelative.getAngle().minus(LauncherAndIntakeConstants.kLauncherBotHeading));
    }

    // Get current velocity
    ChassisSpeeds currentChassisSpeeds = drive().getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond));

    // Get required intial velocity to hit target
    // This is
    Translation3d requiredVelocity = calculateBallLaunchVelocityVector(flywheelSpeed);

    // Calculate required contribution from launching (Vtotal = Vlaunch + Vbot =>
    // Vlaunch = Vtotal - Vbot)
    Translation3d launchVelocity = requiredVelocity.minus(botVelocity);

    return new LaunchSetpoints(calculateRequiredAngularVelocity(MetersPerSecond.of(launchVelocity.getNorm())),
        launchVelocity.toTranslation2d().getAngle().minus(LauncherAndIntakeConstants.kLauncherBotHeading));
  }

  /**
   * Calculate the launch setpoints (flywheel speed and bot heading) to hit the
   * hub, optionally applying lead to account for drivetrain velocity
   * 
   * @param targetBotRelative The target translation relative to the bot
   * @param applyLead         Whether to apply lead to account for drivetrain
   *                          velocity
   * @return The launch setpoints to hit the target
   */
  public static LaunchSetpoints calculateHubLaunchSetpoints(boolean applyLead) {
    return calculateLaunchSetpoints(drive().getHubTranslation2dBotRelative(), applyLead);
  }
}