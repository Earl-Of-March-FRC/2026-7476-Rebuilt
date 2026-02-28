package frc.robot.util.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;

public class LaunchHelpers {

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
  public static Translation2d predictBallEndpoint(Distance targetHeight, Pose2d botPose,
      AngularVelocity wheelAngularVelocity) {
    LinearVelocity ballLaunchVelocity = calculateBallLaunchVelocity(wheelAngularVelocity);
    Time airTime = calculateBallAirTime(targetHeight, wheelAngularVelocity);

    double VxMPS = Math.cos(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians))
        * ballLaunchVelocity.in(MetersPerSecond);

    return botPose.getTranslation()
        .plus(new Translation2d(VxMPS * airTime.in(Seconds), botPose.getRotation()));
  }

  /**
   * Predicts the endpoint of a ball launched with the current launch parameters
   * 
   * @param targetHeight The height of the target (final height of the ball)
   * @return The Translation2d of the ball endpoint
   */
  public static Translation2d predictBallEndpoint(Distance targetHeight) {
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

    Translation2d ballEndpoint = predictBallEndpoint(target.getMeasureZ(), botPose, wheelAngularVelocity);

    return target.toTranslation2d().getDistance(ballEndpoint) < tolerance.in(Meters);
  }

  /**
   * Predicts whether a ball launched with the current launch parameters will hit
   * the provided target within a certain tolerance.
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
   * Calculate the correct RPM to shoot at the hub from a certain distance
   * 
   * @param targetDistance The distance from the hub in the X-Y plane
   * @return The ideal wheel speed
   */
  public static AngularVelocity calculateWheelRPM(Distance targetDistance) {
    // Use lookup table first, then polynomial regression
    int lookupIndex = -1;
    for (int i = 0; i < LauncherAndIntakeConstants.kLaunchDistancesLookup.length; i++) {
      Distance tableEntry = LauncherAndIntakeConstants.kLaunchDistancesLookup[i];
      double distance = Math.abs(tableEntry.in(Meters) - targetDistance.in(Meters));
      double minDistance = 0;

      if (distance < LauncherAndIntakeConstants.kLaunchLookupTolerance.in(Meters) &&
          distance < minDistance) {
        lookupIndex = i;
      }
    }

    return lookupIndex == -1 ? LauncherAndIntakeConstants.kDistanceToRPMCurve.apply(targetDistance)
        : LauncherAndIntakeConstants.kLaunchWheelSpeedLookup[lookupIndex];
  }

  /**
   * Calculate the correct RPM to shoot at the hub from the current distance
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
    LinearVelocity ballLaunchVelocity = calculateBallLaunchVelocity(wheelAngularVelocity);

    double g = 9.81;

    double VyMPS = Math.sin(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians))
        * ballLaunchVelocity.in(MetersPerSecond);

    double deltaHeightMeters = targetHeight.minus(LauncherAndIntakeConstants.kBallReleaseHeight).in(Meters);

    double tSeconds = (VyMPS + Math.sqrt(VyMPS * VyMPS - 2 * g * deltaHeightMeters)) / g;

    return Seconds.of(tSeconds);
  }

  /**
   * Calculate how long the ball will be in the air with the current wheel speed
   * 
   * @param targetHeight The height of the target
   * @return
   */
  public static Time calculateBallAirTime(Distance targetHeight) {
    return calculateBallAirTime(targetHeight, launcher().getVelocity());
  }

  /**
   * Calculates the launch velocity of the ball with the given flywheel speed
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity
   */
  public static LinearVelocity calculateBallLaunchVelocity(AngularVelocity wheelAngularVelocity) {
    double wheelLinearVeocityMPS = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * wheelAngularVelocity.in(RadiansPerSecond);

    return MetersPerSecond.of(LauncherAndIntakeConstants.kWheelSlipCoefficient * wheelLinearVeocityMPS);
  }

  /**
   * Calculates the launch velocity of the ball with the current flywheel speed
   * 
   * @return Ball linear velocity
   */
  public static LinearVelocity calculateBallLaunchVelocity() {
    return calculateBallLaunchVelocity(launcher().getVelocity());
  }
}