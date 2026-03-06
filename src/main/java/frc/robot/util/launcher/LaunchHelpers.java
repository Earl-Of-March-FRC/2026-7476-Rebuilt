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
  public static Translation2d predictBallEndpoint(Distance targetHeight, Pose2d botPose,
      AngularVelocity wheelAngularVelocity) {
    LinearVelocity ballLaunchVelocity = calculateBallLaunchVelocity(wheelAngularVelocity);
    Time airTime = calculateBallAirTime(targetHeight, wheelAngularVelocity);

    double VxMPS = Math.cos(LauncherAndIntakeConstants.kBallReleaseAngle.in(Radians))
        * ballLaunchVelocity.in(MetersPerSecond);

    return botPose.getTranslation()
        .plus(new Translation2d(VxMPS * airTime.in(Seconds),
            botPose.getRotation().plus(LauncherAndIntakeConstants.kLauncherBotHeading)));
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

    double discriminant = VyMPS * VyMPS - 2 * g * deltaHeightMeters;

    if (Double.isNaN(discriminant) || discriminant < 0) {
      return Seconds.of(0);
    }

    double tSeconds = (VyMPS + Math.sqrt(discriminant)) / g;

    if (Double.isNaN(tSeconds) || tSeconds < 0) {
      String msg = String.format("Computed non-positive flight time: t=%.6f. Returning 0s.", tSeconds);
      DriverStation.reportError(msg, false);
      return Seconds.of(0);
    }

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
   * Calculates the launch velocity of the ball with the given flywheel speed,
   * accounting for bot speeds
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity
   */
  public static LinearVelocity calculateBallLaunchVelocity(AngularVelocity wheelAngularVelocity) {
    return MetersPerSecond.of(calculateBallLaunchVelocityVector(wheelAngularVelocity).getNorm());
  }

  /**
   * Calculates the launch velocity of the ball with the given flywheel speed,
   * accounting for bot speeds
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallLaunchVelocityVector(AngularVelocity wheelAngularVelocity) {
    double wheelLinearVelocityMPS = LauncherAndIntakeConstants.kWheelRadius.in(Meters)
        * wheelAngularVelocity.in(RadiansPerSecond);

    // Get current velocity
    ChassisSpeeds currentChassisSpeeds = drive().getChassisSpeedsFieldRelative();
    Translation3d botVelocity = new Translation3d(new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond));

    Translation3d launchVelocity = new Translation3d(
        LauncherAndIntakeConstants.kWheelSlipCoefficient * wheelLinearVelocityMPS,
        new Rotation3d(drive().getPose().getRotation().minus(LauncherAndIntakeConstants.kLauncherBotHeading))
            .rotateBy(new Rotation3d(Degrees.zero(), LauncherAndIntakeConstants.kBallReleaseAngle.unaryMinus(),
                Degrees.zero())));

    Logger.recordOutput("LaunchHelpers/LaunchVelocity", launchVelocity);

    return botVelocity.plus(launchVelocity);
  }

  /**
   * Calculates the launch velocity of the ball with the current flywheel speed,
   * accounting for bot speeds
   * 
   * @param wheelAngularVelocity Flywheel speed
   * @return Ball linear velocity as a Translation3d, in mps
   */
  public static Translation3d calculateBallLaunchVelocityVector() {
    return calculateBallLaunchVelocityVector(calculateWheelRPM());
  }

  /**
   * Calculates the launch velocity of the ball with the current flywheel speed
   * 
   * @return Ball linear velocity
   */
  public static LinearVelocity calculateBallLaunchVelocity() {
    return calculateBallLaunchVelocity(launcher().getVelocity());
  }

  /**
   * Calculates the required flywheel speed to launch the ball at a certain
   * velocity
   * 
   * This method is the inverse function of
   * {@link #calculateBallLaunchVelocity(AngularVelocity)}
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
  public static LaunchSetpoints calculateLaunchSetpoints(Translation2d targetBotRelative, Boolean applyLead) {
    AngularVelocity flywheelSpeed = calculateWheelRPM(Meters.of(targetBotRelative.getNorm()));

    if (!applyLead) {
      return new LaunchSetpoints(flywheelSpeed,
          targetBotRelative.getAngle().minus(LauncherAndIntakeConstants.kLauncherBotHeading));
    }

    // Get current velocity
    ChassisSpeeds currentChassisSpeeds = drive().getChassisSpeedsFieldRelative();
    Translation2d botVelocity = new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond);

    // Get required intial velocity to hit target
    Translation2d requiredVelocity = new Translation2d(
        calculateBallLaunchVelocity(flywheelSpeed).in(MetersPerSecond),
        targetBotRelative.getAngle());

    // Calculate required contribution from launching (Vtotal = Vlaunch + Vbot =>
    // Vlaunch = Vtotal - Vbot)
    Translation2d launchVelocity = requiredVelocity.minus(botVelocity);

    return new LaunchSetpoints(calculateRequiredAngularVelocity(MetersPerSecond.of(launchVelocity.getNorm())),
        launchVelocity.getAngle().minus(LauncherAndIntakeConstants.kLauncherBotHeading));
  }
}