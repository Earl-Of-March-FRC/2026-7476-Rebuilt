// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.launcher.LaunchHelpers.LaunchSetpoints;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.SwerveConfig;

/* 
 * Command for driving while tracking the hub.
 * Note on coordinate system: "bot-relative" in this command means that the vectors (like toHub)
 * are still in blue field coordinate system, just with the origin at the robot's position.
*/
public class DriveTrackHubCmd extends Command {

  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final BooleanSupplier shouldLockRangeSupplier;
  private final boolean leadShots;

  private boolean atLimit;
  private Distance launchingRange;

  /**
   * Create a new DriveTrackHubCmd
   * 
   * @param driveSub                The drivetrain subsystem
   * @param xSupplier               Supplier for X-axis input (-1 to 1,
   *                                field-relative)
   * @param ySupplier               Supplier for Y-axis input (-1 to 1,
   *                                field-relative)
   * @param leadShots               Whether to lead shots based on current
   *                                velocity
   * @param shouldLockRangeSupplier Whether or not to lock the range at the
   *                                current distance
   */
  public DriveTrackHubCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      boolean leadShots,
      BooleanSupplier shouldLockRangeSupplier) {

    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.leadShots = leadShots;
    this.shouldLockRangeSupplier = shouldLockRangeSupplier;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    driveSub.resetControllers();
  }

  @Override
  public void execute() {
    Translation3d toHub3d = driveSub.getHubTranslation3dBotRelative();
    Translation2d toHub = toHub3d.toTranslation2d();
    launchingRange = Meters.of(toHub.getNorm());

    LaunchSetpoints launchSetpoints = LaunchHelpers.calculateLaunchSetpoints(toHub3d, leadShots);
    Rotation2d desiredHeading = launchSetpoints.botHeading();
    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(desiredHeading);

    // Process input based on shouldLockRangeSupplier
    ChassisSpeeds speeds = null;
    Pose2d futurePose = null;
    if (shouldLockRangeSupplier.getAsBoolean()) {
      // Calculate the vector perpendicular to the vector from the robot to the hub,
      // and project the joystick input onto that vector
      Translation2d perpendicularUnitVelocity = toHub.rotateBy(new Rotation2d(Math.PI / 2.0));
      perpendicularUnitVelocity = perpendicularUnitVelocity.div(toHub.getNorm());

      Translation2d joystickInput = new Translation2d(xSupplier.get(), ySupplier.get());

      // Invert for red alliance to maintain consistent controls, this must be done
      // before projection
      joystickInput = joystickInput.times(PoseHelpers.getAlliance() == Alliance.Red ? -1 : 1);

      Translation2d projectedInput = perpendicularUnitVelocity.times(
          joystickInput.dot(perpendicularUnitVelocity) * SwerveConfig.kMaxSpeed.in(MetersPerSecond));

      // Get radial correction velocity
      Translation2d radialCorrectionVelocity = driveSub.getRadialDistanceCorrectionVector(launchingRange);

      // Combine projected joystick input with radial correction
      Translation2d finalVelocity = projectedInput.plus(radialCorrectionVelocity);

      // Limit max speed (avoid division by zero when stationary)
      double velocityNorm = finalVelocity.getNorm();
      if (velocityNorm > SwerveConfig.kMaxSpeed.in(MetersPerSecond)) {
        finalVelocity = finalVelocity.div(velocityNorm).times(SwerveConfig.kMaxSpeed.in(MetersPerSecond));
      }

      // Get current velocity
      ChassisSpeeds currentChassisSpeeds = driveSub.getChassisSpeedsFieldRelative();
      Translation2d velocity = new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
          currentChassisSpeeds.vyMetersPerSecond);

      // Predict future pose 10 command cycles ahead with 20 substeps
      futurePose = futurePoseWithSubSteps(atLimit ? finalVelocity : velocity, 0.20, 20);

      speeds = new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), omega.in(RadiansPerSecond));
    } else {
      LinearVelocity xVel = SwerveConfig.kMaxSpeed.times(xSupplier.get());
      LinearVelocity yVel = SwerveConfig.kMaxSpeed.times(ySupplier.get());

      speeds = new ChassisSpeeds(xVel, yVel, omega);

      // Predict future pose 10 command cycles ahead
      futurePose = driveSub.getPose().plus(
          new Transform2d(
              xVel.times(Seconds.of(0.20)),
              yVel.times(Seconds.of(0.20)),
              new Rotation2d(omega.times(Seconds.of(0.20)))));
    }

    // Only drive if still in launching zone
    if (driveSub.getPoseZone(futurePose) == FieldZones.Launch) {
      driveSub.runVelocity(speeds, true, !shouldLockRangeSupplier.getAsBoolean());
      atLimit = false;
    } else {
      driveSub.runVelocity(new ChassisSpeeds(MetersPerSecond.zero(),
          MetersPerSecond.zero(), omega), true, !shouldLockRangeSupplier.getAsBoolean());
      atLimit = true;
    }

    // Perform logging
    Logger.recordOutput("Drivetrain/DriveTrackHub/DesiredHeading", desiredHeading);
    Logger.recordOutput("Drivetrain/DriveTrackHub/HubPoseBotRelative", toHub);
    Logger.recordOutput("Drivetrain/LaunchingRange", launchingRange);
    Logger.recordOutput("Drivetrain/IsLockedAtRange", shouldLockRangeSupplier.getAsBoolean());
    Logger.recordOutput("Drivetrain/DriveTrackHub/NextPose", futurePose);
    Logger.recordOutput("Drivetrain/DriveTrackHub/AtLimit", atLimit);
    Logger.recordOutput("Drivetrain/DriveTrackHub/DesiredSpeeds",
        speeds);

  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    // Only track hub when in accepted launching zone
    return !(driveSub.getCurrentBotZone() == FieldZones.Launch);
  }

  /**
   * Predicts what the next Pose2d of the bot will be if the current velocity is
   * maintained for a duration of dt, uses substeps to keep the velocity aligned
   * with the
   * curve. This method will only predict the translation, not the rotation
   * 
   * @param velocity The current velocity, field relative
   * @param dt       The total timestep
   * @param numSteps The number of calculation steps
   * @return The predicted Pose2d
   */
  private Pose2d futurePoseWithSubSteps(Translation2d velocity, double dt, int numSteps) {
    Pose2d currentPose = driveSub.getPose();
    Translation2d position = currentPose.getTranslation();

    // Initial hub-relative vector
    Translation2d toHub = driveSub.getHubTranslation3dBotRelative().toTranslation2d();
    double radius = toHub.getNorm();
    if (radius < 1e-6) {
      return currentPose;
    }

    Translation2d hubField = toHub.plus(position);

    // Initial unit vectors
    Translation2d radialUnit = toHub.div(radius);
    Translation2d tangentUnit = radialUnit.rotateBy(Rotation2d.fromRadians(Math.PI / 2));

    // Preserve scalar components
    double radialSpeed = velocity.dot(radialUnit);
    double tangentialSpeed = velocity.dot(tangentUnit);

    double dtSubStep = dt / numSteps;

    for (int i = 0; i < numSteps; i++) {
      // Recompute hub-relative vector from predicted pose
      Translation2d toHubField = hubField.minus(position);
      double r = toHubField.getNorm();
      if (r < 1e-6)
        break;

      Translation2d radial = toHubField.div(r);
      Translation2d tangent = radial.rotateBy(Rotation2d.fromRadians(Math.PI / 2));

      Translation2d stepVelocity = radial.times(radialSpeed).plus(tangent.times(tangentialSpeed));

      position = position.plus(stepVelocity.times(dtSubStep));
    }

    return new Pose2d(position, currentPose.getRotation());
  }
}