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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.SwerveConfig;

/* 
 * Command for driving while tracking the hub. When lockRangeSupplier is ture, the bot will maintain a constant distance from the hub
 * 
 * Note on coordinate system: "bot-relative" in this command means that the vectors (like toHub)
 * are still in blue field coordinate system, just with the origin at the robot's position.
*/
public class DriveTrackHubCmd extends Command {

  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private Distance launchingRange;
  private final boolean leadShots;
  private final BooleanSupplier lockRangeSupplier;
  private final BooleanSupplier updateLaunchRangeSupplier;
  private boolean atLimit;

  /**
   * Create a new DriveTrackHubCmd
   * 
   * @param driveSub          The drivetrain subsystem
   * @param xSupplier         Supplier for X-axis input (-1 to 1, field-relative)
   * @param ySupplier         Supplier for Y-axis input (-1 to 1, field-relative)
   * @param lockRangeSupplier Supplier to lock range at current distance from hub
   * @param leadShots         Whether to lead shots based on current velocity
   */
  public DriveTrackHubCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      BooleanSupplier lockRangeSupplier,
      boolean leadShots) {

    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.lockRangeSupplier = lockRangeSupplier;
    this.updateLaunchRangeSupplier = new BooleanSupplier() {
      // Rising edges of lockRangeSupplier
      private boolean prevValue = lockRangeSupplier.getAsBoolean();

      @Override
      public boolean getAsBoolean() {
        boolean currValue = lockRangeSupplier.getAsBoolean();
        boolean risingEdge = currValue && !prevValue;
        prevValue = currValue;

        return risingEdge;
      }
    };
    updateLaunchRange();
    this.leadShots = leadShots;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    driveSub.resetControllers();

    Logger.recordOutput("Drivetrain/LockedAtLaunchingRange", lockRangeSupplier.getAsBoolean());
  }

  @Override
  public void execute() {

    if (updateLaunchRangeSupplier.getAsBoolean()) {
      updateLaunchRange();
    }

    Translation2d toHub = driveSub.getHubTranslation2dBotRelative();

    // Handle edge case where robot is exactly at the hub
    double toHubNorm = toHub.getNorm();
    if (toHubNorm < 1e-6) {
      driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
      return;
    }

    // Get current velocity
    ChassisSpeeds currentChassisSpeeds = driveSub.getChassisSpeedsRobotRelative();
    // Convert to field-relative speeds
    currentChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds,
        driveSub.getPose().getRotation());
    Translation2d velocity = new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond);

    // Get heading correction to face the hub
    Translation2d targetBotRelative = toHub;
    if (leadShots) {
      targetBotRelative = targetBotRelative.minus(velocity
          .times(LaunchHelpers.calculateBallAirTime(FieldConstants.kHubHeight,
              Meters.of(toHubNorm)).in(Seconds)));
    }

    Rotation2d desiredHeading = targetBotRelative.getAngle();
    double omega = driveSub.getHeadingCorrectionOmega(desiredHeading).in(RadiansPerSecond);

    Translation2d finalVelocity;
    if (lockRangeSupplier.getAsBoolean()) {
      // Calculate the vector perpendicular to the vector from the robot to the hub,
      // and project the joystick input onto that vector
      Translation2d perpendicularUnitVelocity = toHub.rotateBy(new Rotation2d(Math.PI / 2.0));
      perpendicularUnitVelocity = perpendicularUnitVelocity.div(toHubNorm);

      Translation2d joystickInput = new Translation2d(xSupplier.get(), ySupplier.get());

      // Invert for red alliance to maintain consistent controls, this must be done
      // before projection
      joystickInput = joystickInput.times(PoseHelpers.getAlliance() == Alliance.Red ? -1 : 1);

      Translation2d projectedInput = perpendicularUnitVelocity.times(
          joystickInput.dot(perpendicularUnitVelocity) * SwerveConfig.kMaxSpeed.in(MetersPerSecond));

      // Get radial correction velocity
      Translation2d radialCorrectionVelocity = driveSub.getRadialDistanceCorrectionVector(launchingRange);

      // Combine projected joystick input with radial correction
      finalVelocity = projectedInput.plus(radialCorrectionVelocity);

      // Limit max speed (avoid division by zero when stationary)
      double velocityNorm = finalVelocity.getNorm();
      if (velocityNorm > SwerveConfig.kMaxSpeed.in(MetersPerSecond)) {
        finalVelocity = finalVelocity.div(velocityNorm).times(SwerveConfig.kMaxSpeed.in(MetersPerSecond));
      }

      Logger.recordOutput("Drivetrain/DriveTrackHub/RadialCorrection",
          new ChassisSpeeds(radialCorrectionVelocity.getX(), radialCorrectionVelocity.getY(), 0));
      Logger.recordOutput("Drivetrain/DriveTrackHub/ProjectedInput",
          new ChassisSpeeds(projectedInput.getX(), projectedInput.getY(), 0));
      Logger.recordOutput("Drivetrain/DriveTrackHub/PerpendicularUnitVelocity",
          new ChassisSpeeds(perpendicularUnitVelocity.getX(), perpendicularUnitVelocity.getY(), 0));
    } else {
      double xVel = SwerveConfig.kMaxSpeed.times(xSupplier.get()).in(MetersPerSecond);
      double yVel = SwerveConfig.kMaxSpeed.times(ySupplier.get()).in(MetersPerSecond);
      finalVelocity = new Translation2d(xVel, yVel);
    }

    // Predict future pose 6 command cycles ahead with 10 substeps
    Pose2d futurePose = futurePoseWithSubSteps(atLimit ? finalVelocity : velocity, 0.12, 20);

    // Only drive if still in launching zone
    if (driveSub.getPoseZone(futurePose) == FieldZones.Launch) {
      driveSub.runVelocity(new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), omega), true, false);
      atLimit = false;
    } else {
      driveSub.runVelocity(new ChassisSpeeds(0, 0, omega), true, false);
      atLimit = true;
    }

    // Perform logging
    Logger.recordOutput("Drivetrain/LockedAtLaunchingRange", lockRangeSupplier.getAsBoolean());
    Logger.recordOutput("Drivetrain/DriveTrackHub/RadialError",
        Math.abs(toHub.getNorm() - launchingRange.in(Meters)));
    Logger.recordOutput("Drivetrain/DriveTrackHub/DesiredHeading", desiredHeading);
    Logger.recordOutput("Drivetrain/DriveTrackHub/HubPoseBotRelative", toHub);
    Logger.recordOutput("Drivetrain/DriveTrackHub/LeadCorrection", targetBotRelative.minus(toHub));
    Logger.recordOutput("Drivetrain/DriveTrackHub/TargetBotRelative", targetBotRelative);
    Logger.recordOutput("Drivetrain/DriveTrackHub/NextPose", futurePose);
    Logger.recordOutput("Drivetrain/DriveTrackHub/AtLimit", atLimit);
    Logger.recordOutput("Drivetrain/DriveTrackHub/FinalVelocity",
        new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), 0));

  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));

    // Log that the command is no longer active
    Logger.recordOutput("Drivetrain/LockedAtLaunchingRange", false);
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
    Translation2d toHub = driveSub.getHubTranslation2dBotRelative();
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

  private void updateLaunchRange() {
    launchingRange = Meters.of(driveSub.getHubTranslation2dBotRelative().getNorm());
  }
}
