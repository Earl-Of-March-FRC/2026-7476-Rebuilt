// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/* 
 * Command for driving while maintaining a specific distance from the hub.
 * The robot can move tangentially around the hub while automatically correcting
 * its radial distance to stay at the desired shooting range.
 * The robot will also face the hub, optionally leading shots based on current velocity.
 * 
 * Note on coordinate system: "bot-relative" in this command means that the vectors (like toHub)
 * are still in blue field coordinate system, just with the origin at the robot's position.
*/
public class DriveAtShootingRangeCmd extends Command {

  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Distance shootingRange;
  private final boolean leadShots;

  /**
   * Creates a new DriveAtShootingRangeCmd.
   * 
   * @param driveSub      The drivetrain subsystem
   * @param xSupplier     Supplier for X-axis input (-1 to 1, field-relative)
   * @param ySupplier     Supplier for Y-axis input (-1 to 1, field-relative)
   * @param shootingRange The desired shooting range distance from the hub
   * @param leadShots     Whether to lead shots based on current velocity
   */
  public DriveAtShootingRangeCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Distance shootingRange,
      boolean leadShots) {

    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.shootingRange = shootingRange;
    this.leadShots = leadShots;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    driveSub.resetControllers();

    Logger.recordOutput("Drivetrain/LockedAtShootingRange", true);
  }

  @Override
  public void execute() {

    Translation2d toHub = driveSub.getHubTranslation2dBotRelative();

    // calculate the vector prependicular to the vector from the robot to the hub,
    // and project the joystick input onto that vector
    Translation2d perpendicularUnitVelocity = toHub.rotateBy(new Rotation2d(Math.PI / 2.0));
    perpendicularUnitVelocity = perpendicularUnitVelocity.div(perpendicularUnitVelocity.getNorm());

    // Invert for red alliance to maintain consistent controls
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      perpendicularUnitVelocity = perpendicularUnitVelocity.times(-1);
    }

    Translation2d joystickInput = new Translation2d(xSupplier.get(), ySupplier.get());
    Translation2d projectedInput = perpendicularUnitVelocity.times(
        joystickInput.dot(perpendicularUnitVelocity) * DriveConstants.kMaxSpeed.in(MetersPerSecond));

    // Get radial correction velocity
    Translation2d radialCorrectionVelocity = driveSub.getRadialDistanceCorrectionVector(shootingRange);

    // Combine projected joystick input with radial correction
    Translation2d finalVelocity = projectedInput.plus(radialCorrectionVelocity);

    // Limit max speed
    finalVelocity = finalVelocity.div(finalVelocity.getNorm()).times(
        Math.min(finalVelocity.getNorm(), DriveConstants.kMaxSpeed.in(MetersPerSecond)));

    // Get heading correction to face the hub
    Translation2d targetBotRelative = toHub;
    if (leadShots) {
      ChassisSpeeds currentChassisSpeeds = driveSub.getChassisSpeedsRobotRelative();
      // Convert to field-relative speeds
      currentChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds,
          driveSub.getPose().getRotation());
      Translation2d velocity = new Translation2d(currentChassisSpeeds.vxMetersPerSecond,
          currentChassisSpeeds.vyMetersPerSecond);
      targetBotRelative = targetBotRelative.minus(velocity.times(LauncherConstants.kBallAirTime.in(Seconds)));
    }

    Rotation2d desiredHeading = targetBotRelative.getAngle();
    double omega = driveSub.getHeadingCorrectionOmega(desiredHeading).in(RadiansPerSecond);

    // Predict future pose 5 cycles ahead (20ms timestep)
    Pose2d botPose = driveSub.getPose();
    Pose2d futurePose = new Pose2d(
        botPose.getTranslation().plus(finalVelocity.times(0.02 * 5)),
        botPose.getRotation());

    // Only drive if still in shooting zone
    if (driveSub.isInShootingZone(futurePose)) {
      driveSub.runVelocity(new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), omega), true, false);
    } else {
      driveSub.runVelocity(new ChassisSpeeds(0, 0, omega), true, false);
    }

    // Perform logging
    Logger.recordOutput("Drivetrain/LockedAtShootingRange", true);
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/RadialError",
        Math.abs(toHub.getNorm() - shootingRange.in(Meters)));
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/DesiredHeading", desiredHeading);
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/HubPoseBotRelative", toHub);
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/LeadCorrection", targetBotRelative.minus(toHub));
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/TargetBotRelative", targetBotRelative);
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/RadialCorrection",
        new ChassisSpeeds(radialCorrectionVelocity.getX(), radialCorrectionVelocity.getY(), 0));
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/ProjectedInput",
        new ChassisSpeeds(projectedInput.getX(), projectedInput.getY(), 0));
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/FinalVelocity",
        new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), 0));
    Logger.recordOutput("Drivetrain/DriveAtShootingRange/PerpendicularUnitVelocity",
        new ChassisSpeeds(perpendicularUnitVelocity.getX(), perpendicularUnitVelocity.getY(), 0));

  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));

    // Log that the command is no longer active
    Logger.recordOutput("Drivetrain/LockedAtShootingRange", false);
  }

  @Override
  public boolean isFinished() {
    // Only track hub when in accepted shooting zone
    return !driveSub.isBotInShootingZone();
  }
}
