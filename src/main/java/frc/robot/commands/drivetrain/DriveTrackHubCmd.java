// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.launcher.LaunchHelpers;
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
  private final boolean leadShots;
  private boolean atLimit;

  /**
   * Create a new DriveTrackHubCmd
   * 
   * @param driveSub  The drivetrain subsystem
   * @param xSupplier Supplier for X-axis input (-1 to 1, field-relative)
   * @param ySupplier Supplier for Y-axis input (-1 to 1, field-relative)
   * @param leadShots Whether to lead shots based on current velocity
   */
  public DriveTrackHubCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      boolean leadShots) {

    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.leadShots = leadShots;
    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
    driveSub.resetControllers();
  }

  @Override
  public void execute() {

    Translation2d toHub = driveSub.getHubTranslation2dBotRelative();

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
          .times(LaunchHelpers.calculateBallAirTime(FieldConstants.kHubHeight).in(Seconds)));
    }

    Rotation2d desiredHeading = targetBotRelative.getAngle();
    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(desiredHeading);
    LinearVelocity xVel = SwerveConfig.kMaxSpeed.times(xSupplier.get());
    LinearVelocity yVel = SwerveConfig.kMaxSpeed.times(ySupplier.get());

    // Predict future pose 6 command cycles ahead
    Pose2d futurePose = driveSub.getPose().plus(
        new Transform2d(
            xVel.times(Seconds.of(0.12)),
            yVel.times(Seconds.of(0.12)),
            new Rotation2d(omega.times(Seconds.of(0.12)))));

    // Only drive if still in launching zone
    if (driveSub.getPoseZone(futurePose) == FieldZones.Launch) {
      driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega), true, false);
      atLimit = false;
    } else {
      driveSub.runVelocity(new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.zero(), omega), true, false);
      atLimit = true;
    }

    // Perform logging
    Logger.recordOutput("Drivetrain/DriveTrackHub/DesiredHeading", desiredHeading);
    Logger.recordOutput("Drivetrain/DriveTrackHub/HubPoseBotRelative", toHub);
    Logger.recordOutput("Drivetrain/DriveTrackHub/LeadCorrection", targetBotRelative.minus(toHub));
    Logger.recordOutput("Drivetrain/DriveTrackHub/TargetBotRelative", targetBotRelative);
    Logger.recordOutput("Drivetrain/DriveTrackHub/NextPose", futurePose);
    Logger.recordOutput("Drivetrain/DriveTrackHub/AtLimit", atLimit);
    Logger.recordOutput("Drivetrain/DriveTrackHub/DesiredSpeeds",
        new ChassisSpeeds(xVel, yVel, omega));

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
}