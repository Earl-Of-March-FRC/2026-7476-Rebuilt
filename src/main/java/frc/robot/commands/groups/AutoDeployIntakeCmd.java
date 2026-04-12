// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OTBIntakeConstants;
import frc.robot.commands.OTBIntake.IntakeCmd;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;
import frc.robot.util.PoseHelpers;
import frc.robot.util.swerve.SwerveConfig;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeployIntakeCmd extends SequentialCommandGroup {
  /** Creates a new AutoDeployIntakeCmd. */
  public AutoDeployIntakeCmd(DrivetrainSubsystem driveSub, OTBIntakeSubsystem otbIntakeSub) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // 0, 1, 0
        // new DriveLockedHeadingCmd(driveSub,
        // () -> (PoseHelpers.getAlliance() == Alliance.Blue ? 1.0 : -1.0),
        // () -> 0.0,
        // PoseHelpers.getAlliance() == Alliance.Blue ? new Rotation2d(0) : new
        // Rotation2d(Math.PI))
        new DriveCmd(driveSub, () -> -1.0, () -> 0.0, () -> {
          if (PoseHelpers.getAlliance() == Alliance.Blue) {
            return driveSub.getHeadingCorrectionOmega(new Rotation2d()).in(RadiansPerSecond)
                / SwerveConfig.kMaxAngularSpeed.in(RadiansPerSecond);
          } else {
            return driveSub.getHeadingCorrectionOmega(new Rotation2d(Math.PI)).in(RadiansPerSecond)
                / SwerveConfig.kMaxAngularSpeed.in(RadiansPerSecond);
          }
        }).withTimeout(AutoConstants.kIntakeDeployDriveTime),
        new DriveCmd(driveSub, () -> 0.0, () -> 0.0, () -> 0.0).withTimeout(AutoConstants.kIntakeDeployStopTime));
    // new IntakeCmd(otbIntakeSub, OTBIntakeConstants.kIntakeSpeed)
    // .withTimeout(AutoConstants.kIntakeDeployIntakeTime));
    // .withTimeout(AutoConstants.kIntakeDeployDriveTime),
    // Commands.waitTime(AutoConstants.kIntakeDeployStopTime));

    // new DriveLockedHeadingCmd(driveSub,
    // () -> 0.0,
    // () -> 0.0,
    // PoseHelpers.getAlliance() == Alliance.Blue ? new Rotation2d(0) : new
    // Rotation2d(Math.PI))
    // .withTimeout(AutoConstants.kIntakeDeployStopTimeSeconds));
  }
}
