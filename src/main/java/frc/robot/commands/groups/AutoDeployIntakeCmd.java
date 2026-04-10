// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.PoseHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeployIntakeCmd extends SequentialCommandGroup {
  /** Creates a new AutoDeployIntakeCmd. */
  public AutoDeployIntakeCmd(DrivetrainSubsystem driveSub) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // 0, 1, 0
        new DriveLockedHeadingCmd(driveSub,
            () ->
            // (PoseHelpers.getAlliance() == Alliance.Blue ? 1.0 :
            -1.0,
            () -> 0.0,
            PoseHelpers.getAlliance() == Alliance.Blue ? new Rotation2d(0) : new Rotation2d(Math.PI))
            .withTimeout(AutoConstants.kIntakeDeployDriveTimeSeconds),
        new DriveLockedHeadingCmd(driveSub,
            () -> 0.0,
            () -> 0.0,
            PoseHelpers.getAlliance() == Alliance.Blue ? new Rotation2d(0) : new Rotation2d(Math.PI))
            .withTimeout(AutoConstants.kIntakeDeployStopTimeSeconds));
  }
}
