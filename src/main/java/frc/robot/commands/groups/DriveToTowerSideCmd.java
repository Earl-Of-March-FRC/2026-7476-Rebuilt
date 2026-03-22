// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.util.swerve.SwerveConfig;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTowerSideCmd extends SequentialCommandGroup {
  /** Creates a new DriveToTowerSide. */
  public DriveToTowerSideCmd(DrivetrainSubsystem drivetrain, TowerSide towerSide) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        PathGenerator.driveToTowerSideAuto(towerSide));
    Commands.run(
        () -> drivetrain.runVelocity(
            new ChassisSpeeds(SwerveConfig.kMaxSpeed.times(-0.1), MetersPerSecond.zero(), RadiansPerSecond.zero()),
            true, true, false),
        drivetrain).withTimeout(0.5);
  }
}
