// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTowerSide extends SequentialCommandGroup {
  /** Creates a new DriveToTowerSide. */
  public DriveToTowerSide(DrivetrainSubsystem drivetrain, TowerSide towerSide) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        PathGenerator.driveToTowerSideAuto(towerSide));
  }
}
