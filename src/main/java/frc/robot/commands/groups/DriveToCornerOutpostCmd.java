// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToCornerOutpostCmd extends SequentialCommandGroup {
  /**
   * Creates a command that drives to the outpost corner and waits until a
   * specific
   * amount of time has passed in auto ({@link AutoConstants#kDefaultAutoDelay}
   * before crossing to the neutral zone.
   * 
   * @param driveSub Drivetrain subsystem
   * 
   * @see AutoConstants#kDefaultAutoDelay Can be modified in Elastic
   */
  public DriveToCornerOutpostCmd(DrivetrainSubsystem driveSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final Command driveToCornerCmd = AutoBuilder.pathfindToPoseFlipped(AutoConstants.outpostCorner,
        AutoConstants.L1ClimbConstraints);

    final Command waitInCornerCmd = Commands.waitUntil(
        () -> 20 - SmartDashboard.getNumber("Delayed Crossing Time (Auto)",
            AutoConstants.kDefaultAutoDelay.in(Seconds)) >= DriverStation
                .getMatchTime());

    final Command driveToNeutralZoneCmd = new DeferredCommand(() -> PathGenerator.driveToNeutralZoneTrenchAuto(),
        Set.of(driveSub));

    addCommands(
        driveToCornerCmd,
        waitInCornerCmd,
        driveToNeutralZoneCmd);
  }
}
