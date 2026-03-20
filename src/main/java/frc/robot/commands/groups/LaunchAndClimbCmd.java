// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndClimbCmd extends SequentialCommandGroup {
  /**
   * Creates a command that launches for a set amount of time
   * ({@link LauncherAndIntakeConstants#kAutoLaunchTime}), and then climbs.
   * 
   * @param drivetrain        Drivetrain subsystem
   * @param indexer           Indexer subsystem
   * @param launcherAndIntake Launcher/Intake subsystem
   * @param climber           Climber subsystem
   * @param climbSide         Tower side to climb
   * 
   * @see {@link DriveAndLaunchCmd},
   *      {@link LauncherAndIntakeConstants#kAutoLaunchTime}
   */
  public LaunchAndClimbCmd(DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
      LauncherAndIntakeSubsystem launcherAndIntake, ClimberSubsystem climber, TowerSide climbSide) {
    addCommands(
        new ParallelCommandGroup(
            new XLockAndLaunchCmd(
                drivetrain,
                indexer,
                launcherAndIntake).withDeadline(
                    Commands.waitUntil(LaunchHelpers::willHitHub)
                        .andThen(Commands.waitTime(LauncherAndIntakeConstants.kAutoLaunchTime))),
            new ClimbDownCmd(climber)),
        new DriveAndClimbCmd(drivetrain, climber, climbSide));
  }
}
