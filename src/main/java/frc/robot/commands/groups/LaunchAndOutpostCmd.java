// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.commands.climber.ClimbToHeightCmd;
import frc.robot.commands.climber.StowClimberCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ClimberArmSide;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndOutpostCmd extends SequentialCommandGroup {
  /**
   * Creates a command that launches, intakes at the outpost for a set amount of
   * time ({@link AutoConstants#kAutoOutpostIntakeTime}), and launches again.
   * 
   * @param driveSub             Drivetrain subsystem
   * @param indexerSub           Indexer subsystem
   * @param launcherAndIntakeSub Launcher/Intake subsystem
   * 
   * @see AutoConstants#kAutoOutpostIntakeTime
   */
  public LaunchAndOutpostCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, ClimberSubsystem climberSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final Command moveToOutpost = AutoBuilder.pathfindToPoseFlipped(AutoConstants.outpostPose,
        AutoConstants.L1ClimbConstraints);

    final Command driveAndClimb = new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Right);

    addCommands(
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub).withDeadline(
                Commands.waitUntil(LaunchHelpers::willHitHub)
                    .andThen(Commands.waitTime(LauncherAndIntakeConstants.kAutoLaunchTime))),
        moveToOutpost,
        Commands.waitTime(AutoConstants.kAutoOutpostIntakeTime),
        new ParallelCommandGroup(
            new XLockAndLaunchCmd(
                driveSub,
                indexerSub,
                launcherAndIntakeSub).withDeadline(
                    Commands.waitUntil(LaunchHelpers::willHitHub)
                        .andThen(Commands.waitTime(LauncherAndIntakeConstants.kAutoLaunchTime))),
            new ClimbDownCmd(climberSub)),
        driveAndClimb);
  }
}
