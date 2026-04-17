// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GameModel;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndDelayedNeutralZoneBumpCmd extends SequentialCommandGroup {
  /**
   * Create a command that launches and drives to the neutral zone after a
   * specific amount of time has passed in auto
   * ({@link AutoConstants#kDefaultAutoDelay})
   * 
   * @param driveSub             Drivetrain subsystem
   * @param indexerSub           Indexer subsystem
   * @param launcherAndIntakeSub Launcher and intake subsystem
   * @param climberSub           Climber subsystem
   * 
   * @see AutoConstants#kDefaultAutoDelay Can be modified in Elastic
   */
  public LaunchAndDelayedNeutralZoneBumpCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub,
      ClimberSubsystem climberSub) {

    final Command launchWaitCmd = Commands.waitUntil(
        () -> 20 - SmartDashboard.getNumber("Delayed Crossing Time (Auto)",
            AutoConstants.kDefaultAutoDelay.in(Seconds)) >= DriverStation
                .getMatchTime());

    final Command launchCmd = new ParallelCommandGroup(
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub).withDeadline(
                Commands.waitUntil(LaunchHelpers::willHitHub)
                    .andThen(launchWaitCmd)),
        new ClimbDownCmd(climberSub));

    final Command driveToNeutralZoneCmd = new DeferredCommand(() -> PathGenerator.driveToNeutralZoneBumpAuto(),
        Set.of(driveSub));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        launchCmd,
        driveToNeutralZoneCmd);
  }
}
