// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.commands.OTBIntake.IntakeCmd;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchAndDepotCmd extends SequentialCommandGroup {
  /**
   * Creates a command that launches, intakes from the depot, and launches again
   * 
   * @param driveSub             Drivetrain subsystem
   * @param indexerSub           Indexer subsystem
   * @param launcherAndIntakeSub Launcher/Intake subsystem
   */
  public LaunchAndDepotCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final PathPlannerPath depotPath = AutoConstants.depotClimbPath;

    final Command moveToDepotCmd = PathGenerator.driveToDepotAuto();

    final Command driveThroughDepotCmd = AutoBuilder.followPath(depotPath);

    // TODO to be replaced with over the bumper intake command
    final Command intakeCmd = new PulsingTreadmillCmd(
        indexerSub,
        IndexerConstants.kWheelSpeed,
        IndexerConstants.kTreadmillSpeed)
        .alongWith(new LauncherCmd(launcherAndIntakeSub, LauncherAndIntakeConstants.kIntakeRPMSetpoint));

    final Command driveThroughDepotAndIntakeCmd = new ParallelDeadlineGroup(
        driveThroughDepotCmd,
        intakeCmd);

    addCommands(
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub).withDeadline(
                Commands.waitUntil(LaunchHelpers::willHitHub)
                    .andThen(Commands.waitTime(LauncherAndIntakeConstants.kAutoLaunchTime))),
        moveToDepotCmd,
        driveThroughDepotAndIntakeCmd,
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub).withDeadline(
                Commands.waitUntil(LaunchHelpers::willHitHub)
                    .andThen(Commands.waitTime(LauncherAndIntakeConstants.kAutoLaunchTime))));
  }
}
