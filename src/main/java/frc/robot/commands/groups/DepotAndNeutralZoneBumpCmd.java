// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.Constants.OTBIntakeConstants;
import frc.robot.commands.OTBIntake.IntakeCmd;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
import frc.robot.commands.indexer.TreadmillOnCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepotAndNeutralZoneBumpCmd extends SequentialCommandGroup {
  /**
   * Creates a command that launches, intakes from the depot, and launches again
   * 
   * @param driveSub             Drivetrain subsystem
   * @param indexerSub           Indexer subsystem
   * @param launcherAndIntakeSub Launcher/Intake subsystem
   */
  public DepotAndNeutralZoneBumpCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      OTBIntakeSubsystem otbIntakeSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, ClimberSubsystem climberSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final Command moveToDepotCmd = PathGenerator.driveToDepotAuto();

    final PathPlannerPath depotPath = AutoConstants.depotPath;
    // final Command driveThroughDepotCmd = AutoBuilder.followPath(depotPath);
    final Command driveThroughDepotCmd = new DriveCmd(driveSub, () -> -0.5, () -> 0.0, () -> 0.0)
        .withTimeout(AutoConstants.kDepotIntakeTime);

    final Command intakeCmd = new TreadmillOnCmd(
        indexerSub,
        () -> 0,
        () -> -IndexerConstants.kTreadmillSpeed)
        .alongWith(new IntakeCmd(otbIntakeSub, () -> OTBIntakeConstants.kIntakeSpeed));

    final Command driveThroughDepotAndIntakeCmd = new ParallelDeadlineGroup(
        driveThroughDepotCmd,
        intakeCmd);

    final Command driveToLaunchCmd = new DeferredCommand(() -> PathGenerator.driveToLaunchPoseAuto(), Set.of(driveSub));

    final Command launchWaitCmd = Commands.defer(
        () -> Commands.waitTime(Seconds.of(
            SmartDashboard.getNumber("32 Fuel Launch Time (Auto)", AutoConstants.kAutoLaunch32Time.in(Seconds)))),
        Set.of());

    final Command launchCmd = new ParallelCommandGroup(
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub).withDeadline(
                Commands.waitUntil(LaunchHelpers::willHitHub)
                    .andThen(launchWaitCmd)),
        new ClimbDownCmd(climberSub));

    final Command driveToNeutralZoneBumpCmd = new DeferredCommand(() -> PathGenerator.driveToNeutralZoneBumpAuto(),
        Set.of(driveSub));

    addCommands(
        moveToDepotCmd,
        driveThroughDepotAndIntakeCmd,
        driveToLaunchCmd,
        launchCmd,
        driveToNeutralZoneBumpCmd);
  }
}
