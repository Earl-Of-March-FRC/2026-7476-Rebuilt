// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.commands.climber.ClimbToHeightCmd;
import frc.robot.commands.climber.StowClimberCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
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
public class OutpostAndNeutralZoneCmd extends SequentialCommandGroup {
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
  public OutpostAndNeutralZoneCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, ClimberSubsystem climberSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final Command moveToOutpostCmd = AutoBuilder.pathfindToPoseFlipped(AutoConstants.outpostPose,
        AutoConstants.L1ClimbConstraints);

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

    final Command driveToNeutralZoneCmd = new DeferredCommand(() -> PathGenerator.driveToNeutralZoneAuto(),
        Set.of(driveSub));

    addCommands(
        moveToOutpostCmd,
        Commands.waitTime(AutoConstants.kAutoOutpostIntakeTime),
        driveToLaunchCmd,
        launchCmd,
        driveToNeutralZoneCmd);
  }
}
