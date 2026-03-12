// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;

public class PassAndIndexCmd extends ParallelCommandGroup {
  /**
   * Creates a new PassAndIndexCmd. This command will run the Launcher at the
   * speed needed to pass to the halfway line of our alliance zone.
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param launchSupplier       A BooleanSupplier that determines whether to
   *                             launch balls or not (controls the indexer wheel)
   */
  public PassAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub,
      BooleanSupplier launchSupplier) {
    addCommands(new PulsingTreadmillCmd(
        indexerSub,
        () -> launchSupplier.getAsBoolean()
            ? -IndexerConstants.kWheelLaunchIndexPercent
            : 0,
        () -> -IndexerConstants.kTreadmillLaunchIndexPercent),
        new LauncherCmd(launcherAndIntakeSub,
            () -> LaunchHelpers.calculatePassRPM()));
  }
}
