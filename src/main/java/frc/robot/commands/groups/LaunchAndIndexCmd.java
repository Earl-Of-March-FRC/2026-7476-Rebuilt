// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.indexer.IndexerCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;

public class LaunchAndIndexCmd extends ParallelCommandGroup {
  /**
   * Creates a new LaunchAndIndexCmd. This command will run the Launcher at the
   * speed needed to score from the current range, and will run the Indexer wheel
   * to feed balls into the Launcher when the launchSupplier is true. Indexer
   * treadmill will run at the same speed regardless of the launchSupplier to
   * ensure that balls are fed into the indexer wheel.
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param launchSupplier       A BooleanSupplier that determines whether to
   *                             laucnh balls or not (controls the indexer wheel)
   */
  public LaunchAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub,
      BooleanSupplier launchSupplier) {
    addCommands(new IndexerCmd(
        indexerSub,
        () -> launchSupplier.getAsBoolean()
            ? IndexerConstants.kWheelLaunchIndexPercent
            : 0,
        () -> IndexerConstants.kTreadmillLaunchIndexPercent),
        new LauncherCmd(launcherAndIntakeSub,
            () -> LaunchHelpers.calculateWheelRPM()));
  }

  /**
   * Convenience constructor for auto launching when we think the ball will go in
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   */
  public LaunchAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub) {
    this(
        indexerSub,
        launcherAndIntakeSub,
        () -> LaunchHelpers.willHitTarget(
            PoseHelpers.getAllianceHubtTranslation3d(),
            FieldConstants.kHubInsideWidth));
  }

}
