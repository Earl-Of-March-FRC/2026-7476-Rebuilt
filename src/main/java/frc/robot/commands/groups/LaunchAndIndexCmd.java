// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
import frc.robot.commands.indexer.TreadmillOnCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;

public class LaunchAndIndexCmd extends ParallelCommandGroup {
  /**
   * Creates a new LaunchAndIndexCmd. This command will run the Launcher at the
   * provided speed, and will run the Indexer wheel
   * to feed balls into the Launcher when the launchSupplier is true. Indexer
   * treadmill will run at the same speed regardless of the launchSupplier to
   * ensure that balls are fed into the indexer wheel.
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param launchSupplier       A BooleanSupplier that determines whether to
   *                             launch balls or not (controls the indexer wheel)
   * @param rpmSupplier          The RPM setpoint supplier for the launcher
   */
  public LaunchAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub,
      BooleanSupplier launchSupplier, Supplier<AngularVelocity> rpmSupplier) {
    addCommands(new PulsingTreadmillCmd(
        indexerSub,
        () -> launchSupplier.getAsBoolean()
            ? -IndexerConstants.kWheelLaunchIndexPercent
            : 0,
        () -> -IndexerConstants.kTreadmillLaunchIndexPercent),
        new LauncherCmd(launcherAndIntakeSub,
            rpmSupplier));
  }

  /**
   * Convenience constructor for calculating RPM based on bot distance
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param launchSupplier       A BooleanSupplier that determines whether to
   *                             launch balls or not (controls the indexer wheel)
   * @param leadShots            Whether to lead shots (affects effective target
   *                             distance)
   */
  public LaunchAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub,
      BooleanSupplier launchSupplier,
      boolean leadShots) {
    this(indexerSub, launcherAndIntakeSub, launchSupplier,
        (Supplier<AngularVelocity>) () -> LaunchHelpers.calculateHubSetpoints(leadShots).flywheelSpeed());
  }

  /**
   * Convenience constructor for auto launching when we think the ball will go in
   * 
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param leadShots            Whether to lead shots (affects effective target
   *                             distance)
   */
  public LaunchAndIndexCmd(IndexerSubsystem indexerSub, LauncherAndIntakeSubsystem launcherAndIntakeSub,
      boolean leadShots) {
    this(
        indexerSub,
        launcherAndIntakeSub,
        () -> LaunchHelpers.willHitHub(),
        leadShots);
  }

}
