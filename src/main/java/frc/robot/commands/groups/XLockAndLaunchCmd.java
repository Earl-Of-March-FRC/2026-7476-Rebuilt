// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveTrackHubCmd;
import frc.robot.commands.drivetrain.DriveXLockCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;

public class XLockAndLaunchCmd extends ParallelDeadlineGroup {

  /**
   * Creates a new XLockAndLaunchCmd. This command will run track the hub, put the
   * bot in xLock once the heading is correct, and
   * index balls to launch when launchSupplier is true.
   * 
   * @param driveSub             The DrivetrainSubsystem to use
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param launchSupplier       The launchSupplier to use
   * @param leadShots            Whether to lead shots based on current velocity
   */
  public XLockAndLaunchCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, BooleanSupplier launchSupplier, boolean leadShots) {

    super(
        Commands.sequence(
            new DriveTrackHubCmd(driveSub, () -> 0.0, () -> 0.0, false, () -> true)
                .until(driveSub::isHeadingControllerAtSetpoint),
            new DriveXLockCmd(driveSub)),
        new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier, leadShots));
  }

  /**
   * Convenience constructor for auto launching when we think the ball will go in
   * 
   * @param driveSub             The DrivetrainSubsystem to use
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   */
  public XLockAndLaunchCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub) {
    super(
        Commands.sequence(
            new DriveTrackHubCmd(driveSub, () -> 0.0, () -> 0.0, false, () -> true)
                .until(driveSub::isHeadingControllerAtSetpoint),
            new DriveXLockCmd(driveSub)),
        new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, false));
  }

}
