// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveTrackHubCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;

public class DriveAndLaunchCmd extends ParallelCommandGroup {

  /**
   * Creates a new DriveAndLaunchCmd. This command will run track the hub, lock
   * the bot at it's current distance from the hub when lockSupplier is true, and
   * index balls to launch when launchSupplier is true.
   * 
   * @param driveSub             The DrivetrainSubsystem to use
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param xSupplier            The xSupplier to use [-1, 1]
   * @param ySupplier            The ySupplier to use [-1, 1]
   * @param launchSupplier       The launchSupplier to use
   * @param lockSupplier         The lockSupplier to use
   * @param leadShots            Whether to lead shots based on current velocity
   */
  public DriveAndLaunchCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      BooleanSupplier launchSupplier, BooleanSupplier lockSupplier, boolean leadShots) {
    addCommands(
        // Use the either command to switch between the two driving commands based on
        // the lockSupplier. If lockSupplier is true, we use the
        // DriveAtLaunchingRangeCmd which will maintain the current distance from the
        // hub

        // We defer the Commands.either(...) block so that the choosen command updates
        // based
        // on the supplier
        Commands.defer(() -> Commands.either(
            // We defer the DriveAtLaunchingRangeCmd so that it updates the distance to the
            // hub when it's choosen
            Commands.defer(() -> new DriveAtLaunchingRangeCmd(
                driveSub,
                xSupplier,
                ySupplier,
                driveSub.getHubDistance(),
                leadShots), Set.of(driveSub)),
            new DriveTrackHubCmd(
                driveSub,
                xSupplier,
                ySupplier,
                leadShots),
            lockSupplier), Set.of(driveSub)),
        new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, leadShots));
  }

  /**
   * Convenience constructor for auto launching when we think the ball will go in
   * 
   * @param driveSub             The DrivetrainSubsystem to use
   * @param indexerSub           The IndexerSubsystem to use
   * @param launcherAndIntakeSub The LauncherAndIntakeSubsystem to use
   * @param xSupplier            The xSupplier to use [-1, 1]
   * @param ySupplier            The ySupplier to use [-1, 1]
   * @param lockSupplier         The lockSupplier to use
   * @param leadShots            Whether to lead shots based on current velocity
   */
  public DriveAndLaunchCmd(DrivetrainSubsystem driveSub, IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      BooleanSupplier lockSupplier, boolean leadShots) {
    addCommands(
        // Use the either command to switch between the two driving commands based on
        // the lockSupplier. If lockSupplier is true, we use the
        // DriveAtLaunchingRangeCmd which will maintain the current distance from the
        // hub

        // We defer the Commands.either(...) block so that the choosen command updates
        // based
        // on the supplier
        Commands.defer(() -> Commands.either(
            // We defer the DriveAtLaunchingRangeCmd so that it updates the distance to the
            // hub when it's choosen
            Commands.defer(() -> new DriveAtLaunchingRangeCmd(
                driveSub,
                xSupplier,
                ySupplier,
                driveSub.getHubDistance(),
                leadShots), Set.of(driveSub)),
            new DriveTrackHubCmd(
                driveSub,
                xSupplier,
                ySupplier,
                leadShots),
            lockSupplier), Set.of(driveSub)),
        new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, leadShots));
  }

}
