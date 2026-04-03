// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.drivetrain.DriveTrackHubCmd;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers;

/**
 * Drives freely while continuously tracking the hub heading and auto-firing
 * whenever the shot prediction says the ball will score.
 *
 * <p>
 * Unlike {@link XLockAndLaunchCmd} (which stops and locks the drivetrain) or
 * {@link DriveAndLaunchCmd} (which optionally locks range), this command never
 * constrains the driver's movement. The robot can drive at full speed in any
 * direction; the heading controller keeps the launcher pointed at the hub, and
 * the indexer feeds a ball the instant {@link LaunchHelpers#willHitHub()}
 * returns {@code true}.
 *
 * <p>
 * Shot leading is always enabled: flywheel speed and aim point are both
 * corrected for the robot's current translational velocity, so balls fired
 * while the robot is moving still arc into the hub.
 *
 * <p>
 * The command ends when the robot leaves the
 * {@link frc.robot.util.swerve.FieldZones#Launch}
 * zone (same termination condition as {@link DriveTrackHubCmd}).
 *
 */
public class LaunchOnTheFlyCmd extends ParallelDeadlineGroup {

  /**
   * Creates a new {@code LaunchOnTheFlyCmd}.
   *
   * @param driveSub             the drivetrain subsystem
   * @param indexerSub           the indexer subsystem
   * @param launcherAndIntakeSub the launcher/intake subsystem
   * @param xSupplier            driver X-axis input supplier, range
   *                             {@code [-1, 1]},
   *                             field-relative
   * @param ySupplier            driver Y-axis input supplier, range
   *                             {@code [-1, 1]},
   *                             field-relative
   */
  public LaunchOnTheFlyCmd(
      DrivetrainSubsystem driveSub,
      IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherAndIntakeSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier) {

    super(
        // Deadline: drive while tracking the hub.
        // lockRange = false -> driver has complete freedom of movement.
        // leadShots = true -> heading + RPM account for robot velocity.
        new DriveTrackHubCmd(
            driveSub,
            xSupplier,
            ySupplier,
            true,
            () -> false),

        // Parallel: spin up the flywheel and feed a ball the instant the
        // shot predictor says it will score. leadShots=true keeps the RPM
        // consistent with what DriveTrackHubCmd is aiming for.
        new LaunchAndIndexCmd(
            indexerSub,
            launcherAndIntakeSub,
            LaunchHelpers::willHitHub,
            true));
  }
}