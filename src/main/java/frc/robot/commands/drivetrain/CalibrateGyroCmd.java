// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * The CalibrateGyroCmd class is a command that zeros the drivetrain's gyro
 * to the current heading while preserving the robot's position on the field.
 * 
 * Note: You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class CalibrateGyroCmd extends InstantCommand {
  // Reference to the drivetrain subsystem
  private final DrivetrainSubsystem driveSub;

  /**
   * Creates a new CalibrateGyroCmd.
   * 
   * @param driveSub The drivetrain subsystem used by this command.
   */
  public CalibrateGyroCmd(DrivetrainSubsystem driveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    addRequirements(driveSub);

  }

  /**
   * Called when the command is initially scheduled.
   * This method zeros the gyro and updates the pose with the current position
   * but zero rotation.
   */
  @Override
  public void initialize() {
    // Get current position (x, y)
    Pose2d currentPose = driveSub.getPose();

    // Zero the gyro
    driveSub.getGyro().calibrate();

    // Reset pose with same translation but zero rotation
    driveSub.resetPose(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
  }
}