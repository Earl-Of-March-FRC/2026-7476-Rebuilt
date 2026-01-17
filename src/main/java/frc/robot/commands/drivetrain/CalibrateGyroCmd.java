// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/**
 * The CalibrateCmd class is a command that calibrates the drivetrain's gyro
 * and resets the odometry.
 * 
 * Note: You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class CalibrateGyroCmd extends Command {
  // Reference to the drivetrain subsystem
  private final DrivetrainSubsystem driveSub;

  /**
   * Creates a new CalibrateCmd.
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
   * This method calibrates the gyro and resets the odometry.
   */
  @Override
  public void initialize() {
    Pose2d currentPose = driveSub.getPose();
    Rotation2d newAngle = new Rotation2d(180 * Math.PI / 180);
    driveSub.getGyro().setAngle(newAngle);
    driveSub.resetPose(new Pose2d(currentPose.getTranslation(), newAngle));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No repeated action needed for this command
  }

  /**
   * Called once the command ends or is interrupted.
   * This method is empty because no cleanup is necessary.
   */
  @Override
  public void end(boolean interrupted) {
    // No cleanup necessary
  }

  /**
   * Returns true when the command should end.
   * This command ends immediately after initialization.
   * 
   * @return true to indicate the command is complete.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}