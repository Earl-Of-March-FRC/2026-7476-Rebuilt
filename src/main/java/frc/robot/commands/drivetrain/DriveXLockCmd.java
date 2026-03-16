// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class DriveXLockCmd extends Command {
  DrivetrainSubsystem driveSub;

  /** Creates a new DriveXLockCmd. */
  public DriveXLockCmd(DrivetrainSubsystem driveSub) {
    this.driveSub = driveSub;

    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Drivetrain/xLockEnabled", true);
    driveSub.xLock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.xLock();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Drivetrain/xLockEnabled", false);
    driveSub.runVelocity(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
