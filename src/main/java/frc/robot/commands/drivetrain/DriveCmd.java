// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCmd extends Command {
  /** Creates a new DriveCmd. */
  private DrivetrainSubsystem driveSub;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> omegaSupplier;
  public boolean gyroDisconnected;

  public DriveCmd(DrivetrainSubsystem driveSub, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    // Declare subsystem dependencies
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity xVel = DriveConstants.kMaxSpeed.times(xSupplier.get());
    LinearVelocity yVel = DriveConstants.kMaxSpeed.times(ySupplier.get());
    AngularVelocity omega = DriveConstants.kMaxAngularSpeed.times(omegaSupplier.get());
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
