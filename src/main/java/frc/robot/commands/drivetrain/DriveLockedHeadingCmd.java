// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.SwerveConfig;

/**
 * Command for restricted swerve driving with locked heading.
 * This command locks the robot's heading to a specific angle (e.g., 45 degrees)
 * while still allowing full X-Y translational movement with the joystick.
 * The robot will automatically rotate to maintain the locked heading.
 * 
 * Locking behavior: Locks to the nearest acute angle (≤90°) relative to the
 * current quadrant based on gyro position.
 */
public class DriveLockedHeadingCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Rotation2d lockedAngle;
  private final LinearVelocity maxSpeed;
  private Rotation2d targetHeading;

  /**
   * Creates a new DriveLockedHeadingCmd.
   * 
   * @param driveSub    The drivetrain subsystem
   * @param xSupplier   Supplier for X-axis input (-1 to 1, field-relative)
   * @param ySupplier   Supplier for Y-axis input (-1 to 1, field-relative)
   * @param lockedAngle The acute angle to lock the robot's heading to (≤90°)
   * @param maxSpeed    Maximum speed in meters per second
   */
  public DriveLockedHeadingCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Rotation2d lockedAngle,
      LinearVelocity maxSpeed) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.lockedAngle = lockedAngle;
    this.maxSpeed = maxSpeed;
    addRequirements(driveSub);
  }

  /**
   * Convenience constructor with default speed limit.
   */
  public DriveLockedHeadingCmd(
      DrivetrainSubsystem driveSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Rotation2d lockedAngle) {
    this(driveSub, xSupplier, ySupplier, lockedAngle, SwerveConfig.kMaxSpeed);
  }

  @Override
  public void initialize() {
    // Calculate and set initial target heading
    updateTargetHeading();
  }

  @Override
  public void execute() {
    // Check if robot has been pushed significantly off target
    double currentAngle = driveSub.getGyro().getRotation2d().getRadians();
    double targetAngle = targetHeading.getRadians();

    // Calculate shortest angular distance (accounting for wraparound)
    double error = currentAngle - targetAngle;
    error = Math.atan2(Math.sin(error), Math.cos(error)); // Normalize to [-π, π]

    // If pushed more than threshold away, recalculate to nearest snap point
    if (Math.abs(error) > Constants.DriveConstants.kRecalibrateThreshold.in(Degrees)) {
      updateTargetHeading();
    }

    // Log that the command is active
    Logger.recordOutput("Drivetrain/RestrictedMode", true);
    Logger.recordOutput("Drivetrain/LockedAngle", targetHeading.getDegrees());
    Logger.recordOutput("Drivetrain/RestrictedMode/HeadingError", Math.toDegrees(error));

    // Get X and Y joystick inputs (full 2D control)
    LinearVelocity xVel = maxSpeed.times(xSupplier.get());
    LinearVelocity yVel = maxSpeed.times(ySupplier.get());

    // Limit max speed again
    Translation2d vel = new Translation2d(xVel.in(MetersPerSecond), yVel.in(MetersPerSecond));

    if (vel.getNorm() > maxSpeed.in(MetersPerSecond)) {
      vel = vel.div(vel.getNorm()).times(maxSpeed.in(MetersPerSecond));
      xVel = MetersPerSecond.of(vel.getX());
      yVel = MetersPerSecond.of(vel.getY());
    }

    // Get rotational velocity to maintain the CALCULATED heading
    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(targetHeading);

    // Apply chassis speeds (field-relative with locked heading)
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));
  }

  /**
   * Calculates and updates the target heading to the nearest acute angle
   * relative to the current quadrant.
   */
  private void updateTargetHeading() {
    double currentAngleRadians = driveSub.getPose().getRotation().getRadians();
    double acuteAngleRadians = lockedAngle.getRadians();

    // Find which 90-degree quadrant we're in
    int quadrant = (int) Math.floor((currentAngleRadians + Math.PI) / (Math.PI / 2));

    // Calculate base angle for the quadrant (0, π/2, π, -π/2)
    double baseAngle = (quadrant * Math.PI / 2) - Math.PI;

    // Apply acute angle: alternate between adding and subtracting based on quadrant
    double targetAngleRadians = baseAngle
        + ((quadrant % 2 == 0) ? acuteAngleRadians : (Math.PI / 2 - acuteAngleRadians));

    // Create the target heading from the calculated angle
    targetHeading = new Rotation2d(targetAngleRadians);

    // Set the target heading for the robot to maintain
    driveSub.setTargetHeading(targetHeading);

    Logger.recordOutput("Drivetrain/RestrictedMode/CurrentAngle", Math.toDegrees(currentAngleRadians));
    Logger.recordOutput("Drivetrain/RestrictedMode/TargetAngle", Math.toDegrees(targetAngleRadians));
    Logger.recordOutput("Drivetrain/RestrictedMode/Quadrant", quadrant);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
    driveSub.clearTargetHeading();

    // Log that the command is no longer active
    Logger.recordOutput("Drivetrain/RestrictedMode", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}