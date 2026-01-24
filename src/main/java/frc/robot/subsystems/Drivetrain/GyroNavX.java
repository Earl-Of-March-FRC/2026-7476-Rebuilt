// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroNavX implements Gyro {
  // Off Season Swerve
  // private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // SpongeBot
  private final AHRS gyro = new AHRS(NavXComType.kUSB1);

  public GyroNavX() {

  }

  /**
   * {@inheritDoc}
   */
  @Override
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public double getRate() {
    return gyro.getRate();
  }

  /**
   * {@inheritDoc}
   * 
   * Note that it is not possible to actually calibrate the NavX gyro as it is
   * calibrated on power up. This method will only zero the gyro.
   */
  @Override
  public void calibrate() {
    gyro.zeroYaw();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void setAngle(Rotation2d angle) {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(angle.getDegrees());
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public boolean isConnected() {
    return gyro.isConnected();
  }
}