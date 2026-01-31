package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

/**
 * ADXRS450-based Gyro implementation.
 */
public class GyroADXRS450 implements Gyro {
  private final ADXRS450_Gyro gyro;
  // A simple offset (degrees) to allow setAngle without modifying the hardware
  private double offsetDeg = 0.0;

  public GyroADXRS450() {
    this(SPI.Port.kOnboardCS0);
  }

  public GyroADXRS450(SPI.Port port) {
    gyro = new ADXRS450_Gyro(port);
  }

  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngleDeg());
  }

  @Override
  public double getRate() {
    return -gyro.getRate(); // degrees per second
  }

  @Override
  public void calibrate() {
    gyro.calibrate();
    // after calibrating, keep offset so current reported angle remains consistent
    offsetDeg = 0.0;
  }

  @Override
  public void setAngle(Rotation2d angle) {
    // Make getRotation2d() return the requested angle by adjusting offset
    double desiredDeg = angle.getDegrees();
    double rawDeg = -gyro.getAngle();
    offsetDeg = desiredDeg - rawDeg;
  }

  @Override
  public boolean isConnected() {
    return gyro.isConnected();
  }

  /** Returns the current angle in degrees including offset. */
  private double getAngleDeg() {
    return -gyro.getAngle() + offsetDeg;
  }

  /** Reset hardware angle to zero and clear offset. */
  public void reset() {
    gyro.reset();
    offsetDeg = 0.0;
  }
}