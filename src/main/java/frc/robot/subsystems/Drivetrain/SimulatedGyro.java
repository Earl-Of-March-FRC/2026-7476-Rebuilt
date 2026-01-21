package frc.robot.subsystems.Drivetrain;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class SimulatedGyro implements Gyro {
  private final GyroSimulation gyroSimulation;

  public SimulatedGyro(GyroSimulation gyroSim) {
    this.gyroSimulation = gyroSim;
  }

  @Override
  public void calibrate() {
    gyroSimulation.setRotation(new Rotation2d(0));
  }

  @Override
  public void setAngle(Rotation2d angle) {
    calibrate();
    gyroSimulation.setRotation(angle);
  }

  @Override
  public Rotation2d getRotation2d() {
    return gyroSimulation.getGyroReading();
  }

  @Override
  public double getRate() {
    return gyroSimulation.getMeasuredAngularVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public boolean isConnected() {
    return true;
  }
}