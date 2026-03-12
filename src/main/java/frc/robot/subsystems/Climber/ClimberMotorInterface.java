package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ClimberMotorInterface {
  void setTargetPosition(double inches);

  void setPercentOutput(double percent);

  LinearVelocity getVelocity();

  void stop();

  Distance getPosition();

  double getAppliedOutput();

  double getCurrent();

  boolean isAtPosition();

  default void simulationPeriodic() {
  };
}