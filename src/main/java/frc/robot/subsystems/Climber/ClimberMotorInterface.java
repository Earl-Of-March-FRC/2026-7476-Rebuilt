package frc.robot.subsystems.Climber;

public interface ClimberMotorInterface {
  void setTargetPosition(double inches);

  void setPercentOutput(double percent);

  double getVelocity();

  void stop();

  double getPosition();

  double getAppliedOutput();

  double getCurrent();

  default void simulationPeriodic() {
  };
}