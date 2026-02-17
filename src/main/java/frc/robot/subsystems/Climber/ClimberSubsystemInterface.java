package frc.robot.subsystems.Climber;

public interface ClimberSubsystemInterface {
  void setTargetPosition(double inches);

  double getVelocity();

  void stop();
}
