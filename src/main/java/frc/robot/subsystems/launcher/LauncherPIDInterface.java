package frc.robot.subsystems.launcher;

// The purpose of this interface is to maintain flexibility in terms of motor choice. 
// As proposed by Wilson, this should be able to let any type motor use this.

public interface LauncherPIDInterface {
  void setReferenceVelocity(double rpm);

  double getVelocity();

  void stop();

}