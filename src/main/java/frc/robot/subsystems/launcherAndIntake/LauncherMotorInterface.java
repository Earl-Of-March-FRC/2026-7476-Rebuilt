package frc.robot.subsystems.launcherAndIntake;

import edu.wpi.first.units.measure.AngularVelocity;

public interface LauncherMotorInterface {
  void setReferenceVelocity(AngularVelocity velocity);

  AngularVelocity getVelocity();

  void stop();

  double getAppliedOutput();

  double getCurrent();
}