package frc.robot.subsystems.launcherAndIntake;

import edu.wpi.first.units.measure.AngularVelocity;

public interface LauncherAndIntakeMotorInterface {

  void setReferenceVelocity(AngularVelocity velocity);

  void setPercent(double percent);

  AngularVelocity getVelocity();

  void stop();

  double getAppliedOutput();

  double getCurrent();
}