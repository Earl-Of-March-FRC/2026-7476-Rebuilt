package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.AngularVelocity;

public interface LauncherPIDInterface {

  /**
   * Sets the closed-loop velocity setpoint.
   *
   * @param velocity Target velocity as an AngularVelocity (e.g. RPM.of(3000)).
   */
  void setReferenceVelocity(AngularVelocity velocity);

  /**
   * @return Current measured velocity as an AngularVelocity.
   */
  AngularVelocity getVelocity();

  void stop();
}