package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ClimberMotorInterface {

  // Commands the motor to a target position via PID.
  void setTargetPosition(Distance position);

  // Commands the motor at open-loop percent output [-1, 1].
  void setPercentOutput(double percent);

  // Stops the motor.
  void stop();

  // Returns the current encoder position.
  Distance getPosition();

  /**
   * Checks if the climbers are within the tolerance
   * 
   * @return boolean
   */
  boolean isAtPosition();

  // Returns the current encoder velocity.
  LinearVelocity getVelocity();

  // Returns the applied output as a fraction of bus voltage [-1, 1].
  double getAppliedOutput();

  // Returns the motor output current in amps.
  double getCurrent();

  // Zeroes the encoder. Called when the bottom limit switch trips so all
  // subsequent position readings are relative to the true home position.
  void resetEncoder();

  // Override to provide simulation behaviour. No-op by default.
  default void simulationPeriodic() {
  }
}