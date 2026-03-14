package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ClimberMotorInterface {

  /** Commands the motor to a target position via onboard PID. */
  void setTargetPosition(Distance position);

  /** Commands both motors at open-loop percent output [-1, 1]. */
  void setPercentOutput(double percent);

  /** Stops both motors immediately. */
  void stop();

  /** Returns the average position of both arms. */
  Distance getPosition();

  /** Returns the current position of the left (leader) arm. */
  Distance getLeftPosition();

  /** Returns the current position of the right (follower) arm. */
  Distance getRightPosition();

  /** Returns the average velocity of both arms. */
  LinearVelocity getVelocity();

  /** Returns the current velocity of the left (leader) arm. */
  LinearVelocity getLeftVelocity();

  /** Returns the current velocity of the right (follower) arm. */
  LinearVelocity getRightVelocity();

  /**
   * Returns true when the average position is within tolerance of the setpoint.
   */
  boolean isAtPosition();

  /** Returns the leader applied output as a fraction of bus voltage [-1, 1]. */
  double getAppliedOutput();

  /** Returns the leader output current in amps. */
  double getCurrent();

  /**
   * Zeroes the left (leader) encoder. Called when the left limit switch triggers.
   */
  void resetLeftEncoder();

  /**
   * Zeroes the right (follower) encoder. Called when the right limit switch
   * triggers.
   */
  void resetRightEncoder();

  /** Override to provide simulation behaviour. No-op by default. */
  default void simulationPeriodic() {
  }
}