package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Defines the interface for a <b>single</b> climber arm.
 *
 * <p>
 * Implementations are responsible for driving the arm motor, reading its
 * position and velocity, zeroing its encoder, and advancing any simulation
 * state. The sign convention is: positive percent output raises the arm,
 * negative lowers it.
 */
public interface ClimberArmInterface {

  /**
   * Drives the arm at the requested percent output.
   * Positive values raise the arm; negative values lower it.
   *
   * @param percent output fraction in {@code [-1, 1]}
   */
  void setPercentOutput(double percent);

  /**
   * Commands the arm to a position setpoint via its onboard PID controller.
   *
   * @param position desired arm extension
   */
  void setTargetPosition(Distance position);

  /** Stops the arm immediately by commanding zero percent output. */
  void stop();

  /**
   * Returns the current arm extension.
   *
   * @return arm position
   */
  Distance getPosition();

  /**
   * Returns the current arm velocity.
   * Negative values indicate the arm is moving downward.
   *
   * @return arm velocity
   */
  LinearVelocity getVelocity();

  /**
   * @return The sign of the desired velocity from percent output <b>or</b> PID,
   *         -1 for down, 1 for up, and 0 for no velocity
   */
  double getDesiredVelocitySign();

  /**
   * Returns {@code true} when the arm is within tolerance of the given setpoint.
   *
   * @param setpoint the target distance to compare against
   * @return {@code true} if at the setpoint
   */
  boolean isAtPosition(Distance setpoint);

  /** @return motor applied output as a fraction of bus voltage {@code [-1, 1]} */
  double getAppliedOutput();

  /** @return motor output current in amps */
  double getCurrent();

  /**
   * Zeroes the encoder for this arm.
   * Call this when the arm's bottom limit switch triggers.
   */
  void resetEncoder();

  /**
   * Advances any simulation state by one robot loop cycle.
   * No-op on real hardware.
   */
  default void simulationPeriodic() {
  }
}