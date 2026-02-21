package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Common interface for all climber implementations.
 * Extends Subsystem so commands can call addRequirements() without an unsafe
 * cast.
 */
public interface ClimberSubsystemInterface extends Subsystem {
  void setTargetPosition(double inches);

  void setPercentOutput(double percent);

  double getVelocity();

  void stop();
}