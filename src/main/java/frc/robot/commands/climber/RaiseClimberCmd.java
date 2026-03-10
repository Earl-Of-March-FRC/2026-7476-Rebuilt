package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class RaiseClimberCmd extends Command {

  private final ClimberSubsystem climber;
  private final Distance target;

  /**
   * @param climber The climber subsystem.
   * @param target  Target height as a Distance (e.g. Inches.of(32)).
   */
  public RaiseClimberCmd(ClimberSubsystem climber, Distance target) {
    this.climber = climber;
    this.target = target;
    addRequirements(climber);
  }

  /** Convenience constructor using a raw inch value. */
  public RaiseClimberCmd(ClimberSubsystem climber, double inches) {
    this(climber, Inches.of(inches));
  }

  @Override
  public void initialize() {
    climber.setTargetPosition(target.in(Inches));
    Logger.recordOutput("RaiseClimberCmd/TargetInches", target.in(Inches));
    Logger.recordOutput("RaiseClimberCmd/Status", "Initialized");
  }

  @Override
  public void execute() {
    Logger.recordOutput("RaiseClimberCmd/MeasuredVelocityInchesPerSec", climber.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    // Hold position on completion; stop only if interrupted
    if (interrupted)
      climber.stop();
    Logger.recordOutput("RaiseClimberCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false; // Let the PID hold position; bind to a button so it ends on release
  }
}