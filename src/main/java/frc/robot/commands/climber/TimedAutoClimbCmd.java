package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * PID drives the climber to {@link ClimberConstants#kClimbPosition}, then
 * holds there for {@link ClimberConstants#kClimbDuration} before stopping.
 *
 * <p>
 * The timer only starts counting once the PID reaches the setpoint —
 * so the robot holds at the top for the full duration regardless of how
 * long the ascent took.
 *
 * <p>
 * Use this as an endgame safety net: one press handles the full sequence.
 *
 * <p>
 * Binding:
 * {@code operatorController.b().onTrue(new TimedAutoClimbCmd(climberSub));}
 */
public class TimedAutoClimbCmd extends Command {

  private final ClimberSubsystem climber;
  private final Timer holdTimer = new Timer();
  private boolean holding = false;

  public TimedAutoClimbCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    holding = false;
    holdTimer.reset();
    climber.setTargetPosition(ClimberConstants.kClimbPosition);

    Logger.recordOutput("TimedAutoClimbCmd/Status", "Ascending");
    Logger.recordOutput("TimedAutoClimbCmd/TargetInches", ClimberConstants.kClimbPosition.in(Inches));
    Logger.recordOutput("TimedAutoClimbCmd/HoldDurationSecs", ClimberConstants.kClimbDuration.in(Seconds));
  }

  @Override
  public void execute() {
    // Once PID settles at the top, start the hold timer
    if (!holding && climber.atSetpoint()) {
      holding = true;
      holdTimer.start();
      Logger.recordOutput("TimedAutoClimbCmd/Status", "Holding");
    }

    Logger.recordOutput("TimedAutoClimbCmd/PositionInches", climber.getPosition().in(Inches));
    Logger.recordOutput("TimedAutoClimbCmd/VelocityInchesPerSec", climber.getVelocity().in(InchesPerSecond));
    Logger.recordOutput("TimedAutoClimbCmd/AtSetpoint", climber.atSetpoint());
    Logger.recordOutput("TimedAutoClimbCmd/HoldElapsedSecs", holdTimer.get());
    Logger.recordOutput("TimedAutoClimbCmd/HoldRemainingSecs",
        Math.max(0.0, ClimberConstants.kClimbDuration.in(Seconds) - holdTimer.get()));
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("TimedAutoClimbCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return holding && holdTimer.hasElapsed(ClimberConstants.kClimbDuration.in(Seconds));
  }
}