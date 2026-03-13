package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Runs the climber to {@link ClimberConstants#kClimbPosition} for up to 3
 * seconds,
 * or until the setpoint is reached — whichever comes first. Then stops.
 *
 * Binding:
 * {@code operatorController.b().onTrue(new TimedAutoClimbCmd(climberSub));}
 */
public class TimedAutoClimbCmd extends Command {

  private final ClimberSubsystem climber;
  private final Timer timer = new Timer();

  public TimedAutoClimbCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    climber.setTargetPosition(ClimberConstants.kClimbPosition);

    Logger.recordOutput("TimedAutoClimbCmd/Status", "Running");
    Logger.recordOutput("TimedAutoClimbCmd/TargetInches", ClimberConstants.kClimbPosition.in(Inches));
    Logger.recordOutput("TimedAutoClimbCmd/TimeoutSecs", ClimberConstants.kClimbDuration.in(Seconds));

  }

  @Override
  public void execute() {
    Logger.recordOutput("TimedAutoClimbCmd/PositionInches", climber.getPosition().in(Inches));
    Logger.recordOutput("TimedAutoClimbCmd/ElapsedSecs", timer.get());
    Logger.recordOutput("TimedAutoClimbCmd/AtSetpoint", climber.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("TimedAutoClimbCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(ClimberConstants.kClimbDuration.in(Seconds)) || climber.atSetpoint();
  }
}