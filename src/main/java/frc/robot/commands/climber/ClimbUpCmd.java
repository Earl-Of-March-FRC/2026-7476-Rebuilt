package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Drives the climber to {@link ClimberConstants#kClimbPosition} via PID.
 * Ends as soon as the encoder is within tolerance of the setpoint, then stops.
 * The large gear ratio holds the robot up passively.
 */
public class ClimbUpCmd extends Command {

  private final ClimberSubsystem climber;

  public ClimbUpCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setTargetPosition(ClimberConstants.kClimbPosition);

    Logger.recordOutput("ClimbUpCmd/Status", "Running");
    Logger.recordOutput("ClimbUpCmd/TargetInches", ClimberConstants.kClimbPosition.in(Inches));
  }

  @Override
  public void execute() {
    Logger.recordOutput("ClimbUpCmd/PositionInches", climber.getPosition().in(Inches));
    Logger.recordOutput("ClimbUpCmd/VelocityInchesPerSec", climber.getVelocity().in(InchesPerSecond));
    Logger.recordOutput("ClimbUpCmd/AtSetpoint", climber.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("ClimbUpCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.atSetpoint();
  }
}