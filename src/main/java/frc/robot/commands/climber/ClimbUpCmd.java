package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Drives both arms to ClimberConstants.kClimbPosition via PID.
 * Ends when both arms are within tolerance of the setpoint, then stops.
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

    Logger.recordOutput("Commands/ClimbUpCmd/Status", "Running");
    Logger.recordOutput("Commands/ClimbUpCmd/TargetInches",
        ClimberConstants.kClimbPosition.in(Inches));
  }

  @Override
  public void execute() {
    Logger.recordOutput("Commands/ClimbUpCmd/LeftPositionInches",
        climber.getLeftPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbUpCmd/RightPositionInches",
        climber.getRightPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbUpCmd/LeftVelocityInchesPerSec",
        climber.getLeftVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbUpCmd/RightVelocityInchesPerSec",
        climber.getRightVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbUpCmd/AtSetpoint", climber.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("Commands/ClimbUpCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.atSetpoint();
  }
}