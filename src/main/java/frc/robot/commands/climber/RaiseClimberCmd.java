package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.Inches;
import frc.robot.subsystems.Climber.ClimberSubsystem; // Direct class import
import frc.robot.Constants.ClimberConstants;

public class RaiseClimberCmd extends Command {
  private final ClimberSubsystem climber;

  public RaiseClimberCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber); // No casting needed!
  }

  @Override
  public void initialize() {
    climber.setTargetPosition(ClimberConstants.kRaisePosition.in(Inches));
  }

  @Override
  public boolean isFinished() {
    // Checking if we are within the tolerance range
    double errorLeft = Math
        .abs(climber.getPosition(ClimberSubsystem.ClimbSide.Left) - ClimberConstants.kRaisePosition.in(Inches));
    double errorRight = Math
        .abs(climber.getPosition(ClimberSubsystem.ClimbSide.Right) - ClimberConstants.kRaisePosition.in(Inches));
    return ((errorLeft < ClimberConstants.kPositionTolerance.in(Inches))
        && errorRight < ClimberConstants.kPositionTolerance.in(Inches));
  }
}