package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem; // Direct class import

public class PullClimberCmd extends Command {
  private final ClimberSubsystem climber;
  private final BooleanSupplier isLeftSide;

  public PullClimberCmd(ClimberSubsystem climber, BooleanSupplier isLeftSide) {
    this.climber = climber;
    this.isLeftSide = isLeftSide;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setTargetPosition(ClimberConstants.kClimberRaisePositionInches.in(Inches), isLeftSide.getAsBoolean());
  }

  @Override
  public boolean isFinished() {

    return climber.isClimberAtBottom;

    // // Checking if we are within the tolerance range
    // double errorLeft = Math
    // .abs(climber.getPosition(ClimberSubsystem.ClimbSide.Left) -
    // ClimberConstants.kStowPosition.in(Inches));
    // double errorRight = Math
    // .abs(climber.getPosition(ClimberSubsystem.ClimbSide.Right) -
    // ClimberConstants.kStowPosition.in(Inches));
    // return ((errorLeft < ClimberConstants.kPositionTolerance.in(Inches))
    // && errorRight < ClimberConstants.kPositionTolerance.in(Inches));
  }
}