package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Drives both arms down to ClimberConstants.kStowPosition via PID.
 * Ends when both bottom limit switches trigger, then zeroes both encoders.
 *
 * Requires the encoders to already be zeroed. Run StowClimberCmd at match
 * start first.
 */
public class ClimbDownCmd extends Command {

  private final ClimberSubsystem climber;

  public ClimbDownCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    // climber.setTargetPosition(ClimberConstants.kStowPosition);
    climber.setPercentOutput(ClimberConstants.kStowCrawlSpeed);

    Logger.recordOutput("Commands/ClimbDownCmd/Status", "Running");
    Logger.recordOutput("Commands/ClimbDownCmd/Mode", "PercentOutput");

  }

  @Override
  public void execute() {
    climber.setPercentOutput(ClimberConstants.kStowCrawlSpeed);

    Logger.recordOutput("Commands/ClimbDownCmd/LeftPositionInches",
        climber.getLeftPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbDownCmd/RightPositionInches",
        climber.getRightPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbDownCmd/LeftVelocityInchesPerSec",
        climber.getLeftVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbDownCmd/RightVelocityInchesPerSec",
        climber.getRightVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbDownCmd/LeftAtBottom", climber.isLeftAtBottom());
    Logger.recordOutput("Commands/ClimbDownCmd/RightAtBottom", climber.isRightAtBottom());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    if (!interrupted && climber.areBothAtBottom()) {
      climber.resetLeftEncoder();
      climber.resetRightEncoder();
    }
    Logger.recordOutput("Commands/ClimbDownCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.areBothAtBottom();
  }
}