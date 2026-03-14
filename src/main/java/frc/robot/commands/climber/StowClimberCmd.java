package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Homing command: Crawls down at open-loop percent output until the bottom
 * limit switch trips, then zeroes the encoder.
 *
 * <p>
 * Run this <em>once at match start</em> before any PID commands so the
 * encoder has a reliable zero reference. After homing, use
 * {@link ClimbDownCmd} (PID) for all subsequent stowing.
 *
 */
public class StowClimberCmd extends Command {

  private final ClimberSubsystem climber;

  public StowClimberCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/StowClimberCmd/Status", "Running");
  }

  @Override
  public void execute() {
    climber.setPercentOutput(ClimberConstants.kStowCrawlSpeed);

    Logger.recordOutput("Commands/StowClimberCmd/LeftPositionInches",
        climber.getLeftPosition().in(Inches));
    Logger.recordOutput("Commands/StowClimberCmd/RightPositionInches",
        climber.getRightPosition().in(Inches));
    Logger.recordOutput("Commands/StowClimberCmd/LeftVelocityInchesPerSec",
        climber.getLeftVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/StowClimberCmd/RightVelocityInchesPerSec",
        climber.getRightVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/StowClimberCmd/LeftAtBottom", climber.isLeftAtBottom());
    Logger.recordOutput("Commands/StowClimberCmd/RightAtBottom", climber.isRightAtBottom());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    if (!interrupted && climber.areBothAtBottom()) {
      climber.resetLeftEncoder();
      climber.resetRightEncoder();
      Logger.recordOutput("Commands/StowClimberCmd/EncoderZeroed", true);
    }
    Logger.recordOutput("Commands/StowClimberCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.areBothAtBottom();
  }
}