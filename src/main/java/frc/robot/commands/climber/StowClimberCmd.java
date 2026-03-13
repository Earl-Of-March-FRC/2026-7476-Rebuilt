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
    Logger.recordOutput("StowClimberCmd/Status", "Running");
  }

  @Override
  public void execute() {
    climber.setPercentOutput(ClimberConstants.kStowCrawlSpeed);

    Logger.recordOutput("StowClimberCmd/PositionInches", climber.getPosition().in(Inches));
    Logger.recordOutput("StowClimberCmd/VelocityInchesPerSec", climber.getVelocity().in(InchesPerSecond));
    Logger.recordOutput("StowClimberCmd/LimitSwitchTripped", climber.isAtBottom());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    if (!interrupted) {
      climber.resetEncoder();
      Logger.recordOutput("StowClimberCmd/EncoderZeroed", true);
    }
    Logger.recordOutput("StowClimberCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.isAtBottom();
  }
}