package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Drives the climber down to {@link ClimberConstants#kStowPosition} via PID.
 * Ends when the bottom limit switch trips, then zeroes the encoder.
 *
 * <p>
 * Requires the encoder to already be zeroed (run {@link StowClimberCmd}
 * at match start first).
 *
 */
public class ClimbDownCmd extends Command {

  private final ClimberSubsystem climber;

  public ClimbDownCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setTargetPosition(ClimberConstants.kStowPosition);

    Logger.recordOutput("Commands/ClimbDownCmd/Status", "Running");
    Logger.recordOutput("Commands/ClimbDownCmd/TargetInches", ClimberConstants.kStowPosition.in(Inches));
  }

  @Override
  public void execute() {
    Logger.recordOutput("Commands/ClimbDownCmd/PositionInches", climber.getPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbDownCmd/VelocityInchesPerSec", climber.getVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbDownCmd/LimitSwitchTripped", climber.isAtBottom());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    if (!interrupted && climber.isAtBottom()) {
      climber.resetEncoder();
    }
    Logger.recordOutput("Commands/ClimbDownCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return climber.isAtBottom();
  }
}