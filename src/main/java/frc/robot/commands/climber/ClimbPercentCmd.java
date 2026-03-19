package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * Drives both arms at a percent output supplied by the caller.
 * Positive values raise the arms; negative values lower them.
 *
 * <p>
 * The subsystem's beam-break guard still applies -- if an arm is at the
 * bottom and a negative percent is commanded, that arm will not move.
 */
public class ClimbPercentCmd extends Command {

  private final ClimberSubsystem climber;
  private final DoubleSupplier percent;

  /**
   * Constructs a {@code ClimbPercentCmd}.
   *
   * @param climber the climber subsystem
   * @param percent supplier returning the desired output fraction in
   *                {@code [-1, 1]}
   */
  public ClimbPercentCmd(ClimberSubsystem climber, DoubleSupplier percent) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double output = percent.getAsDouble();
    climber.setPercentOutput(output);
    Logger.recordOutput("Commands/ClimbPercentCmd/PercentOutput", output);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}