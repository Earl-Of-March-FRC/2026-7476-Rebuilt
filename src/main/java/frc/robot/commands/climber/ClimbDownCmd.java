package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ClimberArmSide;

/**
 * Crawls both arms downward at {@link ClimberConstants#kStowCrawlSpeed} until
 * both beam-break limit switches have triggered.
 *
 * <p>
 * Each arm stops independently the moment its beam-break triggers via the
 * guard inside {@link ClimberSubsystem#setPercentOutput(double)}. This command
 * ends only once both arms are fully seated.
 */
public class ClimbDownCmd extends Command {

  private final ClimberSubsystem climber;
  private final ClimberArmSide side;

  /**
   * Constructs a {@code ClimbDownCmd}.
   *
   * @param climber the climber subsystem
   */
  public ClimbDownCmd(ClimberSubsystem climber) {
    this(climber, ClimberArmSide.Both);
  }

  /**
   * Constructs a {@code ClimbDownCmd}.
   *
   * @param climber the climber subsystem
   * @param side    Arm side
   */
  public ClimbDownCmd(ClimberSubsystem climber, ClimberArmSide side) {
    this.climber = climber;
    this.side = side;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/ClimbDownCmd/Side", side);
    Logger.recordOutput("Commands/ClimbDownCmd/Status", "Running");
  }

  @Override
  public void execute() {
    climber.setPercentOutput(ClimberConstants.kStowCrawlSpeed, side);

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
    if (interrupted) {
      Logger.recordOutput("Commands/ClimbDownCmd/Status", "Interrupted");
      return;
    }
    if (side != ClimberArmSide.Right) {
      climber.resetLeftEncoder();
    }

    if (side != ClimberArmSide.Left) {
      climber.resetRightEncoder();
    }

    Logger.recordOutput("Commands/ClimbDownCmd/Status", "Completed");

  }

  /**
   * Ends when both beam-break limit switches have triggered.
   *
   * @return {@code true} when both arms are fully seated
   */
  @Override
  public boolean isFinished() {
    if (side == ClimberArmSide.Left) {
      return climber.isLeftAtBottom();
    } else if (side == ClimberArmSide.Right) {
      return climber.isRightAtBottom();
    }
    return climber.areBothAtBottom();
  }
}