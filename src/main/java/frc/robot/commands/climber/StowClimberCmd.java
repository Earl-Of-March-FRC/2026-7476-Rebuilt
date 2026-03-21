package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ArmSide;

/**
 * Homing command: crawls both arms downward at
 * {@link ClimberConstants#kStowCrawlSpeed} until both beam-break switches
 * trigger, then zeroes both encoders.
 *
 * <p>
 * Run this once at match start before any other climber commands so the
 * encoders have a reliable zero reference. Each arm stops independently as
 * its beam-break triggers; the command waits until both have seated.
 */
public class StowClimberCmd extends Command {

  private final ClimberSubsystem climber;
  private final ArmSide armSide;

  /**
   * Constructs a {@code StowClimberCmd}.
   *
   * @param climber the climber subsystem
   */
  public StowClimberCmd(ClimberSubsystem climber) {
    this(climber, ArmSide.Both);
  }

  /**
   * Constructs a {@code StowClimberCmd}.
   *
   * @param climber the climber subsystem
   * @param armSide side of the arm to stow
   */
  public StowClimberCmd(ClimberSubsystem climber, ArmSide armSide) {
    this.climber = climber;
    this.armSide = armSide;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/StowClimberCmd/Side", armSide.name());
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
    if (!interrupted) {
      climber.resetLeftEncoder();
      climber.resetRightEncoder();
      Logger.recordOutput("Commands/StowClimberCmd/EncoderZeroed", true);
    }
    Logger.recordOutput("Commands/StowClimberCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  /**
   * Ends when both beam-break limit switches have triggered.
   *
   * @return {@code true} when both arms are fully seated
   */
  @Override
  public boolean isFinished() {
    if (armSide == ArmSide.Left) {
      return climber.isLeftAtBottom();
    } else if (armSide == ArmSide.Right) {
      return climber.isRightAtBottom();
    }
    return climber.areBothAtBottom();
  }
}