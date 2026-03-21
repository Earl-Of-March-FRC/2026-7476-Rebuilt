package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ArmSide;

/**
 * Drives both arms to a certain height using PID
 *
 * <p>
 * Each arm is tracked independently -- once an arm reaches the target it
 * stops while the other continues.
 */
public class ClimbToHeightCmd extends Command {

  private final ClimberSubsystem climber;
  private final Distance targetPosition;
  private final ArmSide side;

  /**
   * Constructs a {@code ClimbUpCmd}.
   *
   * @param climber        the climber subsystem
   * @param targetPosition the desired arm extension
   */
  public ClimbToHeightCmd(ClimberSubsystem climber, Distance targetPosition) {
    this(climber, targetPosition, ArmSide.Both);
  }

  /**
   * Constructs a {@code ClimbUpCmd}.
   *
   * @param climber        the climber subsystem
   * @param targetPosition the desired arm extension
   * @param side           Arm side to run
   */
  public ClimbToHeightCmd(ClimberSubsystem climber, Distance targetPosition, ArmSide side) {
    this.climber = climber;
    this.targetPosition = targetPosition;
    this.side = side;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/ClimbUpCmd/Side", side.name());
    Logger.recordOutput("Commands/ClimbUpCmd/Status", "Running");
    Logger.recordOutput("Commands/ClimbUpCmd/TargetInches", targetPosition.in(Inches));
  }

  @Override
  public void execute() {
    climber.setTargetPosition(targetPosition, side);

    Logger.recordOutput("Commands/ClimbUpCmd/LeftPositionInches",
        climber.getLeftPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbUpCmd/RightPositionInches",
        climber.getRightPosition().in(Inches));
    Logger.recordOutput("Commands/ClimbUpCmd/LeftVelocityInchesPerSec",
        climber.getLeftVelocity().in(InchesPerSecond));
    Logger.recordOutput("Commands/ClimbUpCmd/RightVelocityInchesPerSec",
        climber.getRightVelocity().in(InchesPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("Commands/ClimbUpCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  /**
   * Ends when both arms have reached the target position.
   *
   * @return {@code true} when both arms are done
   */
  @Override
  public boolean isFinished() {
    if (side == ArmSide.Left) {
      return climber.leftAtSetpoint();
    } else if (side == ArmSide.Right) {
      return climber.rightAtSetpoint();
    }
    return climber.atSetpoint();
  }
}