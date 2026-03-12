package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ClimberSide;

public class PullClimberCmd extends Command {

  private final ClimberSubsystem climber;
  private final DoubleSupplier speed;
  public final ClimberSide side;

  public PullClimberCmd(ClimberSubsystem climber, DoubleSupplier speed, ClimberSide climbSide) {
    this.climber = climber;
    this.speed = speed;
    this.side = climbSide;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/PullClimberCmd/Status", "Initialized");
    Logger.recordOutput("Commands/PullClimberCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    climber.setPercentOutput(speed.getAsDouble(), side);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("Commands/PullClimberCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}