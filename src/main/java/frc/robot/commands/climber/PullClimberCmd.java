package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ClimbSide;

public class PullClimberCmd extends Command {

  private final ClimberSubsystem climber;
  private final DoubleSupplier speed;
  public final ClimbSide side;

  public PullClimberCmd(ClimberSubsystem climber, DoubleSupplier speed, ClimbSide climbSide) {
    this.climber = climber;
    this.speed = speed;
    this.side = climbSide;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("PullClimberCmd/Status", "Initialized");
    Logger.recordOutput("PullClimberCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    climber.setPercentOutput(speed.getAsDouble(), side);
    Logger.recordOutput("PullClimberCmd/MeasuredVelocityInchesPerSec", climber.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("PullClimberCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}