package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class PullClimberCmd extends Command {

  private final ClimberSubsystem climber;
  private final double speed;

  public PullClimberCmd(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("PullClimberCmd/Status", "Initialized");
    Logger.recordOutput("PullClimberCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    climber.setPercentOutput(speed);
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