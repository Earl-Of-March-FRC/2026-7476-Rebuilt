package frc.robot.commands.climber;

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
    climber.setClimberSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimbing();
  }
}
