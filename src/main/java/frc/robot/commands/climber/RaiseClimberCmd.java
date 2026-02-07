package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class RaiseClimberCmd extends Command {
  private final ClimberSubsystem climber;
  private final double speed;

  public RaiseClimberCmd(ClimberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() { // Go up and
    this.climber.setClimberSpeed(this.speed);
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.stopClimbing();
  }
}