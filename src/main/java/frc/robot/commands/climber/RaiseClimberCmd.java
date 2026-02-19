package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystemInterface;

public class RaiseClimberCmd extends Command {
  private final ClimberSubsystemInterface climber;
  private final double target;

  public RaiseClimberCmd(ClimberSubsystemInterface climber, double target) {
    this.climber = climber;
    this.target = target;
    addRequirements((edu.wpi.first.wpilibj2.command.Subsystem) climber);
  }

  @Override
  public void initialize() { // Go up and
    this.climber.setTargetPosition(target);
  }

  @Override
  public void end(boolean interrupted) {
    // this.climber.stopClimbing();
  }
}