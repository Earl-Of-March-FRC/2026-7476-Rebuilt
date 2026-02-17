package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystemInterface;

public class PullClimberCmd extends Command {
  private final ClimberSubsystemInterface climber;
  private final double speed;

  public PullClimberCmd(ClimberSubsystemInterface climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements((edu.wpi.first.wpilibj2.command.Subsystem) climber);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
