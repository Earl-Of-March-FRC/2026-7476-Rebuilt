package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class RaiseClimberCmd extends Command {
  private final ClimberSubsystem m_climber;
  private final double m_speed;

  public RaiseClimberCmd(ClimberSubsystem climber, double speed) {
    m_climber = climber;
    m_speed = speed;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() { // Go up and
    m_climber.setClimberSpeed(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimbing();
  }
}