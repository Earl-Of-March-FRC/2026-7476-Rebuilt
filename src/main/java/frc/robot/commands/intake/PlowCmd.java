package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PlowCmd extends Command {
  private final IntakeSubsystem m_intake;
  private final double m_speed;

  public PlowCmd(IntakeSubsystem intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(m_intake);
  }

  // Code here Same premise as other intake command 
  @Override
  public void initialize() {
    m_intake.setVelocity(m_speed);
  }
}