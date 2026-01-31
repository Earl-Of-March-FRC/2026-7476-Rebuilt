package frc.robot.commands.indexer;

import java.util.Queue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCmd extends Command {
  private final IndexerSubsystem m_Indexer;
  private final double m_speed;

  public IndexerCmd(IndexerSubsystem subsystem, double speed) {
    m_Indexer = subsystem;
    m_speed = speed;

    addRequirements(m_Indexer);
  }

  @Override
  public void initialize() {
    System.out.println("Indexer CMD Started");
    m_Indexer.setVelocity(m_speed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake CMD ended.");

    m_Indexer.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
