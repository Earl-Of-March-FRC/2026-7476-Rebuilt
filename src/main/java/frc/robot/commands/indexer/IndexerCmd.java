package frc.robot.commands.indexer;

import java.util.Queue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCmd extends Command {
  private final IndexerSubsystem Indexer;
  private final double speed;

  public IndexerCmd(IndexerSubsystem subsystem, double speed) {
    this.Indexer = subsystem;
    this.speed = speed;

    addRequirements(this.Indexer);
  }

  @Override
  public void initialize() {
    System.out.println("Indexer CMD Started");
    this.Indexer.setVelocity(this.speed);
  }

  @Override
  public void execute() {
    this.Indexer.setVelocity(50);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake CMD ended.");

    this.Indexer.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
