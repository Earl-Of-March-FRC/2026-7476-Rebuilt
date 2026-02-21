package frc.robot.commands.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCmd extends Command {

  private final IndexerSubsystem indexer;
  private final double speed;

  public IndexerCmd(IndexerSubsystem subsystem, double speed) {
    this.indexer = subsystem;
    this.speed = speed;
    addRequirements(this.indexer);
  }

  @Override
  public void initialize() {
    indexer.setVelocity(speed);
    Logger.recordOutput("IndexerCmd/Status", "Initialized");
    Logger.recordOutput("IndexerCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    // Log measured velocity for tuning visibility.
    Logger.recordOutput("IndexerCmd/MeasuredVelocityRPM", indexer.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    Logger.recordOutput("IndexerCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}