package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class TreadmillOffCmd extends Command {

  private final IndexerSubsystem indexer;
  private final DoubleSupplier wheelSpeed;

  public TreadmillOffCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed) {
    this.indexer = indexer;
    this.wheelSpeed = wheelSpeed;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPulse", false);
  }

  @Override
  public void execute() {
    indexer.setWheelPercent(wheelSpeed.getAsDouble());
    indexer.setTreadmillPercent(0);
    Logger.recordOutput("Commands/IndexerCmd/WheelPercentOutput", wheelSpeed.getAsDouble());
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPercentOutput", 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}