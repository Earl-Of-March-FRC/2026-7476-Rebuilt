package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class TreadmillOnCmd extends Command {

  private final IndexerSubsystem indexer;
  private final DoubleSupplier wheelSpeed;
  private final DoubleSupplier treadmillSpeed;

  public TreadmillOnCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    this.indexer = indexer;
    this.wheelSpeed = wheelSpeed;
    this.treadmillSpeed = treadmillSpeed;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPulse", true);
  }

  @Override
  public void execute() {
    indexer.setWheelPercent(wheelSpeed.getAsDouble());
    indexer.setTreadmillPercent(treadmillSpeed.getAsDouble());
    Logger.recordOutput("Commands/IndexerCmd/WheelPercentOutput", wheelSpeed.getAsDouble());
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPercentOutput", treadmillSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPulse", false);
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}