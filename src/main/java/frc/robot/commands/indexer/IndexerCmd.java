package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCmd extends Command {

  private final IndexerSubsystem indexer;
  private final DoubleSupplier wheelSpeed, treadmillSpeed;

  /**
   * Creates a command to run the indexer at the given speeds.
   * 
   * @param subsystem      The indexer subsystem to control.
   * @param wheelSpeed     The speed for the indexer wheel, in range [-1.0, 1.0].
   * @param treadmillSpeed The speed for the indexer treadmill, in range [-1.0,
   *                       1.0].
   */
  public IndexerCmd(IndexerSubsystem subsystem, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    this.indexer = subsystem;
    this.wheelSpeed = wheelSpeed;
    this.treadmillSpeed = treadmillSpeed;
    addRequirements(this.indexer);
  }

  /**
   * Convenience constructor for when the wheel and treadmill speeds are the same
   * 
   * @param subsystem The indexer subsystem to control.
   * @param speed     The speed for both the wheel and treadmill, in range [-1.0,
   *                  1.0].
   */
  public IndexerCmd(IndexerSubsystem subsystem, DoubleSupplier speed) {
    this(subsystem, speed, speed);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("IndexerCmd/Status", "Initialized");
    Logger.recordOutput("IndexerCmd/WheelPercentOutput", wheelSpeed.getAsDouble());
    Logger.recordOutput("IndexerCmd/TreadmillPercentOutput", treadmillSpeed.getAsDouble());
  }

  @Override
  public void execute() {
    indexer.setWheelPercent(wheelSpeed.getAsDouble());
    indexer.setTreadmillPercent(treadmillSpeed.getAsDouble());

    // Log measured velocity for tuning visibility.
    Logger.recordOutput("IndexerCmd/MeasuredVelocityRPM", indexer.getWheelVelocity());
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