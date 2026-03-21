package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCmd extends Command {

  private final IndexerSubsystem indexer;
  private final DoubleSupplier wheelSpeed, treadmillSpeed;

  // Pulse timing constants
  private static final double kPulseOnSeconds = 0.15;
  private static final double kPulseOffSeconds = 0.10;

  private final Timer pulseTimer = new Timer();
  private boolean pulseOn = true;

  public IndexerCmd(IndexerSubsystem subsystem, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    this.indexer = subsystem;
    this.wheelSpeed = wheelSpeed;
    this.treadmillSpeed = treadmillSpeed;
    addRequirements(this.indexer);
  }

  public IndexerCmd(IndexerSubsystem subsystem, DoubleSupplier speed) {
    this(subsystem, speed, speed);
  }

  @Override
  public void initialize() {
    pulseTimer.restart();
    pulseOn = true;
    Logger.recordOutput("Commands/IndexerCmd/Status", "Initialized");
  }

  @Override
  public void execute() {
    // Flip pulse state when timer expires
    if (pulseOn && pulseTimer.hasElapsed(kPulseOnSeconds)) {
      pulseOn = false;
      pulseTimer.restart();
    } else if (!pulseOn && pulseTimer.hasElapsed(kPulseOffSeconds)) {
      pulseOn = true;
      pulseTimer.restart();
    }

    double treadmill = pulseOn ? treadmillSpeed.getAsDouble() : 0.0;

    indexer.setWheelPercent(wheelSpeed.getAsDouble());
    indexer.setTreadmillPercent(treadmill);

    Logger.recordOutput("Commands/IndexerCmd/MeasuredVelocityRPM", indexer.getWheelVelocity());
    Logger.recordOutput("Commands/IndexerCmd/WheelPercentOutput", wheelSpeed.getAsDouble());
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPercentOutput", treadmill);
    Logger.recordOutput("Commands/IndexerCmd/PulseOn", pulseOn);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    Logger.recordOutput("Commands/IndexerCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}