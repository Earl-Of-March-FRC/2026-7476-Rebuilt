package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class PulsingTreadmillCmd extends Command {

  public enum PulseShape {
    SIN_SQUARED,
    REVERSE_SAWTOOTH
  }

  private final IndexerSubsystem indexer;
  private final DoubleSupplier wheelSpeed;
  private final DoubleSupplier treadmillSpeed;
  private final PulseShape shape;

  private final double onDuration;
  private final double offDuration;
  private final double period;

  private int executeCycle = 0;

  public PulsingTreadmillCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    this(indexer, wheelSpeed, treadmillSpeed, Constants.IndexerConstants.kDefaultPulseShape);
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed,
      PulseShape shape) {
    this.indexer = indexer;
    this.wheelSpeed = wheelSpeed;
    this.treadmillSpeed = treadmillSpeed;
    this.shape = shape;
    this.onDuration = Constants.IndexerConstants.kPulseOnSeconds;
    this.offDuration = Constants.IndexerConstants.kPulseOffSeconds;
    this.period = onDuration + offDuration;
    addRequirements(indexer);
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, double wheelSpeed, double treadmillSpeed) {
    this(indexer, () -> wheelSpeed, () -> treadmillSpeed);
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, double wheelSpeed, double treadmillSpeed, PulseShape shape) {
    this(indexer, () -> wheelSpeed, () -> treadmillSpeed, shape);
  }

  @Override
  public void initialize() {
    executeCycle = 0;
  }

  @Override
  public void execute() {
    // Time within the current period (resets every full on+off cycle)
    double t = (executeCycle * 0.02) % period;
    executeCycle++;

    double wheel = wheelSpeed.getAsDouble();

    if (t < onDuration) {
      // ON phase, apply envelope
      double normalizedT = t / onDuration; // 0 to 1 within ON window
      double envelope = computeEnvelope(normalizedT, shape);
      double treadmill = treadmillSpeed.getAsDouble() * envelope;
      indexer.setWheelPercent(wheel);
      indexer.setTreadmillPercent(treadmill);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillEnvelope", envelope);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillT", normalizedT);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillPercentOutput", treadmill);
    } else {
      // OFF phase
      indexer.setWheelPercent(wheel);
      indexer.setTreadmillPercent(0);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillEnvelope", 0.0);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillT", 0.0);
      Logger.recordOutput("Commands/IndexerCmd/TreadmillPercentOutput", 0.0);
    }

    Logger.recordOutput("Commands/IndexerCmd/WheelPercentOutput", wheel);
    Logger.recordOutput("Commands/IndexerCmd/TreadmillCycle", executeCycle);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    Logger.recordOutput("Commands/IndexerCmd/TreadmillPulse", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double sinSquared(double t) {
    return Math.pow(Math.sin(Math.PI * t), 2);
  }

  private static double reverseSawtooth(double t) {
    return 1.0 - t;
  }

  private static double computeEnvelope(double t, PulseShape shape) {
    t = Math.max(0.0, Math.min(1.0, t));
    switch (shape) {
      case SIN_SQUARED:
        return sinSquared(t);
      case REVERSE_SAWTOOTH:
        return reverseSawtooth(t);
      default:
        return sinSquared(t);
    }
  }
}