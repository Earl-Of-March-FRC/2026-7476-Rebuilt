package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class PulsingTreadmillCmd extends SequentialCommandGroup {

  public enum PulseShape {
    SIN_SQUARED,
    REVERSE_SAWTOOTH
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    this(indexer, wheelSpeed, treadmillSpeed, Constants.IndexerConstants.kDefaultPulseShape);
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed,
      PulseShape shape) {
    final double onDuration = Constants.IndexerConstants.kPulseOnSeconds;
    final double offDuration = Constants.IndexerConstants.kPulseOffSeconds;

    final int[] executeCycle = { 0 };

    DoubleSupplier shapedTreadmill = () -> {
      double t = (executeCycle[0]++ * 0.02) / onDuration; // normalized 0→1
      double envelope = computeEnvelope(t, shape);
      return treadmillSpeed.getAsDouble() * envelope;
    };

    addCommands(
        Commands.sequence(
            Commands.runOnce(() -> executeCycle[0] = 0),
            new TreadmillOnCmd(indexer, wheelSpeed, shapedTreadmill)
                .withTimeout(onDuration),
            new TreadmillOffCmd(indexer, wheelSpeed)
                .withTimeout(offDuration))
            .repeatedly());
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, double wheelSpeed, double treadmillSpeed) {
    this(indexer, () -> wheelSpeed, () -> treadmillSpeed);
  }

  public PulsingTreadmillCmd(IndexerSubsystem indexer, double wheelSpeed, double treadmillSpeed, PulseShape shape) {
    this(indexer, () -> wheelSpeed, () -> treadmillSpeed, shape);
  }

  private static double sinSquared(double t) {
    return Math.pow(Math.sin(Math.PI * t), 2);
  }

  private static double reverseSawtooth(double t) {
    return 1.0 - t;
  }

  private static double computeEnvelope(double t, PulseShape shape) {
    // Clamp t to [0, 1] to avoid overshoot on the last cycle
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