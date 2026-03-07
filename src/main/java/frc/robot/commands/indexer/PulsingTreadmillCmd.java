package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class PulsingTreadmillCmd extends SequentialCommandGroup {

  /**
   * Creates a command that runs the wheel continuously and pulses the treadmill
   * on and off to prevent jamming.
   *
   * @param indexer        The indexer subsystem to control.
   * @param wheelSpeed     The speed for the indexer wheel, in range [-1.0, 1.0].
   * @param treadmillSpeed The speed for the indexer treadmill, in range [-1.0,
   *                       1.0].
   */
  public PulsingTreadmillCmd(IndexerSubsystem indexer, DoubleSupplier wheelSpeed, DoubleSupplier treadmillSpeed) {
    addCommands(
        Commands.sequence(
            new TreadmillOnCmd(indexer, wheelSpeed, treadmillSpeed)
                .withTimeout(Constants.IndexerConstants.kPulseOnSeconds),
            new TreadmillOffCmd(indexer, wheelSpeed)
                .withTimeout(Constants.IndexerConstants.kPulseOffSeconds))
            .repeatedly());
  }

  /**
   * Convenience constructor with constant speeds.
   *
   * @param indexer        The indexer subsystem to control.
   * @param wheelSpeed     The speed for the indexer wheel, in range [-1.0, 1.0].
   * @param treadmillSpeed The speed for the indexer treadmill, in range [-1.0,
   *                       1.0].
   */
  public PulsingTreadmillCmd(IndexerSubsystem indexer, double wheelSpeed, double treadmillSpeed) {
    this(indexer, () -> wheelSpeed, () -> treadmillSpeed);
  }
}