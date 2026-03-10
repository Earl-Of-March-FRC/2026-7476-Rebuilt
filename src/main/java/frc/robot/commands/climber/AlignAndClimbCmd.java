package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drivetrain.AlignTowerCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignAndClimbCmd extends SequentialCommandGroup { // Commands that come one after the other.

  public AlignAndClimbCmd(DrivetrainSubsystem drive, ClimberSubsystem climber, BooleanSupplier usesLeftMotor,
      BooleanSupplier usesRightMotor) {

    ParallelCommandGroup raiseClimberCmds = new ParallelCommandGroup();
    ParallelCommandGroup pullClimberCmds = new ParallelCommandGroup(); // If it ain't broke, don't fix it

    if (usesLeftMotor.getAsBoolean()) {
      raiseClimberCmds.addCommands(new RaiseClimberCmd(climber, () -> true));
      pullClimberCmds.addCommands(new PullClimberCmd(climber, () -> true));
    }

    if (usesRightMotor.getAsBoolean()) {
      raiseClimberCmds.addCommands(new RaiseClimberCmd(climber, () -> false));
      pullClimberCmds.addCommands(new PullClimberCmd(climber, () -> false));
    }

    raiseClimberCmds.addCommands(new AlignTowerCmd(drive, usesLeftMotor.getAsBoolean(), usesRightMotor.getAsBoolean())
        .withTimeout(AutoConstants.kAlignTowerTimeoutSeconds));

    addCommands( // These commands are going to be done:
        // Parallel: Align the robot while simultaneously raising the arm
        raiseClimberCmds,
        pullClimberCmds
    // Sequential: Once aligned and raised, pull the robot up
    );
  }
}