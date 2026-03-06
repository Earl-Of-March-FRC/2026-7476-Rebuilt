package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drivetrain.AlignTowerCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignAndClimbCmd extends SequentialCommandGroup { // Commands that come one after the other.

  public AlignAndClimbCmd(DrivetrainSubsystem drive, ClimberSubsystem climber, boolean usesLeftMotor,
      boolean usesRightMotor) {

    ParallelCommandGroup raiseClimberCmds = new ParallelCommandGroup();
    ParallelCommandGroup pullClimberCmds = new ParallelCommandGroup();

    if (usesLeftMotor) {
      raiseClimberCmds.addCommands(new RaiseClimberCmd(climber, true));
      pullClimberCmds.addCommands(new PullClimberCmd(climber, true));
    }

    if (usesRightMotor) {
      raiseClimberCmds.addCommands(new RaiseClimberCmd(climber, false));
      pullClimberCmds.addCommands(new PullClimberCmd(climber, false));
    }

    addCommands( // These commands are going to be done:
        // Parallel: Align the robot while simultaneously raising the arm
        new AlignTowerCmd(drive, usesLeftMotor, usesRightMotor).withTimeout(AutoConstants.kAlignTowerTimeoutSeconds),
        raiseClimberCmds,
        pullClimberCmds
    // Sequential: Once aligned and raised, pull the robot up
    );
  }
}