package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.PullClimberCmd;
import frc.robot.commands.climber.RaiseClimberCmd;
import frc.robot.commands.drivetrain.AlignTowerCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AutoClimbTowerCmdGroup extends SequentialCommandGroup {

  public AutoClimbTowerCmdGroup(DrivetrainSubsystem drive, ClimberSubsystem climber) {
    addCommands(
        // Parallel: Align the robot while simultaneously raising the arm
        new ParallelCommandGroup(
            new AlignTowerCmd(drive).withTimeout(3.0),
            new RaiseClimberCmd(climber)),

        // Sequential: Once aligned and raised, pull the robot up
        new PullClimberCmd(climber));
  }
}