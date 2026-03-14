// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.ClimbPercentCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RaiseAndPullClimberCmd extends SequentialCommandGroup {
  /**
   * Creates a command that raises the climbers, waits, and pulls the climbers
   * 
   * @param climber                 Climber subsystem
   * @param waitBetweenRaiseAndPull Time to wait before the climber starts pulling
   *                                to climbed position
   * @see {@link ClimberConstants#kTimeFromBottomToRaisedPosition}
   *      {@link ClimberConstants#kTimeFromRaisedToClimbedPosition}
   */
  public RaiseAndPullClimberCmd(ClimberSubsystem climber, Time waitBetweenRaiseAndPull) {
    addCommands(
        new ReturnClimbersToBottomCmd(climber), new ClimbPercentCmd(climber, () -> ClimberConstants.kOutputRangeMax)
            .withTimeout(ClimberConstants.kTimeFromBottomToRaisedPosition),
        new WaitCommand(waitBetweenRaiseAndPull), new ClimbPercentCmd(climber, () -> ClimberConstants.kOutputRangeMin)
            .withTimeout(ClimberConstants.kTimeFromRaisedToClimbedPosition));
  }
}
