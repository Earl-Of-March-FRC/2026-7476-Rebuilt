// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.ClimbPercentCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnClimbersToBottomCmd extends SequentialCommandGroup {
  /** Creates a new ReturnClimbersToBottom. */
  public ReturnClimbersToBottomCmd(ClimberSubsystem climbers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClimbPercentCmd(climbers, () -> -ClimberConstants.kOutputUp).onlyWhile(() -> {
          boolean can = !climbers.isEitherAtBottom();
          Logger.recordOutput("Commands/ReturnClimbersToBottom/CanRun", can);
          return can;
        }));
  }
}
