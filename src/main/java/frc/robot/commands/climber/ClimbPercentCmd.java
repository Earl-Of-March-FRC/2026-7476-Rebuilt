// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPercentCmd extends Command {
  private final ClimberSubsystem climbers;
  private final DoubleSupplier percent;

  /** Creates a new ClimbPercentCmd. */
  public ClimbPercentCmd(ClimberSubsystem climbers, DoubleSupplier percent) {
    this.climbers = climbers;
    this.percent = percent;
    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double percentOutput = percent.getAsDouble();
    climbers.setPercentOutput(percentOutput);
    Logger.recordOutput("Commands/ClimbPercentCmd/Percent", percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
