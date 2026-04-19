// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OTBIntake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WinchCmd extends Command {
  /** Creates a new WinchCmd. */
  private final OTBIntakeSubsystem intake;
  private final DoubleSupplier speed;

  public WinchCmd(OTBIntakeSubsystem intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setWinchPercent(speed.getAsDouble());
    Logger.recordOutput("WinchCmd/Status", "Initialized");
    Logger.recordOutput("WinchCmd/TargetPercentOutput", speed.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("WinchCmd/MeasuredVelocityRPM", intake.getWinchVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWinch();
    Logger.recordOutput("WinchCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
