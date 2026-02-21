package frc.robot.commands.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCmd extends Command {

  private final IntakeSubsystem intake;
  private final double speed;

  public IntakeCmd(IntakeSubsystem subsystem, double speed) {
    this.intake = subsystem;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.setVelocity(speed);
    Logger.recordOutput("IntakeCmd/Status", "Initialized");
    Logger.recordOutput("IntakeCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    Logger.recordOutput("IntakeCmd/MeasuredVelocityRPM", intake.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    Logger.recordOutput("IntakeCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}