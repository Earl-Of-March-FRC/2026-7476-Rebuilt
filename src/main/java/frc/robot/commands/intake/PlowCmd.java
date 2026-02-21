package frc.robot.commands.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PlowCmd extends Command {

  private final IntakeSubsystem intake;
  private final double speed;

  public PlowCmd(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.setVelocity(speed);
    Logger.recordOutput("PlowCmd/Status", "Initialized");
    Logger.recordOutput("PlowCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    Logger.recordOutput("PlowCmd/MeasuredVelocityRPM", intake.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    Logger.recordOutput("PlowCmd/Status", interrupted ? "Interrupted" : "Completed");
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}