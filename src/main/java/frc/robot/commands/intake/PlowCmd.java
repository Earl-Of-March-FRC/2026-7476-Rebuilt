package frc.robot.commands.intake;

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

  // Code here Same premise as other intake command
  @Override
  public void initialize() {
    System.out.println("Plow CMD started.");
    intake.setVelocity(speed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Plow CMD ended.");
    // Safety: Always stop the motor when the command ends!
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}