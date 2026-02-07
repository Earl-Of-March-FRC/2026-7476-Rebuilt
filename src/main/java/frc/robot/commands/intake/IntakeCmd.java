package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCmd extends Command {
  private final IntakeSubsystem intake;
  private final double speed;

  public IntakeCmd(IntakeSubsystem subsystem, double speed) {
    this.intake = subsystem;
    this.speed = speed;

    // IMPORTANT: This tells the robot that no other command can use the intake
    // while this one is running.
    addRequirements(this.intake);
  }

  // Initialize the stuff
  @Override
  public void initialize() {
    System.out.println("Intake CMD started.");
    intake.setVelocity(speed);
  }

  // Excecute the stuff

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake CMD ended.");
    // Safety: Always stop the motor when the command ends!
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // end (stop the motors and stuff)

}
