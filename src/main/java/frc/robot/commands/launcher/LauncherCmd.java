package frc.robot.commands.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.*;

public class LauncherCmd extends Command {
  private final LauncherSubsystem launcher;
  public final double speed;

  private final PIDController launcherController = new PIDController(0, 1, 1);

  public LauncherCmd(LauncherSubsystem subsystem, double speed, PIDController pid) {
    this.launcher = subsystem;
    this.speed = speed;

    addRequirements(this.launcher);
  }

  @Override
  public void initialize() {
    System.out.println("Launcher CMD started.");
    this.launcher.setVelocity(this.speed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Launcher CMD ended.");

    this.launcher.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}