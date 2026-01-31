package frc.robot.commands.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.launcher.*;

public class LauncherCmd extends Command {
  private final LauncherSubsystem m_launcher;
  public final double m_speed;

  private final PIDController launcherController = new PIDController(0, 1, 1);

  public LauncherCmd(LauncherSubsystem subsystem, double speed, PIDController pid) {
    m_launcher = subsystem;
    m_speed = speed;

    addRequirements(m_launcher);
  }

  @Override
  public void initialize() {
    System.out.println("Launcher CMD started.");
    m_launcher.setVelocity(m_speed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Launcher CMD ended.");

    m_launcher.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}