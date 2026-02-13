package frc.robot.commands.launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherPIDInterface;

public class LauncherPIDCmd extends Command {

  private final LauncherPIDInterface launcher; // This is an interface, but you can still access methods in the class
                                               // that implements it.
  private final DoubleSupplier targetRPM;

  public LauncherPIDCmd(LauncherPIDInterface launcher, DoubleSupplier targetRPM) {
    this.launcher = launcher;
    this.targetRPM = targetRPM;

    if (launcher instanceof edu.wpi.first.wpilibj2.command.Subsystem) {
      addRequirements((edu.wpi.first.wpilibj2.command.Subsystem) launcher); // Cast the type and set the launcher motor
                                                                            // as a dependency
    }
  }

  public LauncherPIDCmd(LauncherPIDInterface launcher, double rpm) {
    this(launcher, () -> rpm);
  }

  @Override
  public void initialize() {
    System.out.println("Launcher PID CMD started");
  }

  @Override
  public void execute() {
    launcher.setReferenceVelocity(targetRPM.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stop();
    System.out.println("Launcher PID CMD ended");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}