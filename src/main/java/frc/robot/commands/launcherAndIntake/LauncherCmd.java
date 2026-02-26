package frc.robot.commands.launcherAndIntake;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcherAndIntake.LauncherSubsystem;

public class LauncherCmd extends Command {

  private final LauncherSubsystem launcher;
  private final Supplier<AngularVelocity> targetVelocity;

  public LauncherCmd(LauncherSubsystem launcher, Supplier<AngularVelocity> targetVelocity) {
    this.launcher = launcher;
    this.targetVelocity = targetVelocity;
    addRequirements(launcher);
  }

  public LauncherCmd(LauncherSubsystem launcher, AngularVelocity velocity) {
    this(launcher, () -> velocity);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("LauncherPIDCmd/Status", "Initialized");
  }

  @Override
  public void execute() {
    AngularVelocity target = targetVelocity.get();
    launcher.setReferenceVelocity(target);
    Logger.recordOutput("LauncherPIDCmd/TargetRPM", target.in(RPM));
    Logger.recordOutput("LauncherPIDCmd/MeasuredRPM", launcher.getVelocity().in(RPM));
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stop();
    Logger.recordOutput("LauncherPIDCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}