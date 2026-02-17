// package frc.robot.commands.launcher;

// import java.util.function.DoubleSupplier;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.launcher.*;

// public class LauncherCmd extends Command {
// private final LauncherPIDCmd launcher;

// private final DoubleSupplier targetRPM;

// public LauncherCmd(LauncherSubsystem subsystem, DoubleSupplier targetRPM) {
// this.launcher = subsystem;
// this.targetRPM = targetRPM;

// addRequirements(launcher);
// }

// @Override
// public void initialize() {
// System.out.println("Launcher CMD started.");

// }

// public LauncherCmd(LauncherSubsystem subsystem, double speed) {
// this(subsystem, () -> speed);
// }

// @Override
// public void execute() {
// launcher.setReferenceVelocity(
// targetRPM.getAsDouble());
// }

// @Override
// public void end(boolean interrupted) {
// System.out.println("Launcher CMD ended.");

// this.launcher.setVelocity(0);
// }

// @Override
// public boolean isFinished() {
// return false;
// }
// }