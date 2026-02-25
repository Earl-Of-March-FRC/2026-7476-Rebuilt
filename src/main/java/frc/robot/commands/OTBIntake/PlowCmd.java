package frc.robot.commands.OTBIntake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;

public class PlowCmd extends Command {

  private final OTBIntakeSubsystem intake;
  private final DoubleSupplier speed;

  public PlowCmd(OTBIntakeSubsystem intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.setRollerPercent(speed.getAsDouble());
    Logger.recordOutput("PlowCmd/Status", "Initialized");
    Logger.recordOutput("PlowCmd/TargetPercentOutput", speed.getAsDouble());
  }

  @Override
  public void execute() {
    Logger.recordOutput("PlowCmd/MeasuredVelocityRPM", intake.getRollerVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopRoller();
    Logger.recordOutput("PlowCmd/Status", interrupted ? "Interrupted" : "Completed");

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}