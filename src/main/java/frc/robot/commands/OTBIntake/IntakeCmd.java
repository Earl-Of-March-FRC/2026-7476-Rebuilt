package frc.robot.commands.OTBIntake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;

public class IntakeCmd extends Command {

  private final OTBIntakeSubsystem intake;
  private final DoubleSupplier speed;

  public IntakeCmd(OTBIntakeSubsystem subsystem, DoubleSupplier speed) {
    this.intake = subsystem;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.setRollerPercent(speed.getAsDouble());
    Logger.recordOutput("IntakeCmd/Status", "Initialized");
    Logger.recordOutput("IntakeCmd/TargetPercentOutput", speed.getAsDouble());
  }

  @Override
  public void execute() {
    Logger.recordOutput("IntakeCmd/MeasuredVelocityRPM", intake.getRollerVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopRoller();
    Logger.recordOutput("IntakeCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}