package frc.robot.commands.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystemInterface;
import frc.robot.subsystems.Climber.TalonFXClimberSubsystem;

public class PullClimberCmd extends Command {

  private final ClimberSubsystemInterface climber;
  private final double speed;

  /**
   * Drives the climber at a fixed percent output (open-loop pull).
   *
   * @param climber The climber subsystem.
   * @param speed   Output in range [-1.0, 1.0]. Negative pulls down.
   */
  public PullClimberCmd(ClimberSubsystemInterface climber, double speed) {
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("PullClimberCmd/Status", "Initialized");
    Logger.recordOutput("PullClimberCmd/TargetPercentOutput", speed);
  }

  @Override
  public void execute() {
    climber.setPercentOutput(speed);
    Logger.recordOutput("PullClimberCmd/MeasuredVelocityInchesPerSec", climber.getVelocity());
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    Logger.recordOutput("PullClimberCmd/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}