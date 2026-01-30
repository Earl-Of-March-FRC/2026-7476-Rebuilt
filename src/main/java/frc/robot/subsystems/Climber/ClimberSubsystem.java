package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberSpark;
  private final RelativeEncoder climberEncoder;

  public ClimberSubsystem(SparkMax motor) {
    this.climberSpark = motor;
    climberEncoder = climberSpark.getEncoder();

    // Apply your climber-specific configs (Current limits should be high here!)
    climberSpark.configure(ClimberConfigs.climberConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  // Clanker Generated
  @Override
  public void periodic() {
    // Log climber position or current so you can see if it's straining
    Logger.recordOutput("Climber/AppliedOutput", climberSpark.getAppliedOutput());
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Intake/Setpoint/PercentVelocity", percent);
    climberSpark.set(percent);
  }

  public double getVelocity() {
    return climberEncoder.getVelocity();
  }

  public void stopClimbing() {
    setVelocity(0);
  }

  public void setClimberSpeed(double speed) {

  }

}
