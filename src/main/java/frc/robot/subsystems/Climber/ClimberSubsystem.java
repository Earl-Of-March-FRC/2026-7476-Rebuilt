package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController; // The PID "Brain"
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberSpark;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;
  private final SparkMaxConfig climberSparkMaxConfig = new SparkMaxConfig();

  public ClimberSubsystem(SparkMax motor) {
    climberSpark = motor;
    climberEncoder = climberSpark.getEncoder();

    climberPIDController = climberSpark.getClosedLoopController();

    climberSparkMaxConfig.closedLoop
        .p(0.0)
        .i(0.0)
        .d(0.0)
        .outputRange(ClimberConstants.kMinClimberLength, ClimberConstants.kMaxClimberLength);

    // Apply your climber-specific configs (Current limits should be high here!)
    // climberSpark.configure(ClimberConfigs.climberConfig,
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Log climber position or current so you can see if it's straining
    Logger.recordOutput("Climber/AppliedOutput", climberSpark.getAppliedOutput());
  }

  public void setTarget(double inches) {
    Logger.recordOutput("Climber/Setpoint/TargetInches", inches);

    climberPIDController.setSetpoint(inches, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentVelocity", percent);
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
