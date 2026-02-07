package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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

    climberSparkMaxConfig.smartCurrentLimit(40);

    climberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.kTicksToInchesConversion);

    climberSparkMaxConfig.closedLoop
        .p(ClimberConstants.kP)
        .i(ClimberConstants.kI)
        .d(ClimberConstants.kD)
        .outputRange(-1, 1); // -1, 1 as in -1 full speed backwards, and 1 full speed forwards (Motor maximum
                             // speeds)

    // Don't reset the safe parameters, don't persist parameters Whatever this means
    climberSpark.configure(climberSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
    // TODO ask hardware about gearbox rotations
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
