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

public class ClimberSubsystem extends SubsystemBase implements ClimberSubsystemInterface {
  private final SparkMax climberSpark;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;

  public ClimberSubsystem(SparkMax motor) {
    climberSpark = motor;
    climberEncoder = motor.getEncoder();

    climberPIDController = motor.getClosedLoopController();

    SparkMaxConfig climberConfig = new SparkMaxConfig();

    climberConfig.smartCurrentLimit(40);

    climberConfig.encoder.positionConversionFactor(ClimberConstants.kTicksToInchesConversion);

    climberConfig.closedLoop
        .p(ClimberConstants.kP)
        .i(ClimberConstants.kI)
        .d(ClimberConstants.kD)
        .outputRange(-1, 1); // -1, 1 as in -1 full speed backwards, and 1 full speed forwards (Motor maximum
                             // speeds)
                             // It's like duty cycles

    // Reset the safe parameters, don't persist parameters
    climberSpark.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // Log climber position or current so you can see if it's straining

    Logger.recordOutput("Climber/PositionInches", climberEncoder.getPosition());
    Logger.recordOutput("Climber/VelocityInchesPerSec", climberEncoder.getVelocity());
    Logger.recordOutput("Climber/AppliedOutput", climberSpark.getAppliedOutput());
    Logger.recordOutput("Climber/CurrentAmps", climberSpark.getOutputCurrent());
  }

  public void setPercentSpeed(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentVelocity", percent);
    climberSpark.set(percent);
  }

  public void setTargetPosition(double inches) {
    Logger.recordOutput("Climber/Setpoint/TargetInches", inches);
    // TODO ask hardware about gearbox rotations
    climberPIDController.setSetpoint(inches, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stop() {
    setPercentSpeed(0);
  }

  public double getVelocity() {
    return climberEncoder.getVelocity();
  }

}
