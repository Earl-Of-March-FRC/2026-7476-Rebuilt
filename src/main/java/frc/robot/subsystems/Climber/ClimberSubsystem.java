package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements ClimberSubsystemInterface {

  private final SparkMax climberSpark;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberPIDController;

  public ClimberSubsystem(SparkMax motor) {
    this.climberSpark = motor;
    this.climberEncoder = motor.getEncoder();
    this.climberPIDController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit((int) ClimberConstants.kSmartCurrentLimit.in(Amps));
    config.encoder.positionConversionFactor(ClimberConstants.kTicksToInchesConversion);
    config.closedLoop
        .p(ClimberConstants.kPIDClimberControllerP)
        .i(ClimberConstants.kPIDClimberControllerI)
        .d(ClimberConstants.kPIDClimberControllerD)
        .outputRange(ClimberConstants.kOutputRangeMin, ClimberConstants.kOutputRangeMax);

    climberSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    Distance position = Inches.of(climberEncoder.getPosition());
    LinearVelocity velocity = InchesPerSecond.of(climberEncoder.getVelocity());

    Logger.recordOutput("Climber/Measured/PositionInches", position.in(Inches));
    Logger.recordOutput("Climber/Measured/VelocityInchesPerSec", velocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AppliedOutput", climberSpark.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/CurrentAmps", climberSpark.getOutputCurrent());
  }

  @Override
  public void setPercentOutput(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
    climberSpark.set(percent);
  }

  @Override
  public void setTargetPosition(double inches) {
    Distance target = Inches.of(inches);
    Logger.recordOutput("Climber/Setpoint/TargetInches", target.in(Inches));
    climberPIDController.setSetpoint(target.in(Inches), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    setPercentOutput(0);
  }

  /** @return Velocity in inches per second. */
  @Override
  public double getVelocity() {
    return climberEncoder.getVelocity();
  }
}