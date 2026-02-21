package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class TalonFXClimberSubsystem extends SubsystemBase implements ClimberSubsystemInterface {

  private final TalonFX climberTalon;
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  public TalonFXClimberSubsystem(TalonFX climberTalon) {
    this.climberTalon = climberTalon;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ClimberConstants.kPIDClimberControllerP;
    config.Slot0.kI = ClimberConstants.kPIDClimberControllerI;
    config.Slot0.kD = ClimberConstants.kPIDClimberControllerD;
    config.CurrentLimits.StatorCurrentLimit = ClimberConstants.kStatorCurrentLimit.in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.kSensorToMechanismRatio;

    climberTalon.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    Distance position = Inches.of(climberTalon.getPosition().getValueAsDouble());
    LinearVelocity velocity = InchesPerSecond.of(
        RotationsPerSecond.of(climberTalon.getVelocity().getValueAsDouble()).magnitude());

    Logger.recordOutput("Climber/Measured/PositionInches", position.in(Inches));
    Logger.recordOutput("Climber/Measured/VelocityInchesPerSec", velocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/StatorCurrentAmps",
        climberTalon.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Climber/Measured/AppliedDutyCycle",
        climberTalon.getDutyCycle().getValueAsDouble());
  }

  @Override
  public void setPercentOutput(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
    climberTalon.set(percent);
  }

  @Override
  public void setTargetPosition(double inches) {
    Distance target = Inches.of(inches);
    Logger.recordOutput("Climber/Setpoint/TargetInches", target.in(Inches));
    climberTalon.setControl(positionRequest.withPosition(target.in(Inches)));
  }

  @Override
  public void stop() {
    setPercentOutput(0);
  }

  /** @return Velocity in inches per second. */
  @Override
  public double getVelocity() {
    return climberTalon.getVelocity().getValueAsDouble();
  }
}