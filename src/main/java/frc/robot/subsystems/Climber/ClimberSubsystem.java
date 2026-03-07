package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public boolean isClimberAtBottom = true;

  public static boolean ClimbMotor;
  // True = Left
  // False = Right

  // SparkMax motor implementation
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {
    private final SparkMax sparkMax;

    public SparkMaxClimberMotor(SparkMax sparkMax) {
      this.sparkMax = sparkMax;
      sparkMax.configure(ClimberConstants.kConfigLeft, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
      sparkMax.configure(ClimberConstants.kConfigRight, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPercentOutput(double percent) {
      sparkMax.set(percent);
    }

    @Override
    public void setTargetPosition(double inches) {
      sparkMax.getClosedLoopController().setSetpoint(inches, SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public double getVelocity() {
      return sparkMax.getEncoder().getVelocity();
    }

    @Override
    public double getPosition() {
      return sparkMax.getEncoder().getPosition();
    }

    @Override
    public double getAppliedOutput() {
      return sparkMax.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
      return sparkMax.getOutputCurrent();
    }
  }

  // TalonFX motor implementation
  public static class TalonFXClimberMotor implements ClimberMotorInterface {
    private final TalonFX talonFX;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public TalonFXClimberMotor(TalonFX talonFX) {
      this.talonFX = talonFX;

      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = ClimberConstants.kPIDClimberControllerP;
      config.Slot0.kI = ClimberConstants.kPIDClimberControllerI;
      config.Slot0.kD = ClimberConstants.kPIDClimberControllerD;
      config.CurrentLimits.StatorCurrentLimit = ClimberConstants.kStatorCurrentLimit.in(Amps);
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.Feedback.SensorToMechanismRatio = ClimberConstants.kSensorToMechanismRatio;
      talonFX.getConfigurator().apply(config);
    }

    @Override
    public void setPercentOutput(double percent) {
      talonFX.set(percent);
    }

    @Override
    public void setTargetPosition(double inches) {
      talonFX.setControl(positionRequest.withPosition(inches));
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public double getVelocity() {
      return talonFX.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPosition() {
      return talonFX.getPosition().getValueAsDouble();
    }

    @Override
    public double getAppliedOutput() {
      return talonFX.getDutyCycle().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
      return talonFX.getStatorCurrent().getValueAsDouble();
    }
  }

  public double getPosition(boolean isLeftSide) {
    return isLeftSide == true ? leftMotor.getPosition() : rightMotor.getPosition();
  }

  // Main Climber Subsystem
  private final ClimberMotorInterface leftMotor;
  private final ClimberMotorInterface rightMotor;

  public ClimberSubsystem(ClimberMotorInterface leftMotor, ClimberMotorInterface rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Left/Measured/PositionInches", leftMotor.getPosition());
    Logger.recordOutput("Climber/Left/Measured/VelocityInchesPerSec", leftMotor.getVelocity());
    Logger.recordOutput("Climber/Left/Measured/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Left/Measured/CurrentAmps", leftMotor.getCurrent());

    Logger.recordOutput("Climber/Right/Measured/PositionInches", rightMotor.getPosition());
    Logger.recordOutput("Climber/Right/Measured/VelocityInchesPerSec", rightMotor.getVelocity());
    Logger.recordOutput("Climber/Right/Measured/AppliedOutput", rightMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Right/Measured/CurrentAmps", rightMotor.getCurrent());
  }

  public void setPercentOutput(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
    leftMotor.setPercentOutput(percent);
    rightMotor.setPercentOutput(percent);
  }

  public void setTargetPositionLeft(double inches) {
    Logger.recordOutput("Climber/Setpoint/TargetInchesLeft", inches);

    leftMotor.setTargetPosition(inches);

  }

  public void setTargetPositionRight(double inches) {
    Logger.recordOutput("Climber/Setpoint/TargetInchesRight", inches);

    rightMotor.setTargetPosition(inches);

  }

  public void stop() {
    setPercentOutput(0);
  }

  public void setPercentOutput(double percent, boolean isLeftSide) {
    Logger.recordOutput("Climber/" + (isLeftSide ? "right" : "left") + "/Setpoint/PercentOutput", percent);
    if (isLeftSide == true) {
      leftMotor.setPercentOutput(percent);
    } else {
      rightMotor.setPercentOutput(percent);
    }
  }

  public void setTargetPosition(double ticks, boolean isLeftSide) {
    Logger.recordOutput("Climber/" + (isLeftSide ? "right" : "left") + "/Setpoint/TargetInches", ticks);
    if (isLeftSide) {
      leftMotor.setTargetPosition(ticks);
    } else {
      rightMotor.setTargetPosition(ticks);
    }
  }

  public LinearVelocity getVelocity(boolean isLeftSide) {
    if (isLeftSide == true) {
      return InchesPerSecond.of(leftMotor.getVelocity());
    } else {
      return InchesPerSecond.of(rightMotor.getVelocity());
    }
  }
}