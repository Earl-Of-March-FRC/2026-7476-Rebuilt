package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimbSide {
    Left, Right
  }

  // SparkMax motor implementation
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {
    private final SparkMax sparkMax;

    public SparkMaxClimberMotor(SparkMax sparkMax, SparkMaxConfig config) {
      this.sparkMax = sparkMax;
      sparkMax.configure(config, ResetMode.kResetSafeParameters,
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

  // Main Climber Subsystem
  private final ClimberMotorInterface leftMotor;
  private final ClimberMotorInterface rightMotor;

  public ClimberSubsystem(ClimberMotorInterface leftMotor, ClimberMotorInterface rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Left/Measured/Position/Inches", leftMotor.getPosition());
    Logger.recordOutput("Climber/Left/Measured/Position/Pose3d",
        new Pose3d(0, Inches.of(leftMotor.getPosition()).in(Meters), 0, Rotation3d.kZero));
    Logger.recordOutput("Climber/Left/Measured/VelocityInchesPerSec", leftMotor.getVelocity());
    Logger.recordOutput("Climber/Left/Measured/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Left/Measured/CurrentAmps", leftMotor.getCurrent());

    Logger.recordOutput("Climber/Right/Measured/Position/Inches", rightMotor.getPosition());
    Logger.recordOutput("Climber/Right/Measured/Position/Pose3d",
        new Pose3d(0, Inches.of(rightMotor.getPosition()).in(Meters), 0, Rotation3d.kZero));
    Logger.recordOutput("Climber/Right/Measured/VelocityInchesPerSec", rightMotor.getVelocity());
    Logger.recordOutput("Climber/Right/Measured/AppliedOutput", rightMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Right/Measured/CurrentAmps", rightMotor.getCurrent());
  }

  public void setPercentOutput(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
    leftMotor.setPercentOutput(percent);
    rightMotor.setPercentOutput(percent);
  }

  public void setTargetPosition(double inches) {
    Logger.recordOutput("Climber/Setpoint/Target/Inches", inches);
    Logger.recordOutput("Climber/Setpoint/Target/Pose3d",
        new Pose3d(0, Inches.of(inches).in(Meters), 0, Rotation3d.kZero));
    leftMotor.setTargetPosition(inches);
    rightMotor.setTargetPosition(inches);
  }

  public void stop() {
    setPercentOutput(0);
  }

  public double getVelocity() {
    return leftMotor.getVelocity();
  }

  public void setPercentOutput(double percent, ClimbSide side) {
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/PercentOutput", percent);
    if (side == ClimbSide.Left) {
      leftMotor.setPercentOutput(percent);
    } else {
      rightMotor.setPercentOutput(percent);
    }
  }

  public void setTargetPosition(double inches, ClimbSide side) {
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/Target/Inches", inches);
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/Target/Pose3d",
        new Pose3d(0, Inches.of(inches).in(Meters), 0, Rotation3d.kZero));
    if (side == ClimbSide.Left) {
      leftMotor.setTargetPosition(inches);
    } else {
      rightMotor.setTargetPosition(inches);
    }
  }

  public LinearVelocity getVelocity(ClimbSide side) {
    if (side == ClimbSide.Left) {
      return InchesPerSecond.of(leftMotor.getVelocity());
    } else {
      return InchesPerSecond.of(rightMotor.getVelocity());
    }
  }
}