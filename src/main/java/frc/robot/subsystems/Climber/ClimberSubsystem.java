package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.UnitHelpers;

public class ClimberSubsystem extends SubsystemBase {

  public static enum ClimbSide {
    Left, Right
  }

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
    private final TalonFXSimState talonFXSimState;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    // Percent flags/variables are for simulation purposes
    private boolean usingPercent = false;
    private double percent = 0;
    private double lastSimulationSeconds = 0; // Kept as double for simulation efficency

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

      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public void setPercentOutput(double percent) {
      this.percent = MathUtil.clamp(percent, -1, 1);
      usingPercent = true;
      talonFX.set(percent);
    }

    @Override
    public void setTargetPosition(double inches) {
      usingPercent = false;
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

    @Override
    public void simulationPeriodic() {
      final double currentTime = Timer.getFPGATimestamp();
      final double deltaTime = currentTime - lastSimulationSeconds; // In seconds
      talonFXSimState
          .setSupplyVoltage(RobotController.getBatteryVoltage());

      // Calculate velocity
      final AngularVelocity rotorVelocity;
      if (usingPercent) {
        rotorVelocity = SimulationConstants.kSimulatedMaxClimberSpeed.times(percent);
      } else {
        final double errorPosition = positionRequest.Position * ClimberConstants.kRotationsToInchesConversion
            - getPosition();
        final AngularVelocity desiredVelocity = RotationsPerSecond.of(errorPosition / deltaTime);
        rotorVelocity = (AngularVelocity) UnitHelpers.clamp(desiredVelocity,
            SimulationConstants.kSimulatedMaxClimberSpeed.times(-1),
            SimulationConstants.kSimulatedMaxClimberSpeed);
      }
      talonFXSimState.setRotorVelocity(rotorVelocity);
      talonFXSimState
          .setRawRotorPosition(Math.max(getPosition() + (rotorVelocity.in(RotationsPerSecond) * deltaTime), 0));

      lastSimulationSeconds = currentTime;
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
        new Pose3d(0, 0, Inches.of(leftMotor.getPosition()).in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Left/Measured/VelocityInchesPerSec", leftMotor.getVelocity());
    Logger.recordOutput("Climber/Left/Measured/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Left/Measured/CurrentAmps", leftMotor.getCurrent());

    Logger.recordOutput("Climber/Right/Measured/Position/Inches", rightMotor.getPosition());
    Logger.recordOutput("Climber/Right/Measured/Position/Pose3d",
        new Pose3d(0, 0, Inches.of(rightMotor.getPosition()).in(Meters), Rotation3d.kZero));
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

  @Override
  public void simulationPeriodic() {
    leftMotor.simulationPeriodic();
    rightMotor.simulationPeriodic();
  }
}