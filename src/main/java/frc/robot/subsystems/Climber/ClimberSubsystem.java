package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.UnitHelpers;

public class ClimberSubsystem extends SubsystemBase {
  public boolean isClimberAtBottom = true;

  public static boolean ClimbMotor;
  // True = Left
  // False = Right

  // SparkMax motor implementation
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {
    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;

    // Percent flags/variables are for simulation purposes
    private boolean usingPercent = false;
    private double setpoint = 0;
    private double lastSimulationSeconds = 0; // Kept as double for simulation efficency

    public SparkMaxClimberMotor(SparkMax sparkMax, SparkMaxConfig config) {
      this.sparkMax = sparkMax;
      sparkMax.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
      if (RobotBase.isSimulation()) {
        this.sparkMaxSim = new SparkMaxSim(sparkMax, SimulationConstants.kSimulatedSparkMaxClimberMotor);
      } else {
        this.sparkMaxSim = null;
      }
    }

    @Override
    public void setPercentOutput(double percent) {
      sparkMax.set(percent);
      usingPercent = true;
      setpoint = percent;
    }

    @Override
    public void setTargetPosition(double inches) {
      usingPercent = false;
      setpoint = inches;
      sparkMax.getClosedLoopController().setSetpoint(inches, SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public double getVelocity() {
      if (RobotBase.isSimulation() && sparkMaxSim != null) {
        return sparkMaxSim.getVelocity();
      }
      return sparkMax.getEncoder().getVelocity();
    }

    @Override
    public double getPosition() {
      if (RobotBase.isSimulation() && sparkMaxSim != null) {
        return sparkMaxSim.getPosition();
      }
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

    @Override
    public void simulationPeriodic() {
      if (sparkMaxSim == null) {
        return;
      }

      final double currentTime = Timer.getFPGATimestamp();
      double deltaTime = currentTime - lastSimulationSeconds; // In seconds
      if (deltaTime <= 0) {
        deltaTime = 0.02; // Default 20ms
      }
      sparkMaxSim.setBusVoltage(RobotController.getBatteryVoltage());

      // Calculate velocity
      final LinearVelocity velocity;

      if (usingPercent) {
        velocity = SimulationConstants.kSimulatedMaxClimberSpeed.times(setpoint);
      } else {
        final double errorPosition = setpoint - getPosition();
        final LinearVelocity desiredVelocity = InchesPerSecond.of(errorPosition / deltaTime);
        velocity = (LinearVelocity) UnitHelpers.clamp(desiredVelocity,
            SimulationConstants.kSimulatedMaxClimberSpeed.times(-1),
            SimulationConstants.kSimulatedMaxClimberSpeed);
      }
      sparkMaxSim.setVelocity(velocity.in(InchesPerSecond));
      sparkMaxSim
          .setPosition(MathUtil.clamp(sparkMaxSim.getPosition() + (velocity.in(InchesPerSecond) * deltaTime), 0,
              SimulationConstants.kSimulatedMaxClimberHeight.in(Inches)));

      lastSimulationSeconds = currentTime;
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

  public double getPosition(boolean isLeftSide) {
    if (isLeftSide) {
      return leftMotor.getPosition();
    } else {
      return rightMotor.getPosition();
    }
  }

  public void setTargetPosition(double inches, Boolean isLeftSide) {
    Logger.recordOutput("Climber/" + isLeftSide + "/Setpoint/Target/Inches", inches);
    Logger.recordOutput("Climber/" + isLeftSide + "/Setpoint/Target/Pose3d",
        new Pose3d(0, Inches.of(inches).in(Meters), 0, Rotation3d.kZero));
    if (isLeftSide) { // leftmoder
      leftMotor.setTargetPosition(inches);
    } else {
      rightMotor.setTargetPosition(inches);
    }
  }

  public LinearVelocity getVelocity(boolean isLeftSide) {
    if (isLeftSide == true) {
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