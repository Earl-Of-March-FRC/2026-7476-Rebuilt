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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  public static enum ClimberSide {
    Left, Right
  }

  /**
   * Side of the tower relative to alliance driverstation
   */
  public static enum TowerSide { // Idk if we want middle
    Left, Right;

    /**
     * Gets the corresponding climber side for climbing up the current tower side.
     * 
     * @param isFacingDriverstation Whether or not the robot is facing the
     *                              driverstation
     * @return The corresponding climber side
     */
    public ClimberSide getCorrespondingClimberSide(boolean isFacingDriverstation) {
      return switch (this) {
        case Left -> isFacingDriverstation ? ClimberSide.Left : ClimberSide.Right;
        case Right -> isFacingDriverstation ? ClimberSide.Right : ClimberSide.Left;
      };
    }
  }

  // SparkMax motor implementation
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {
    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;

    // Percent flags/variables are for simulation purposes
    private boolean usingPercent = false;
    private double setpointPercent = 0;
    private Distance setpoint = Inches.of(0);
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
      setpointPercent = percent;
    }

    @Override
    public void setTargetPosition(double inches) {
      usingPercent = false;
      setpoint = Inches.of(inches);
      sparkMax.getClosedLoopController().setSetpoint(inches, SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0);
    }

    @Override
    public boolean isAtPosition() {
      return getPosition().minus(setpoint).abs(Inches) <= ClimberConstants.kPositionTolerance.in(Inches);
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public LinearVelocity getVelocity() {
      if (RobotBase.isSimulation() && sparkMaxSim != null) {
        return InchesPerSecond.of(sparkMaxSim.getVelocity());
      }
      return InchesPerSecond.of(sparkMax.getEncoder().getVelocity());
    }

    @Override
    public Distance getPosition() {
      if (RobotBase.isSimulation() && sparkMaxSim != null) {
        return Inches.of(sparkMaxSim.getPosition());
      }
      return Inches.of(sparkMax.getEncoder().getPosition());
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
        velocity = SimulationConstants.kSimulatedMaxClimberSpeed.times(setpointPercent);
      } else {
        final Distance errorPosition = setpoint.minus(getPosition());
        final LinearVelocity desiredVelocity = InchesPerSecond.of(errorPosition.in(Inches) / deltaTime);
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
    public boolean isAtPosition() {
      return Math.abs(talonFX.getClosedLoopError().getValue()) <= ClimberConstants.kPositionTolerance.in(Inches);
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public LinearVelocity getVelocity() {
      return InchesPerSecond.of(talonFX.getVelocity().getValueAsDouble());
    }

    @Override
    public Distance getPosition() {
      return Inches.of(talonFX.getPosition().getValueAsDouble());
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
      System.err.println(
          "Climber simulation with TalonFX is not supported right now. Please use the SparkMax implementation instead.");
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
        new Pose3d(0, 0, leftMotor.getPosition().in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Left/Measured/VelocityInchesPerSec", leftMotor.getVelocity());
    Logger.recordOutput("Climber/Left/Measured/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Left/Measured/CurrentAmps", leftMotor.getCurrent());
    Logger.recordOutput("Climber/Left/Measured/AtSetpoint", isAtPosition(ClimberSide.Left));

    Logger.recordOutput("Climber/Right/Measured/Position/Inches", rightMotor.getPosition());
    Logger.recordOutput("Climber/Right/Measured/Position/Pose3d",
        new Pose3d(0, 0, rightMotor.getPosition().in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Right/Measured/VelocityInchesPerSec", rightMotor.getVelocity());
    Logger.recordOutput("Climber/Right/Measured/AppliedOutput", rightMotor.getAppliedOutput());
    Logger.recordOutput("Climber/Right/Measured/CurrentAmps", rightMotor.getCurrent());
    Logger.recordOutput("Climber/Right/Measured/AtSetpoint", isAtPosition(ClimberSide.Right));
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

  public boolean isAtPosition(ClimberSide side) {
    return switch (side) {
      case Left -> leftMotor.isAtPosition();
      case Right -> rightMotor.isAtPosition();
    };
  }

  public boolean isAtPosition() {
    return leftMotor.isAtPosition() && rightMotor.isAtPosition();
  }

  public void setPercentOutput(double percent, ClimberSide side) {
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/PercentOutput", percent);
    if (side == ClimberSide.Left) {
      leftMotor.setPercentOutput(percent);
    } else {
      rightMotor.setPercentOutput(percent);
    }
  }

  public void setTargetPosition(double inches, ClimberSide side) {
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/Target/Inches", inches);
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/Target/Pose3d",
        new Pose3d(0, Inches.of(inches).in(Meters), 0, Rotation3d.kZero));
    if (side == ClimberSide.Left) {
      leftMotor.setTargetPosition(inches);
    } else {
      rightMotor.setTargetPosition(inches);
    }
  }

  public LinearVelocity getVelocity(ClimberSide side) {
    if (side == ClimberSide.Left) {
      return leftMotor.getVelocity();
    } else {
      return rightMotor.getVelocity();
    }
  }

  @Override
  public void simulationPeriodic() {
    leftMotor.simulationPeriodic();
    rightMotor.simulationPeriodic();
  }
}