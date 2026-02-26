package frc.robot.subsystems.launcherAndIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherAndIntakeConstants;

public class LauncherSubsystem extends SubsystemBase {

  // SparkMax motor implementation
  public static class SparkMaxLauncherMotor implements LauncherMotorInterface {
    private final SparkMax leaderSparkMax;
    private final SparkMax followerSparkMax;
    private final SparkClosedLoopController closedLoopController;

    public SparkMaxLauncherMotor(SparkMax leader, SparkMax follower) {
      this.leaderSparkMax = leader;
      this.followerSparkMax = follower;
      this.closedLoopController = leader.getClosedLoopController();

      leaderSparkMax.configure(LauncherAndIntakeConstants.kLeaderConfig, ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      followerSparkMax.configure(LauncherAndIntakeConstants.kFollowerConfig, ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }

    @Override
    public void setReferenceVelocity(AngularVelocity velocity) {
      closedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public AngularVelocity getVelocity() {
      return RPM.of(leaderSparkMax.getEncoder().getVelocity());
    }

    @Override
    public void stop() {
      leaderSparkMax.set(0);
    }

    @Override
    public double getAppliedOutput() {
      return leaderSparkMax.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
      return leaderSparkMax.getOutputCurrent();
    }

    public void setPercent(double percent) {
      leaderSparkMax.set(percent);
    }
  }

  // TalonFX motor implementation
  public static class TalonFXLauncherMotor implements LauncherMotorInterface {
    private final TalonFX talonFX;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public TalonFXLauncherMotor(TalonFX talonFX) {
      this.talonFX = talonFX;

      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = LauncherAndIntakeConstants.kPIDLauncherControllerP;
      config.Slot0.kI = LauncherAndIntakeConstants.kPIDLauncherControllerI;
      config.Slot0.kD = LauncherAndIntakeConstants.kPIDLauncherControllerD;
      config.CurrentLimits.StatorCurrentLimit = LauncherAndIntakeConstants.kSmartCurrentLimit.magnitude();
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      talonFX.getConfigurator().apply(config);
    }

    @Override
    public void setReferenceVelocity(AngularVelocity velocity) {
      talonFX.setControl(velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
    }

    @Override
    public AngularVelocity getVelocity() {
      return RotationsPerSecond.of(talonFX.getVelocity().getValueAsDouble());
    }

    @Override
    public void stop() {
      talonFX.set(0);
    }

    @Override
    public double getAppliedOutput() {
      return talonFX.getDutyCycle().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
      return talonFX.getStatorCurrent().getValueAsDouble();
    }

    public void setVelocityPercent(double percent) {
      talonFX.set(percent);
    }
  }

  // Main Launcher Subsystem
  private final LauncherMotorInterface motor;
  private double velocityOffsetRPM = 0.0;
  private boolean useHighVelocities = true;

  public LauncherSubsystem(LauncherMotorInterface motor) {
    this.motor = motor;
  }

  @Override
  public void periodic() {
    AngularVelocity measuredVelocity = motor.getVelocity();
    Logger.recordOutput("Launcher/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Launcher/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM);
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
    Logger.recordOutput("Launcher/Measured/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Launcher/Measured/CurrentAmps", motor.getCurrent());
  }

  public AngularVelocity getVelocity() {
    return motor.getVelocity();
  }

  public void setReferenceVelocity(AngularVelocity velocity) {
    AngularVelocity withOffset = RPM.of(velocity.in(RPM) + velocityOffsetRPM);

    Logger.recordOutput("Launcher/Setpoint/TargetRPM", velocity.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRPMWithOffset", withOffset.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRadPerSecWithOffset", withOffset.in(RadiansPerSecond));

    motor.setReferenceVelocity(withOffset);
  }

  public void stop() {
    motor.stop();
    Logger.recordOutput("Launcher/Setpoint/PercentOutput", 0.0);
  }

  public void offsetReferenceVelocity(AngularVelocity offset) {
    velocityOffsetRPM += offset.in(RPM);
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM);
    Logger.recordOutput("Launcher/VelocityOffsetRadPerSec", RPM.of(velocityOffsetRPM).in(RadiansPerSecond));
  }

  public void setUseHighVelocities(boolean use) {
    useHighVelocities = use;
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
  }

  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }
}