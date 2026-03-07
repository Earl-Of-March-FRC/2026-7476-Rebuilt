package frc.robot.subsystems.launcherAndIntake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;

public class LauncherAndIntakeSubsystem extends SubsystemBase {

  // SparkMax motor implementation
  public static class SparkMaxLauncherAndIntakeMotor implements LauncherAndIntakeMotorInterface {
    private final SparkMax leaderSparkMax;
    private final SparkMax followerSparkMax;
    private final SparkClosedLoopController closedLoopController;

    public SparkMaxLauncherAndIntakeMotor(SparkMax leader, SparkMax follower) {
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

    @Override
    public void setPercent(double percent) {
      leaderSparkMax.set(percent);
    }
  }

  // TalonFX motor implementation
  public static class TalonFXLauncherAndIntakeMotor implements LauncherAndIntakeMotorInterface {
    private final TalonFX talonFX;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public TalonFXLauncherAndIntakeMotor(TalonFX talonFX) {
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

    @Override
    public void setPercent(double percent) {
      talonFX.set(percent);
    }
  }

  // Sim spark max motor implementation
  // Models the velocity as changing linearly with applied ouput
  public static class SimSparkMaxLauncherAndIntakeMotor implements LauncherAndIntakeMotorInterface {
    private final SparkMaxSim sparkMaxSim;
    private final AngularVelocity maxRPM;

    public SimSparkMaxLauncherAndIntakeMotor(SparkMax sparkMax, DCMotor motorModel, AngularVelocity maxRPM) {
      sparkMax.configure(LauncherAndIntakeConstants.kLeaderConfig, ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

      this.maxRPM = maxRPM;
      this.sparkMaxSim = new SparkMaxSim(sparkMax, motorModel);
      sparkMaxSim.enable();
    }

    @Override
    public void setReferenceVelocity(AngularVelocity velocity) {
      sparkMaxSim.setVelocity(velocity.in(RPM));
      sparkMaxSim.setAppliedOutput(velocity.div(maxRPM).magnitude());
    }

    @Override
    public AngularVelocity getVelocity() {
      return RPM.of(sparkMaxSim.getVelocity());
    }

    @Override
    public void stop() {
      setPercent(0);
    }

    @Override
    public double getAppliedOutput() {
      return sparkMaxSim.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
      return sparkMaxSim.getMotorCurrent();
    }

    @Override
    public void setPercent(double percent) {
      sparkMaxSim.setAppliedOutput(percent);
      sparkMaxSim.setVelocity(maxRPM.times(percent).in(RPM));
    }
  }

  // Main Launcher Subsystem
  private final LauncherAndIntakeMotorInterface motor;
  private double velocityOffsetRPM = 0.0;
  private boolean useHighVelocities = true;

  public LauncherAndIntakeSubsystem(LauncherAndIntakeMotorInterface motor) {
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
    Logger.recordOutput("Launcher/Prediction/BallAirTimeSeconds",
        LaunchHelpers.calculateBallAirTime(PoseHelpers.getAllianceHubtTranslation3d().getMeasureZ()));
    Logger.recordOutput("Launcher/Prediction/PredictedBallEndpointHubHeight",
        LaunchHelpers.predictBallEndpoint(PoseHelpers.getAllianceHubtTranslation3d().getMeasureZ()));
    Logger.recordOutput("Launcher/Prediction/BallLaunchVelocityMPS",
        LaunchHelpers.calculateBallLaunchVelocityVector());
    Logger.recordOutput("Launcher/Prediction/BallInitialVelocityMPS",
        LaunchHelpers.calculateBallResultantVelocityVector());
    Logger.recordOutput("Launcher/Prediction/WillHitHub",
        LaunchHelpers.willHitHub());
  }

  public AngularVelocity getVelocity() {
    return motor.getVelocity();
  }

  public void setReferenceVelocity(AngularVelocity velocity) {
    AngularVelocity withOffset = RPM.of(velocity.in(RPM) + velocityOffsetRPM);

    Logger.recordOutput("Launcher/Setpoint/TargetRPM", velocity.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/VelocityOffsetRPM", velocityOffsetRPM);
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