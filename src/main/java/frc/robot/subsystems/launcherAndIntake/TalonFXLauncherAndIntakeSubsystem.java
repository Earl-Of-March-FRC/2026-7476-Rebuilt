package frc.robot.subsystems.launcherAndIntake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class TalonFXLauncherAndIntakeSubsystem extends SubsystemBase implements LauncherPIDInterface {

  private final TalonFX launcherTalon;
  // Reusable control request object — avoids allocating a new one every loop
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Accumulated offset added on top of every setpoint (compensates for battery
  // sag)
  private AngularVelocity velocityOffset = RPM.of(0);

  private boolean useHighVelocities = true;

  public TalonFXLauncherAndIntakeSubsystem(TalonFX launcherTalon) {
    this.launcherTalon = launcherTalon;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = LauncherConstants.kPIDLauncherControllerP;
    config.Slot0.kI = LauncherConstants.kPIDLauncherControllerI;
    config.Slot0.kD = LauncherConstants.kPIDLauncherControllerD;
    config.CurrentLimits.StatorCurrentLimit = LauncherConstants.kSmartCurrentLimit.magnitude();
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    launcherTalon.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // TalonFX reports velocity in rotations/sec natively
    AngularVelocity measuredVelocity = RotationsPerSecond.of(
        launcherTalon.getVelocity().getValueAsDouble());

    Logger.recordOutput("Launcher/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Launcher/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffset.in(RPM));
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
    Logger.recordOutput("Launcher/Measured/StatorCurrentAmps",
        launcherTalon.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Launcher/Measured/AppliedDutyCycle",
        launcherTalon.getDutyCycle().getValueAsDouble());
  }

  /** @return Measured velocity in RPM. */
  @Override
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(launcherTalon.getVelocity().getValueAsDouble());
  }

  /** Runs the launcher at a raw percent output. Use for open-loop only. */
  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentOutput", percent);
    launcherTalon.set(percent);
  }

  /**
   * Adjusts the accumulated velocity offset by the given amount.
   *
   * @param offsetRPM Amount to add to the offset, in RPM.
   */
  public void offsetReferenceVelocity(AngularVelocity offset) {
    velocityOffset = RPM.of(velocityOffset.in(RPM) + offset.in(RPM));
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffset.in(RPM));
    Logger.recordOutput("Launcher/VelocityOffsetRadPerSec", velocityOffset.in(RadiansPerSecond));
  }

  public void setUseHighVelocities(boolean use) {
    useHighVelocities = use;
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
  }

  /**
   * Sets the closed-loop velocity setpoint, applying the accumulated offset.
   *
   * @param rpm Target velocity in RPM (before offset).
   */
  @Override
  public void setReferenceVelocity(AngularVelocity velocity) {
    AngularVelocity withOffset = RPM.of(velocity.in(RPM) + velocityOffset.in(RPM));

    Logger.recordOutput("Launcher/Setpoint/TargetRPM", velocity.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRPMWithOffset", withOffset.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRPS", withOffset.in(RotationsPerSecond));

    launcherTalon.setControl(velocityRequest.withVelocity(withOffset.in(RotationsPerSecond)));
  }

  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }

  @Override
  public void stop() {
    setVelocity(0);
  }
}