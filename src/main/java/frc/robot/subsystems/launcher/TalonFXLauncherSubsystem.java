// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.Configs.LauncherConfigs;

import frc.robot.Constants.LauncherConstants;

// LauncherSubsystem is a subsystem that represents the flywheel motor, which spins and launches out balls ("Fuel")

// Structurally, this is similar to SpongeBot's code in Launcher.java on main branch.
// It does not have a front or back motor, it's a single motor.
public class TalonFXLauncherSubsystem extends SubsystemBase implements LauncherPIDInterface {
  private final TalonFX launcherTalon;

  // Control requests are "reusable" objects to save memory
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final TalonFXConfiguration launcherSparkMaxConfig = new TalonFXConfiguration();

  // TODO: learn what this means because I just copied this from the last year's
  // launcher code

  // Raw baseline for what should be the velocity.
  private double referenceVelocityWithoutOffset = 0.0;

  // In case battery declines, this number will be added to accomodate the drop in
  // distance.
  private double velocityOffsetRPM = 0.0;

  // Used to toggle kSlotHigh and kSlotLow (these are high and low constants for
  // PID)
  private boolean useHighVelocities = true;

  public TalonFXLauncherSubsystem(TalonFX launcherTalon) {

    // Connect with the hardware
    this.launcherTalon = launcherTalon;
    TalonFXConfiguration launcherTalonConfig = new TalonFXConfiguration();
    launcherTalonConfig.Slot0.kP = LauncherConstants.kLauncherP; // Realistic starting P for TalonFX
    launcherTalonConfig.Slot0.kI = LauncherConstants.kLauncherI;
    launcherTalonConfig.Slot0.kD = LauncherConstants.kLauncherD;

    launcherTalonConfig.CurrentLimits.StatorCurrentLimit = 40;
    launcherTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    SmartDashboard.putNumber("LauncherLowVelocity",
        LauncherConstants.kVelocityLowRPM * LauncherConstants.kVelocityConversionFactor);

  }

  @Override
  public double getVelocity() {
    return launcherTalon.getVelocity().getValueAsDouble() * 60.0;
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentVelocity", percent);
    launcherTalon.set(percent);
  }

  public void offsetReferenceVelocity(double offsetRPM) {
    setReferenceVelocity(velocityOffsetRPM + offsetRPM);
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM + offsetRPM);
    Logger.recordOutput("Launcher/VelocityOffsetRadPerSec",
        velocityOffsetRPM * LauncherConstants.kVelocityConversionFactor);
  }

  public void setUseHighVelocities(boolean use) {
    useHighVelocities = use;
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
  }

  /**
   * Sets the reference velocity for the launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */

  @Override
  public void setReferenceVelocity(double rpm) {
    double referenceVelocityWithOffset = rpm;
    Logger.recordOutput("Launcher/Setpoint/Velocity", rpm);
    Logger.recordOutput("Launcher/Setpoint/VelocityWithOffset", referenceVelocityWithOffset);

    double rps = rpm / 60.0;

    // We send a "Control Request" to the motor
    launcherTalon.setControl(velocityRequest.withVelocity(rps));
  }

  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }

  @Override
  public void stop() {
    setVelocity(0);
    // important
  }

}
