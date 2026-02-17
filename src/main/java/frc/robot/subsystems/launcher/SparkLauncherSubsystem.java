// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

//import frc.robot.Configs.LauncherConfigs;

import frc.robot.Constants.LauncherConstants;

// LauncherSubsystem is a subsystem that represents the flywheel motor, which spins and launches out balls ("Fuel")

// Structurally, this is similar to SpongeBot's code in Launcher.java on main branch.
// It does not have a front or back motor, it's a single motor.
public class SparkLauncherSubsystem extends SubsystemBase implements LauncherPIDInterface {
  private final SparkMax launcherSpark;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherClosedLoopController;
  private final SparkMaxConfig launcherSparkMaxConfig = new SparkMaxConfig();

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

  public SparkLauncherSubsystem(SparkMax launcherSpark) {

    // Connect with the hardware
    this.launcherSpark = launcherSpark;
    this.launcherEncoder = launcherSpark.getEncoder();
    this.launcherClosedLoopController = launcherSpark.getClosedLoopController();

    launcherSparkMaxConfig.smartCurrentLimit(LauncherConstants.kLauncherSmartCurrentLimit); // Limit the amps
    launcherSparkMaxConfig.closedLoop
        .p(LauncherConstants.kLauncherP) // Returns the config with the p modification
        .i(LauncherConstants.kLauncherI) // Returns the config with the i mod.
        .d(LauncherConstants.kLauncherD) // Yeah.
        .outputRange(-1, 1); // So that's why we're able to chain together stuff

    launcherSpark
        .configure(launcherSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("LauncherLowVelocity",
        LauncherConstants.kVelocityLowRPM * LauncherConstants.kVelocityConversionFactor);

  }

  @Override
  public void periodic() { // Will be used for logging for now.
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vel", launcherEncoder.getVelocity());
  }

  @Override
  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentVelocity", percent);
    launcherSpark.set(percent);
  }

  public void offsetReferenceVelocity(double offsetRPM) {
    // Yes interface what will it be called
    // It will be LauncherPIDCmd.java, and it implements whatever. You go research
    // it and shi
    // IT's gonna be in the launcher, II will commit the code
    velocityOffsetRPM += offsetRPM;
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM);
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
    Logger.recordOutput("Launcher/Setpoint/Velocity", rpm);

    // Converts RPM to radians per second
    launcherClosedLoopController.setSetpoint(
        rpm * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity, useHighVelocities ? LauncherConstants.kSlotHigh : LauncherConstants.kSlotLow);
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
