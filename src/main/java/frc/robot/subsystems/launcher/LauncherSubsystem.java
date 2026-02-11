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

public class LauncherSubsystem extends SubsystemBase {
  /** Creates a new Launcher. */
  private final SparkMax launcherSpark;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherClosedLoopController;
  private final SparkMaxConfig launcherSparkMaxConfig = new SparkMaxConfig();

  // TODO: learn what this means because I just copied this from the last year's
  // launcher code
  private double referenceVelocityWithoutOffset = 0.0;

  private double velocityOffsetRPM = 0.0;

  private boolean useHighVelocities = true;

  public LauncherSubsystem(SparkMax launcherSpark) {
    this.launcherSpark = launcherSpark;
    this.launcherEncoder = launcherSpark.getEncoder();
    this.launcherClosedLoopController = launcherSpark.getClosedLoopController();

    launcherSparkMaxConfig.smartCurrentLimit(40);

    launcherSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.kTicksToInchesConversion);

    launcherSparkMaxConfig.closedLoop
        .p(ClimberConstants.kP)
        .i(ClimberConstants.kI)
        .d(ClimberConstants.kD)
        .outputRange(-1, 1); // -1, 1 as in -1 full speed backwards, and 1 full speed forwards (Motor maximum
                             // speeds)

    launcherSpark.configure(launcherSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("LauncherLowVelocity",
        LauncherConstants.kVelocityLowRPM * LauncherConstants.kVelocityConversionFactor);

  }

  @Override
  public void periodic() { // Will be used for logging for now.
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vel", launcherEncoder.getVelocity());
  }

  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentVelocity", percent);
    launcherSpark.set(percent);
  }

  public void setReferenceVelocityOffset(double offsetRPM) {
    // Yes interface what will it be called
    // It will be LauncherPIDCmd.java, and it implements whatever. You go research
    // it and shi
    // IT's gonna be in the launcher, II will commit the code
    velocityOffsetRPM = offsetRPM;
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
  public void setReferenceVelocity(double referenceVelocity) {
    double referenceVelocityWithOffset = referenceVelocity;
    Logger.recordOutput("Launcher/Setpoint/Velocity", referenceVelocity);
    Logger.recordOutput("Launcher/Setpoint/VelocityWithOffset", referenceVelocityWithOffset);

    // Converts RPM to radians per second
    launcherClosedLoopController.setSetpoint(
        (referenceVelocity == 0 ? referenceVelocity : referenceVelocityWithOffset)
            * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity, useHighVelocities ? LauncherConstants.kSlotHigh : LauncherConstants.kSlotLow);
  }

  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }

}
