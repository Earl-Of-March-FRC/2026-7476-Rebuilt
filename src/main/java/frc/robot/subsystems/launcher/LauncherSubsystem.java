// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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

    SmartDashboard.putNumber("LauncherLowFrontVelocity",
        LauncherConstants.kVelocityLowFrontRPM * LauncherConstants.kVelocityConversionFactor);

  }

  @Override
  public void periodic() { // Will be used for logging for now.
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FrontVel", launcherEncoder.getVelocity());
  }

  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentVelocity", percent);
    launcherSpark.set(percent);
  }

  public void setReferenceVelocity(double referenceVelocity) {
    setReferenceVelocity(referenceVelocity, referenceVelocity);
  }

}
