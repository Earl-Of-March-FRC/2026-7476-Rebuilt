// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  /** Creates a new Launcher. */
  private final SparkMax launcherSpark;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherClosedLoopController;

  public LauncherSubsystem(SparkMax launcherSpark) {
    this.launcherSpark = launcherSpark;
    this.launcherEncoder = launcherSpark.getEncoder();
    this.launcherClosedLoopController = launcherSpark.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Launcher/Measured/Velocity", getVelocity());
  }

  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentVelocity", percent);
    launcherSpark.set(percent);
  }
}
