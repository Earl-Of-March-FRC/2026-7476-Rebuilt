// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkMax intakeSpark;
  private final RelativeEncoder intakeEncoder;

  public IntakeSubsystem(SparkMax intakeSpark) {
    this.intakeSpark = intakeSpark;
    intakeEncoder = intakeSpark.getEncoder();
    intakeSpark.configure(IntakeConfigs.intakeConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/Measured/Velocity",
        getVelocity() / IntakeConstants.kVelocityConversionFactor);
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Intake/Setpoint/PercentVelocity", percent);
    intakeSpark.set(percent);
  }

  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void stopIntake() {
    setVelocity(0);
  }

}
