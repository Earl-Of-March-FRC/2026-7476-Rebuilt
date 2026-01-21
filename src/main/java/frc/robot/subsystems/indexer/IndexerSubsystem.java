// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerConfigs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax indexerSpark;
  private final RelativeEncoder encoder;

  /** Creates a new Indexer. */
  public IndexerSubsystem(SparkMax indexerSpark) {
    this.indexerSpark = indexerSpark;

    indexerSpark.configure(IndexerConfigs.indexerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    encoder = indexerSpark.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity(double percent) {
    Logger.recordOutput("Indexer/Setpoint/PercentVelocity", percent);
    indexerSpark.set(percent * IndexerConstants.kDirectionConstant);
  };

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setVoltage(double voltage) {
    Logger.recordOutput("Indexer/Setpoint/Voltage", voltage);
    indexerSpark.setVoltage(voltage * IndexerConstants.kDirectionConstant);
  };

  public double getVoltage() {
    return indexerSpark.getAppliedOutput();
  }
}
