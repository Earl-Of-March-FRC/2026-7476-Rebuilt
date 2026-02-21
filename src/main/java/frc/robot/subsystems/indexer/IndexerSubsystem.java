package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerConfigs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax indexerSpark;
  private final RelativeEncoder encoder;

  public IndexerSubsystem(SparkMax indexerSpark) {
    this.indexerSpark = indexerSpark;
    this.encoder = indexerSpark.getEncoder();
    indexerSpark.configure(IndexerConfigs.indexerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    AngularVelocity measuredVelocity = RPM.of(encoder.getVelocity());
    Logger.recordOutput("Indexer/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Indexer/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Indexer/Measured/AppliedOutput", indexerSpark.getAppliedOutput());
    Logger.recordOutput("Indexer/Measured/CurrentAmps", indexerSpark.getOutputCurrent());
  }

  /**
   * Sets the indexer to a percent output, applying the direction constant.
   *
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Indexer/Setpoint/PercentOutput", percent);
    indexerSpark.set(percent * IndexerConstants.kDirectionConstant);
  }

  /** @return Encoder velocity in RPM. */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Sets the indexer voltage, applying the direction constant.
   *
   * @param volts Target voltage.
   */
  public void setVoltage(double volts) {
    Voltage voltage = Volts.of(volts);
    Logger.recordOutput("Indexer/Setpoint/Volts", voltage.in(Volts));
    indexerSpark.setVoltage(voltage.in(Volts) * IndexerConstants.kDirectionConstant);
  }

  /** @return Applied output as a fraction of bus voltage. */
  public double getAppliedOutput() {
    return indexerSpark.getAppliedOutput();
  }

  public void stop() {
    setVelocity(0);
  }
}