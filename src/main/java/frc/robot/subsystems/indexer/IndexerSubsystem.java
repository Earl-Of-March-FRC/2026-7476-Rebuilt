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
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax wheelSparkMax;
  private final SparkMax treadmillSparkMax;

  public IndexerSubsystem(SparkMax wheelSparkMax, SparkMax treadmillSparkMax) {
    this.wheelSparkMax = wheelSparkMax;
    this.treadmillSparkMax = treadmillSparkMax;

    wheelSparkMax.configure(IndexerConstants.kWheelConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    treadmillSparkMax.configure(IndexerConstants.kTreadmillConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    AngularVelocity wheelVelocity = RPM.of(wheelSparkMax.getEncoder().getVelocity());
    Logger.recordOutput("Indexer/Measured/WheelVelocityRPM", wheelVelocity.in(RPM));
    Logger.recordOutput("Indexer/Measured/WheelVelocityRadPerSec", wheelVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Indexer/Measured/WheelOutput", wheelSparkMax.getAppliedOutput());
    Logger.recordOutput("Indexer/Measured/WheelCurrentAmps", wheelSparkMax.getOutputCurrent());
  }

  /**
   * Sets the indexer wheel to a percent output, applying the direction constant.
   *
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setWheelPercent(double percent) {
    Logger.recordOutput("Indexer/Setpoint/PercentOutput", percent);
    wheelSparkMax.set(percent * IndexerConstants.kDirectionConstant);
  }

  /**
   * Sets the indexer treadmill to a percent output, applying the direction
   * constant.
   *
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setTreadmillPercent(double percent) {
    Logger.recordOutput("Indexer/Setpoint/PercentOutput", percent);
    treadmillSparkMax.set(percent * IndexerConstants.kDirectionConstant);
  }

  /** @return Wheel velocity in RPM. */
  public double getWheelVelocity() {
    return wheelSparkMax.getEncoder().getVelocity();
  }

  /** @return Treadmill velocity in MPS. */
  public double getTreadmillVelocity() {
    return treadmillSparkMax.getEncoder().getVelocity();
  }

  /**
   * Sets the indexer voltage, applying the direction constant.
   *
   * @param volts Target voltage.
   */
  public void setWheelVoltage(double volts) {
    Voltage voltage = Volts.of(volts);
    Logger.recordOutput("Indexer/Setpoint/Volts", voltage.in(Volts));
    wheelSparkMax.setVoltage(voltage.in(Volts) * IndexerConstants.kDirectionConstant);
  }

  /**
   * Sets the indexer voltage, applying the direction constant.
   *
   * @param volts Target voltage.
   */
  public void setTreadmillVoltage(double volts) {
    Voltage voltage = Volts.of(volts);
    Logger.recordOutput("Indexer/Setpoint/Volts", voltage.in(Volts));
    treadmillSparkMax.setVoltage(voltage.in(Volts) * IndexerConstants.kDirectionConstant);
  }

  /** @return Applied output as a fraction of bus voltage. */
  public double getWheelAppliedOutput() {
    return wheelSparkMax.getAppliedOutput() / wheelSparkMax.getBusVoltage();
  }

  /** @return Applied output as a fraction of bus voltage. */
  public double getTreadmillAppliedOutput() {
    return treadmillSparkMax.getAppliedOutput() / treadmillSparkMax.getBusVoltage();
  }

  public void stop() {
    setWheelPercent(0);
    setTreadmillPercent(0);
  }
}