package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intakeSpark;
  private final RelativeEncoder intakeEncoder;

  public IntakeSubsystem(SparkMax intakeSpark) {
    this.intakeSpark = intakeSpark;
    this.intakeEncoder = intakeSpark.getEncoder();
    intakeSpark.configure(IntakeConfigs.intakeConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    AngularVelocity measuredVelocity = RPM.of(intakeEncoder.getVelocity());
    Logger.recordOutput("Intake/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Intake/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Intake/Measured/AppliedOutput", intakeSpark.getAppliedOutput());
    Logger.recordOutput("Intake/Measured/CurrentAmps", intakeSpark.getOutputCurrent());

  }

  /**
   * Sets the intake to a percent output.
   *
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Intake/Setpoint/PercentOutput", percent);
    intakeSpark.set(percent);
  }

  /** @return Encoder velocity in RPM. */
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void stopIntake() {
    setVelocity(0);
  }
}