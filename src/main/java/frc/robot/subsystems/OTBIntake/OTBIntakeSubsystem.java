package frc.robot.subsystems.OTBIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OTBIntakeConstants;

/*
 * Subsystem for the Over The Bumper (OTB) intake
 */
public class OTBIntakeSubsystem extends SubsystemBase {

  private final SparkMax rollerSparkMax;

  public OTBIntakeSubsystem(SparkMax rollerSparkMax) {
    this.rollerSparkMax = rollerSparkMax;
    rollerSparkMax.configure(OTBIntakeConstants.kRollerConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    AngularVelocity measuredVelocity = RPM.of(rollerSparkMax.getEncoder().getVelocity());
    Logger.recordOutput("Intake/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Intake/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Intake/Measured/RollerAppliedOutput", rollerSparkMax.getAppliedOutput());
    Logger.recordOutput("Intake/Measured/RollerCurrentAmps", rollerSparkMax.getOutputCurrent());
  }

  /** @return Encoder velocity in RPM. */
  public double getRollerVelocity() {
    return rollerSparkMax.getEncoder().getVelocity();
  }

  /**
   * Sets the intake to a percent output.
   *
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setRollerPercent(double percent) {
    Logger.recordOutput("Intake/Setpoint/RollerPercentOutput", percent);
    rollerSparkMax.set(percent);
  }

  public void stopRoller() {
    setRollerPercent(0);
  }
}