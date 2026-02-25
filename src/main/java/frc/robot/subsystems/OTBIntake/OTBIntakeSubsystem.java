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
 * Subsystem for the Over The Bumper (OTB) intake, which consists of a shoulder joint and a roller.
 * This subsystem funnels balls into the internal intake when deployed.
 */
public class OTBIntakeSubsystem extends SubsystemBase {

  private final SparkMax shoulderSparkMax;
  private final SparkMax rollerSparkMax;

  private Angle angularOffset = Degrees.of(0);
  private Angle pidSetpointWithoutOffset = OTBIntakeConstants.kStowPosition;
  private boolean usePID = true;

  public OTBIntakeSubsystem(SparkMax shoulderSparkMax, SparkMax rollerSparkMax) {
    this.shoulderSparkMax = shoulderSparkMax;
    this.rollerSparkMax = rollerSparkMax;

    shoulderSparkMax.configure(OTBIntakeConstants.kShoulderConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    rollerSparkMax.configure(OTBIntakeConstants.kRollerConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    if (usePID) {
      // Gravity compensation feedforward is calculated automatically
      REVLibError isOk = shoulderSparkMax.getClosedLoopController().setSetpoint(
          getShoulderSetpoint().in(Degrees),
          ControlType.kPosition);
      Logger.recordOutput("Intake/Setpoint/Status", isOk);
      Logger.recordOutput("Intake/Setpoint/ShoulderSetpointDegrees", getShoulderSetpoint().in(Degrees));
    }

    AngularVelocity measuredVelocity = RPM.of(rollerSparkMax.getEncoder().getVelocity());
    Logger.recordOutput("Intake/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Intake/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Intake/Measured/RollerAppliedOutput", rollerSparkMax.getAppliedOutput());
    Logger.recordOutput("Intake/Measured/RollerCurrentAmps", rollerSparkMax.getOutputCurrent());

    Logger.recordOutput("Intake/Measured/ShoulderPositionDegrees",
        shoulderSparkMax.getEncoder().getPosition());
    Logger.recordOutput("Intake/Measured/ShoulderAppliedOutput", shoulderSparkMax.getAppliedOutput());
    Logger.recordOutput("Intake/Measured/ShoulderCurrentAmps", shoulderSparkMax.getOutputCurrent());

  }

  /** @return Encoder velocity in RPM. */
  public double getRollerVelocity() {
    return rollerSparkMax.getEncoder().getVelocity();
  }

  /** @return Shoulder position as an angle. */
  public Angle getShoulderPosition() {
    return Degrees.of(shoulderSparkMax.getEncoder().getPosition());
  }

  /** @return Shoulder angle setpoint for the PID controller. */
  public Angle getShoulderSetpoint() {
    return pidSetpointWithoutOffset.plus(angularOffset);
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

  /**
   * Sets the shoulder to a percent output.
   * 
   * <strong>This method will ONLY start working after given a non zero
   * input, which then manual mode will be enabled.</strong> After enabling
   * manual mode, an input of zero will command the arm to run at 0% velocity
   * (stopping it).
   * See {@link #setShoulderPercent(double, boolean)} to force manual mode even if
   * the
   * input is zero.
   * Use {@link #setShoulderSetpoint(Angle)} to re-enable PID control.
   * 
   * @param percent Output in range [-1.0, 1.0].
   */
  public void setShoulderPercent(double percent) {
    if (percent != 0) {
      usePID = false;
    }
    if (usePID) {
      return;
    }

    Logger.recordOutput("Intake/Setpoint/ShoulderPercentOutput", percent);
    shoulderSparkMax.set(percent);
  }

  /**
   * Sets the shoulder to a perrcent output, with an option to force manual
   * control
   * 
   * @param percent     Output in range [-1.0, 1.0].
   * @param forceManual Whether or not to disable the PID controller
   */
  public void setShoulderPercent(double percent, boolean forceManual) {
    if (forceManual) {
      usePID = false;
    }

    setShoulderPercent(percent);
  }

  /**
   * Sets the angle setpoint of the shoulder closed loop controller.
   * 
   * @param position Desired shoulder position as an angle.
   */
  public void setShoulderSetpoint(Angle position) {
    usePID = true;
    pidSetpointWithoutOffset = position;

    Logger.recordOutput("Intake/Setpoint/ShoulderPositionDegrees", position.in(Degrees));
    shoulderSparkMax.getClosedLoopController().setSetpoint(position.in(Degrees) + angularOffset.in(Degrees),
        ControlType.kPosition);
  }

  public void stopRoller() {
    setRollerPercent(0);
  }

  /**
   * Stops the shoulder motor using percent output.
   */
  public void stopShoulder() {
    setShoulderPercent(0, true);
  }

  /**
   * Resets the arm encoder to 0.
   */
  public void resetShoulderPosition() {
    shoulderSparkMax.getEncoder().setPosition(0);
  }

  /**
   * Calibrates the shoulder encoder to the starting angle.
   */
  public void calibrateShoulderPosition() {
    shoulderSparkMax.getEncoder().setPosition(OTBIntakeConstants.kStowPosition.in(Degrees));
  }

  /**
   * Returns whether the arm is using PID.
   * 
   * @return boolean Whether the arm is using PID.
   */
  public boolean getIsUsingPid() {
    return usePID;
  }

  /**
   * Returns the angular offset of the shoulder.
   * 
   * @return The angular offset of the shoulder.
   */
  public Angle getAngularOffset() {
    return angularOffset;
  }

  /**
   * Resets the angular offset of the shoulder to 0 degrees.
   */
  public void clearAngularOffset() {
    setAngularOffset(Degrees.of(0));
  }

  /**
   * Sets the angular offset of the shoulder.
   * 
   * @param newOffset The angular offset of the shoulder.
   */
  public void setAngularOffset(Angle newOffset) {
    angularOffset = newOffset;
  }

  /**
   * Increases the angular offset of the shoulder. Use a negative value to
   * decrease the
   * offset.
   * 
   * @param offsetIncrement The increment to the angular offset of the shoulder.
   *                        Can be negative.
   */
  public void increaseAngularOffset(Angle offsetIncrement) {
    angularOffset = angularOffset.plus(offsetIncrement);
  }

  /**
   * Increases the angular offset of the shoulder. Use a negative value to
   * decrease the
   * offset.
   * 
   * @param offsetDeg The angular offset of the shoulder, in degrees. Can be
   *                  negative.
   */
  public void increaseAngularOffset(double offsetDeg) {
    angularOffset = angularOffset.plus(Degrees.of(offsetDeg));
  }

}