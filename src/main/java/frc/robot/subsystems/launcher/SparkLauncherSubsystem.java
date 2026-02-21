package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class SparkLauncherSubsystem extends SubsystemBase implements LauncherPIDInterface {

  private final SparkMax launcherSpark;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherClosedLoopController;

  // Accumulated offset added on top of every setpoint (compensates for battery
  // sag)
  private double velocityOffsetRPM = 0.0;
  private boolean useHighVelocities = true;

  public SparkLauncherSubsystem(SparkMax launcherSpark) {
    this.launcherSpark = launcherSpark;
    this.launcherEncoder = launcherSpark.getEncoder();
    this.launcherClosedLoopController = launcherSpark.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit((int) LauncherConstants.kSmartCurrentLimit.magnitude());
    config.closedLoop
        .p(LauncherConstants.kPIDLauncherControllerP)
        .i(LauncherConstants.kPIDLauncherControllerI)
        .d(LauncherConstants.kPIDLauncherControllerD)
        .outputRange(LauncherConstants.kOutputRangeMin, LauncherConstants.kOutputRangeMax);

    launcherSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    AngularVelocity measuredVelocity = RPM.of(launcherEncoder.getVelocity());
    Logger.recordOutput("Launcher/Measured/VelocityRPM", measuredVelocity.in(RPM));
    Logger.recordOutput("Launcher/Measured/VelocityRadPerSec", measuredVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM);
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
    Logger.recordOutput("Launcher/Measured/AppliedOutput", launcherSpark.getAppliedOutput());
    Logger.recordOutput("Launcher/Measured/CurrentAmps", launcherSpark.getOutputCurrent());
  }

  /** @return Measured velocity in RPM. */
  @Override
  public AngularVelocity getVelocity() {
    return RPM.of(launcherEncoder.getVelocity());
  }

  /** Runs the launcher at a raw percent output. Use for open-loop only. */
  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Setpoint/PercentOutput", percent);
    launcherSpark.set(percent);
  }

  /**
   * Adjusts the accumulated velocity offset by the given amount.
   * Call repeatedly to nudge the setpoint up/down during a match.
   *
   * @param offsetRPM Amount to add to the offset, in RPM.
   */
  public void offsetReferenceVelocity(AngularVelocity offset) {
    velocityOffsetRPM += offset.in(RPM);
    AngularVelocity total = RPM.of(velocityOffsetRPM);
    Logger.recordOutput("Launcher/VelocityOffsetRPM", total.in(RPM));
    Logger.recordOutput("Launcher/VelocityOffsetRadPerSec", total.in(RadiansPerSecond));
  }

  public void setUseHighVelocities(boolean use) {
    useHighVelocities = use;
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
  }

  /**
   * Sets the closed-loop velocity setpoint, applying the accumulated offset.
   *
   * @param rpm Target velocity in RPM (before offset).
   */
  @Override
  public void setReferenceVelocity(AngularVelocity velocity) {
    AngularVelocity withOffset = RPM.of(velocity.in(RPM) + velocityOffsetRPM);

    Logger.recordOutput("Launcher/Setpoint/TargetRPM", velocity.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRPMWithOffset", withOffset.in(RPM));
    Logger.recordOutput("Launcher/Setpoint/TargetRadPerSecWithOffset", withOffset.in(RadiansPerSecond));

    launcherClosedLoopController.setSetpoint(
        withOffset.in(RadiansPerSecond),
        ControlType.kVelocity,
        useHighVelocities ? LauncherConstants.kSlotHigh : LauncherConstants.kSlotLow);
  }

  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

}
