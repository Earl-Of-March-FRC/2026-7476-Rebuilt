package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.UnitHelpers;

public class ClimberSubsystem extends SubsystemBase {

  /**
   * Tower side relative to the driverstation
   */
  public static enum TowerSide {
    Left, Right
  }

  /**
   * Wraps the <em>leader</em> SparkMax. The follower SparkMax is passed in
   * and configured to mirror the leader via {@code SparkMaxConfig.follow()};
   * after that it is never touched in software — hardware handles it.
   */
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {

    private final SparkMax leader;
    private SparkMaxSim leaderSim;

    // Simulation bookkeeping
    private boolean usingPercent = false;
    private Distance simSetpoint = Inches.of(0);
    private double simPercentSetpoint = 0;
    private double lastSimSeconds = 0;

    /**
     * @param leader         The leading SparkMax (all sensor reads come from here).
     * @param leaderConfig   Config applied to the leader.
     * @param follower       The following SparkMax.
     * @param followerConfig Config applied to the follower — must include
     *                       {@code .follow(leaderCanId, invert)}.
     */
    public SparkMaxClimberMotor(
        SparkMax leader, SparkMaxConfig leaderConfig,
        SparkMax follower, SparkMaxConfig followerConfig) {

      this.leader = leader;

      leader.configure(leaderConfig,
          ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      follower.configure(followerConfig,
          ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      if (RobotBase.isSimulation()) {
        leaderSim = new SparkMaxSim(
            leader, SimulationConstants.kSimulatedSparkMaxClimberMotor);
      }
    }

    @Override
    public void setPercentOutput(double percent) {
      leader.set(percent);
      usingPercent = true;
      simPercentSetpoint = percent;
    }

    @Override
    public void setTargetPosition(Distance position) {
      usingPercent = false;
      simSetpoint = position;
      leader.getClosedLoopController()
          .setSetpoint(position.in(Inches), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public boolean isAtPosition() {
      return getPosition()
          .minus(RobotBase.isReal() ? Inches.of(leader.getClosedLoopController().getSetpoint()) : simSetpoint)
          .abs(Inches) <= ClimberConstants.kPositionTolerance.in(Inches);
    }

    @Override
    public void stop() {
      setPercentOutput(0);
    }

    @Override
    public Distance getPosition() {
      double raw = RobotBase.isSimulation() && leaderSim != null
          ? leaderSim.getPosition()
          : leader.getEncoder().getPosition();
      return Inches.of(raw);
    }

    @Override
    public LinearVelocity getVelocity() {
      double raw = RobotBase.isSimulation() && leaderSim != null
          ? leaderSim.getVelocity()
          : leader.getEncoder().getVelocity();
      return InchesPerSecond.of(raw);
    }

    @Override
    public double getAppliedOutput() {
      return leader.getAppliedOutput();
    }

    @Override
    public double getCurrent() {
      return leader.getOutputCurrent();
    }

    @Override
    public void resetEncoder() {
      leader.getEncoder().setPosition(0);
      if (leaderSim != null) {
        leaderSim.setPosition(0);
      }
    }

    @Override
    public void simulationPeriodic() {
      if (leaderSim == null)
        return;

      final double now = Timer.getFPGATimestamp();
      double dt = now - lastSimSeconds;
      if (dt <= 0)
        dt = 0.02;
      lastSimSeconds = now;

      leaderSim.setBusVoltage(RobotController.getBatteryVoltage());

      final LinearVelocity maxSpeed = SimulationConstants.kSimulatedMaxClimberSpeed;

      LinearVelocity velocity;
      if (usingPercent) {
        velocity = maxSpeed.times(simPercentSetpoint).times(ClimberConstants.kOutputUp);
      } else {
        Distance error = simSetpoint.minus(getPosition());
        // Proportional approximation: Drive toward setpoint clamped to max speed
        double desiredInchesPerSec = error.in(Inches) / dt;
        double clampedInchesPerSec = MathUtil.clamp(
            desiredInchesPerSec,
            -maxSpeed.in(InchesPerSecond),
            maxSpeed.in(InchesPerSecond));
        velocity = InchesPerSecond.of(clampedInchesPerSec);
      }

      leaderSim.setVelocity(velocity.in(InchesPerSecond));

      Distance newPos = getPosition().plus(Inches.of(velocity.in(InchesPerSecond) * dt));
      Distance clampedPos = (Distance) UnitHelpers.clamp(newPos, ClimberConstants.kMinLength,
          SimulationConstants.kSimulatedMaxClimberHeight);
      leaderSim.setPosition(clampedPos.in(Inches));
    }
  }

  // Single motor wrapper — the follower is wired in hardware via config.
  private final ClimberMotorInterface motor;

  // Normally-open limit switch; closes (reads false) when at the bottom.
  private final DigitalInput leftBottomLimitSwitch;
  private final DigitalInput rightBottomLimitSwitch;

  // Last PID setpoint issued, used for atSetpoint() calculation.
  private Distance lastSetpoint = Inches.of(0);

  /**
   * @param motor             The leader motor wrapper (follower already
   *                          configured inside via SparkMaxConfig.follow()).
   * @param bottomLimitSwitch DIO-wired limit switch at the bottom of travel.
   */
  public ClimberSubsystem(ClimberMotorInterface motor, DigitalInput leftBottomLimitSwitch,
      DigitalInput rightBottomLimitSwitch) {
    this.motor = motor;
    this.leftBottomLimitSwitch = leftBottomLimitSwitch;
    this.rightBottomLimitSwitch = rightBottomLimitSwitch;
  }

  /**
   * Commands the climber to a position via onboard PID.
   * The target is clamped to {@code [kMinLength, kMaxLength]}.
   */
  public void setTargetPosition(Distance position) {
    Distance clamped = Inches.of(MathUtil.clamp(
        position.in(Inches),
        ClimberConstants.kMinLength.in(Inches),
        ClimberConstants.kMaxLength.in(Inches)));

    lastSetpoint = clamped;
    motor.setTargetPosition(clamped);

    Logger.recordOutput("Climber/Setpoint/Inches", clamped.in(Inches));
    Logger.recordOutput("Climber/Setpoint/Meters", clamped.in(Meters));
  }

  // Drives the climber at open-loop percent output [-1, 1].
  public void setPercentOutput(double percent) {
    motor.setPercentOutput(percent);
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
  }

  // Stops both motors immediately.
  public void stop() {
    motor.stop();
  }

  /**
   * Zeroes the encoder. Call this when the bottom limit switch trips so
   * future position commands are accurate.
   */
  public void resetEncoder() {
    motor.resetEncoder();
    lastSetpoint = Inches.of(0);
    Logger.recordOutput("Climber/EncoderReset", true);
  }

  /**
   * Check if the left bottom limit switch was triggered
   * 
   * @return {@code true} when the bottom limit switch is triggered.
   */
  public boolean isLeftAtBottom() {
    return limitSwitchFlaggedAtBottom(leftBottomLimitSwitch);
  }

  /**
   * Check if the right bottom limit switch was triggered
   * 
   * @return {@code true} when the bottom limit switch is triggered.
   */
  public boolean isRightAtBottom() {
    return limitSwitchFlaggedAtBottom(rightBottomLimitSwitch);
  }

  /**
   * Check if either bottom limit switches was triggered
   * 
   * @return {@code true} when either the bottom limit switch is triggered.
   */
  public boolean isEitherAtBottom() {
    return isLeftAtBottom() || isRightAtBottom();
  }

  private boolean limitSwitchFlaggedAtBottom(DigitalInput limitSwitch) {
    // DigitalInput reads false when the circuit is closed (active-low wiring).
    // However, if the switch is active-high, remove the negation.
    boolean triggered;
    if (RobotBase.isReal()) {
      triggered = !limitSwitch.get();
    } else {
      triggered = getPosition().isNear(Inches.of(0), SimulationConstants.kSimulatedClimberBottomTolerance);
    }
    return triggered;
  }

  // @return Current position as a {@link Distance}.
  public Distance getPosition() {
    return motor.getPosition();
  }

  // @return Current velocity as a {@link LinearVelocity}.
  public LinearVelocity getVelocity() {
    return motor.getVelocity();
  }

  /**
   * @return {@code true} when the encoder position is within
   *         {@link ClimberConstants#kPIDPositionTolerance} of the last setpoint.
   */
  public boolean atSetpoint() {
    Distance error = motor.getPosition().minus(lastSetpoint);
    return Math.abs(error.in(Inches)) < ClimberConstants.kPIDPositionTolerance.in(Inches);
  }

  @Override
  public void periodic() {
    Distance position = motor.getPosition();
    LinearVelocity velocity = motor.getVelocity();

    Logger.recordOutput("Climber/Measured/PositionInches", position.in(Inches));
    Logger.recordOutput("Climber/Measured/Pose3d",
        new Pose3d(0, 0, position.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/VelocityInchesPerSec", velocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/CurrentAmps", motor.getCurrent());
    Logger.recordOutput("Climber/AtSetpoint", atSetpoint());
    Logger.recordOutput("Climber/Measured/LimitSwitch/LeftAtBottom", isLeftAtBottom());
    Logger.recordOutput("Climber/Measured/LimitSwitch/RightAtBottom", isRightAtBottom());

    // Safety net: if the limit switch trips while descending, stop and zero.
    if ((isLeftAtBottom() || isRightAtBottom()) && velocity.in(InchesPerSecond) < 0) {
      motor.stop();
      motor.resetEncoder();
    }
  }

  @Override
  public void simulationPeriodic() {
    motor.simulationPeriodic();
  }
}