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

/**
 * Controls the dual-arm climber mechanism.
 *
 * Both motors receive identical commands (software-mirrored), but each motor
 * maintains its own encoder. Each arm's encoder is independently zeroed when
 * its corresponding bottom limit switch triggers, eliminating mechanical desync
 * between the two arms over time.
 *
 * Position reads are exposed both per-arm and as an average so that commands
 * can choose the level of detail they need.
 */
public class ClimberSubsystem extends SubsystemBase {

  /** Identifies which side of the climb tower the robot is targeting. */
  public static enum TowerSide {
    Left,
    Right
  }

  /**
   * *
   * The two motors are configured identically (same PID, same conversion factors)
   * but driven independently. The follower is inverted in its config
   * so that the same signed command drives both arms in the correct direction.
   * Each motor has its own encoder and can be zeroed independently.
   */
  public static class SparkMaxClimberMotor implements ClimberMotorInterface {

    private final SparkMax leader;
    private final SparkMax follower;

    /* Simulation wrappers, null on real hardware. */
    private SparkMaxSim leaderSim;
    private SparkMaxSim followerSim;

    /* True when the last command was percent output rather than a PID setpoint. */
    private boolean usingPercent = false;

    /* Last PID position setpoint issued, used in simulation only. */
    private Distance simSetpoint = Inches.of(0);

    /* Last percent output issued, used in simulation only. */
    private double simPercentSetpoint = 0;

    /* FPGA timestamp of the previous simulationPeriodic() call. */
    private double lastSimSeconds = 0;

    @Override
    public void stopLeft() {
      leader.stopMotor();
    }

    @Override
    public void stopRight() {
      follower.stopMotor();
    }

    /**
     * Constructs the motor wrapper and applies configuration to both controllers.
     *
     * @param leader         The SparkMax whose encoder is the <em>left arm</em>
     *                       reference.
     * @param leaderConfig   Config for the leader
     * @param follower       The SparkMax for the right arm.
     * @param followerConfig Config for the follower.
     */
    public SparkMaxClimberMotor(
        SparkMax leader, SparkMaxConfig leaderConfig,
        SparkMax follower, SparkMaxConfig followerConfig) {

      this.leader = leader;
      this.follower = follower;

      leader.configure(leaderConfig,
          ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      follower.configure(followerConfig,
          ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      if (RobotBase.isSimulation()) {
        leaderSim = new SparkMaxSim(leader, SimulationConstants.kSimulatedSparkMaxClimberMotor);
        followerSim = new SparkMaxSim(follower, SimulationConstants.kSimulatedSparkMaxClimberMotor);
      }
    }

    /**
     * Drives both motors at the same percent output.
     * Inversion is handled by the follower's config, not here.
     *
     * @param percent Output fraction in [-1, 1].
     */
    @Override
    public void setPercentOutput(double percent) {
      leader.set(-percent);
      follower.set(-percent);
      usingPercent = true;
      simPercentSetpoint = percent;
    }

    /**
     * Commands both motors to the same position setpoint via their individual
     * onboard PID controllers.
     *
     * @param position Desired arm extension.
     */
    @Override
    public void setTargetPosition(Distance position) {
      usingPercent = false;
      simSetpoint = position;
      leader.getClosedLoopController()
          .setSetpoint(position.in(Inches), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
      follower.getClosedLoopController()
          .setSetpoint(position.in(Inches), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /** Stops both motors immediately by commanding zero percent output. */
    @Override
    public void stop() {
      leader.set(0);
      follower.set(0);
      usingPercent = true;
      simPercentSetpoint = 0;
    }

    /**
     * Returns the current position of the left (leader) arm.
     *
     * @return Left arm extension.
     */
    @Override
    public Distance getLeftPosition() {
      double raw = RobotBase.isSimulation() && leaderSim != null
          ? leaderSim.getPosition()
          : leader.getEncoder().getPosition();
      return Inches.of(raw);
    }

    /**
     * Returns the current position of the right (follower) arm.
     *
     * @return Right arm extension.
     */
    @Override
    public Distance getRightPosition() {
      double raw = RobotBase.isSimulation() && followerSim != null
          ? followerSim.getPosition()
          : follower.getEncoder().getPosition();
      return Inches.of(raw);
    }

    /**
     * Returns the average position of both arms.
     * Used by commands that only need a single coarse reference.
     *
     * @return Average arm extension.
     */
    @Override
    public Distance getPosition() {
      return Inches.of(
          (getLeftPosition().in(Inches) + getRightPosition().in(Inches)) / 2.0);
    }

    /**
     * Returns the current velocity of the left (leader) arm.
     *
     * @return Left arm velocity.
     */
    @Override
    public LinearVelocity getLeftVelocity() {
      double raw = RobotBase.isSimulation() && leaderSim != null
          ? leaderSim.getVelocity()
          : leader.getEncoder().getVelocity();
      return InchesPerSecond.of(raw);
    }

    /**
     * Returns the current velocity of the right (follower) arm.
     *
     * @return Right arm velocity.
     */
    @Override
    public LinearVelocity getRightVelocity() {
      double raw = RobotBase.isSimulation() && followerSim != null
          ? followerSim.getVelocity()
          : follower.getEncoder().getVelocity();
      return InchesPerSecond.of(raw);
    }

    /**
     * Returns the average velocity of both arms.
     * Used by the periodic safety net to determine direction of travel.
     *
     * @return Average arm velocity.
     */
    @Override
    public LinearVelocity getVelocity() {
      return InchesPerSecond.of(
          (getLeftVelocity().in(InchesPerSecond) + getRightVelocity().in(InchesPerSecond)) / 2.0);
    }

    /**
     * Returns true when the average arm position is within
     * ClimberConstants.kPositionTolerance of the last commanded setpoint.
     */
    @Override
    public boolean isAtPosition() {
      Distance setpoint = RobotBase.isReal()
          ? Inches.of(leader.getClosedLoopController().getSetpoint())
          : simSetpoint;

      boolean leftAtPosition = getLeftPosition().isNear(setpoint, ClimberConstants.kPositionTolerance);
      boolean rightAtPosition = getRightPosition().isNear(setpoint, ClimberConstants.kPositionTolerance);

      return leftAtPosition && rightAtPosition;
    }

    @Override
    public boolean isUsingPercentSetpoints() {
      return usingPercent;
    }

    /**
     * Returns the leader applied output as a fraction of bus voltage [-1, 1].
     * Both motors receive the same command so the leader is representative.
     */
    @Override
    public double getAppliedOutput() {
      return leader.getAppliedOutput();
    }

    /**
     * Returns the leader output current in amps.
     * Both motors receive the same command so the leader is representative.
     */
    @Override
    public double getCurrent() {
      return leader.getOutputCurrent();
    }

    /**
     * Zeroes the left (leader) encoder.
     * Called when the left bottom limit switch triggers so that subsequent
     * position commands for the left arm are accurate.
     */
    @Override
    public void resetLeftEncoder() {
      leader.getEncoder().setPosition(0);
      if (leaderSim != null) {
        leaderSim.setPosition(0);
      }
    }

    /**
     * Zeroes the right (follower) encoder.
     * Called when the right bottom limit switch triggers so that subsequent
     * position commands for the right arm are accurate.
     */
    @Override
    public void resetRightEncoder() {
      follower.getEncoder().setPosition(0);
      if (followerSim != null) {
        followerSim.setPosition(0);
      }
    }

    /**
     * Advances the simulation model by one robot loop cycle.
     * Updates bus voltage, integrates velocity into position, and clamps
     * the simulated position to the physical travel limits.
     */
    @Override
    public void simulationPeriodic() {
      if (leaderSim == null || followerSim == null)
        return;

      final double now = Timer.getFPGATimestamp();
      double dt = now - lastSimSeconds;
      if (dt <= 0)
        dt = 0.02;
      lastSimSeconds = now;

      leaderSim.setBusVoltage(RobotController.getBatteryVoltage());
      followerSim.setBusVoltage(RobotController.getBatteryVoltage());

      final LinearVelocity maxSpeed = SimulationConstants.kSimulatedMaxClimberSpeed;

      LinearVelocity velocity;
      if (usingPercent) {
        velocity = maxSpeed.times(simPercentSetpoint).times(ClimberConstants.kOutputUp);
      } else {
        Distance error = simSetpoint.minus(getPosition());
        double desiredInchesPerSec = error.in(Inches) / dt;
        double clampedInchesPerSec = MathUtil.clamp(
            desiredInchesPerSec,
            -maxSpeed.in(InchesPerSecond),
            maxSpeed.in(InchesPerSecond));
        velocity = InchesPerSecond.of(clampedInchesPerSec);
      }

      leaderSim.setVelocity(velocity.in(InchesPerSecond));
      followerSim.setVelocity(velocity.in(InchesPerSecond));

      Distance newPos = getPosition().plus(Inches.of(velocity.in(InchesPerSecond) * dt));
      Distance clampedPos = (Distance) UnitHelpers.clamp(
          newPos,
          ClimberConstants.kMinLength,
          SimulationConstants.kSimulatedMaxClimberHeight);

      leaderSim.setPosition(clampedPos.in(Inches));
      followerSim.setPosition(clampedPos.in(Inches));
    }
  }

  /* The motor wrapper that drives both arms. */
  private final ClimberMotorInterface motor;

  /*
   * Normally-open limit switches, one per arm. Active-low wiring — reads false
   * when triggered.
   */
  private final DigitalInput leftBottomLimitSwitch;
  private final DigitalInput rightBottomLimitSwitch;

  /* Last PID setpoint issued, used for atSetpoint() calculation. */
  private Distance lastSetpoint = Inches.of(0);

  /**
   * @param motor                  The motor wrapper driving both arms.
   * @param leftBottomLimitSwitch  DIO-wired limit switch at the bottom of the
   *                               left arm.
   * @param rightBottomLimitSwitch DIO-wired limit switch at the bottom of the
   *                               right arm.
   */
  public ClimberSubsystem(
      ClimberMotorInterface motor,
      DigitalInput leftBottomLimitSwitch,
      DigitalInput rightBottomLimitSwitch) {
    this.motor = motor;
    this.leftBottomLimitSwitch = leftBottomLimitSwitch;
    this.rightBottomLimitSwitch = rightBottomLimitSwitch;
  }

  /**
   * Commands both arms to a position via onboard PID.
   * The target is clamped to [kMinLength, kMaxLength].
   *
   * @param position Desired arm extension.
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

  /**
   * Drives both arms at open-loop percent output [-1, 1].
   *
   * @param percent Output fraction.
   */
  public void setPercentOutput(double percent) {
    motor.setPercentOutput(percent);
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
  }

  /** Stops both motors immediately. */
  public void stop() {
    motor.stop();
  }

  /**
   * Zeroes the left arm encoder.
   * Call this when the left bottom limit switch triggers.
   */
  public void resetLeftEncoder() {
    motor.resetLeftEncoder();
    Logger.recordOutput("Climber/LeftEncoderReset", true);
  }

  /**
   * Zeroes the right arm encoder.
   * Call this when the right bottom limit switch triggers.
   */
  public void resetRightEncoder() {
    motor.resetRightEncoder();
    Logger.recordOutput("Climber/RightEncoderReset", true);
  }

  /** @return True when the left bottom limit switch is triggered. */
  public boolean isLeftAtBottom() {
    return limitSwitchTriggered(leftBottomLimitSwitch);
  }

  /** @return True when the right bottom limit switch is triggered. */
  public boolean isRightAtBottom() {
    return limitSwitchTriggered(rightBottomLimitSwitch);
  }

  /** @return True when either bottom limit switch is triggered. */
  public boolean isEitherAtBottom() {
    return isLeftAtBottom() || isRightAtBottom();
  }

  /** @return True when both bottom limit switches are triggered. */
  public boolean areBothAtBottom() {
    return isLeftAtBottom() && isRightAtBottom();
  }

  /** @return Current left arm position. */
  public Distance getLeftPosition() {
    return motor.getLeftPosition();
  }

  /** @return Current right arm position. */
  public Distance getRightPosition() {
    return motor.getRightPosition();
  }

  /** @return Average position of both arms. */
  public Distance getPosition() {
    return motor.getPosition();
  }

  /** @return Current left arm velocity. */
  public LinearVelocity getLeftVelocity() {
    return motor.getLeftVelocity();
  }

  /** @return Current right arm velocity. */
  public LinearVelocity getRightVelocity() {
    return motor.getRightVelocity();
  }

  /** @return Average velocity of both arms. */
  public LinearVelocity getVelocity() {
    return motor.getVelocity();
  }

  /**
   * @return True when the average arm position is within
   *         ClimberConstants.kPIDPositionTolerance of the last setpoint.
   */
  public boolean atSetpoint() {
    return motor.getPosition().isNear(lastSetpoint, ClimberConstants.kPIDPositionTolerance);
  }

  /**
   * Evaluates a limit switch and returns whether it is at the bottom.
   * On real hardware the switch is active-low (reads false when closed).
   * In simulation the position proximity to zero is used instead.
   *
   * @param limitSwitch The DigitalInput to evaluate.
   * @return True when the switch indicates the arm is at the bottom.
   */
  private boolean limitSwitchTriggered(DigitalInput limitSwitch) {
    if (RobotBase.isReal()) {
      return !limitSwitch.get();
    } else {
      return getPosition().isNear(Inches.of(0), SimulationConstants.kSimulatedClimberBottomTolerance);
    }
  }

  @Override
  public void periodic() {
    Distance leftPosition = motor.getLeftPosition();
    Distance rightPosition = motor.getRightPosition();
    Distance avgPosition = motor.getPosition();

    LinearVelocity leftVelocity = motor.getLeftVelocity();
    LinearVelocity rightVelocity = motor.getRightVelocity();
    LinearVelocity avgVelocity = motor.getVelocity();

    /*
     * Independent encoder zeroing safety net.
     * Each arm zeros its own encoder the moment its limit switch triggers
     * while descending. This keeps the two arms independently calibrated
     * regardless of which one reaches the bottom first.
     */
    if (isLeftAtBottom()) {
      motor.resetLeftEncoder();
      if (motor.isUsingPercentSetpoints() && leftVelocity.in(InchesPerSecond) < 0) {
        motor.stopLeft();
      }
    }
    if (isRightAtBottom()) {
      motor.resetRightEncoder();
      if (motor.isUsingPercentSetpoints() && rightVelocity.in(InchesPerSecond) < 0) {
        motor.stopRight();
      }
    }

    /* Stop both motors only once both arms have fully seated at the bottom. */
    // if (areBothAtBottom()) {
    // if (motor.isUsingPercentSetpoints()) {
    // motor.stop();
    // } else {

    // }
    // }

    Logger.recordOutput("Climber/Measured/LeftPositionInches", leftPosition.in(Inches));
    Logger.recordOutput("Climber/Measured/RightPositionInches", rightPosition.in(Inches));
    Logger.recordOutput("Climber/Measured/AvgPositionInches", avgPosition.in(Inches));
    Logger.recordOutput("Climber/Measured/Pose3d",
        new Pose3d(0, 0, avgPosition.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/LeftVelocityInchesPerSec", leftVelocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/RightVelocityInchesPerSec", rightVelocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AvgVelocityInchesPerSec", avgVelocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AppliedOutput", motor.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/CurrentAmps", motor.getCurrent());
    Logger.recordOutput("Climber/Measured/PositionDeltaInches",
        Math.abs(leftPosition.in(Inches) - rightPosition.in(Inches)));
    Logger.recordOutput("Climber/AtSetpoint", atSetpoint());
    Logger.recordOutput("Climber/LimitSwitch/LeftAtBottom", isLeftAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/RightAtBottom", isRightAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/BothAtBottom", areBothAtBottom());
  }

  @Override
  public void simulationPeriodic() {
    motor.simulationPeriodic();
  }
}