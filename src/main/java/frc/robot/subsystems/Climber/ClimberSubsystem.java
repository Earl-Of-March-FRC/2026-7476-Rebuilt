package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.UnitHelpers;

/**
 * Controls the dual-arm climber mechanism.
 *
 * <p>
 * Each arm is driven by its own {@link ClimberArm} instance and can be
 * stopped independently when its beam-break limit switch triggers. Both arms
 * always receive the same commanded percent output, but if one arm reaches
 * the bottom while the other has not, only that arm stops — the other
 * continues unaffected.
 *
 * <p>
 * Sign convention: positive percent output raises both arms, negative
 * lowers them. Physical motor inversion is handled inside the motor configs
 * so all software above this layer uses a consistent sign convention.
 */
public class ClimberSubsystem extends SubsystemBase {

  /** Identifies which side of the climb tower the robot is targeting. */
  public static enum TowerSide {
    Left,
    Right
  }

  private final ClimberArmInterface leftArm;
  private final ClimberArmInterface rightArm;

  private final DigitalInput leftBottomLimitSwitch;
  private final DigitalInput rightBottomLimitSwitch;

  private Distance lastSetpoint = Inches.of(0);

  /**
   * Constructs the climber subsystem with two independent motor controllers and
   * two beam-break limit switches.
   *
   * @param leftMotor              the SparkMax for the left arm
   * @param leftConfig             configuration to apply to the left motor
   * @param rightMotor             the SparkMax for the right arm
   * @param rightConfig            configuration to apply to the right motor
   * @param leftBottomLimitSwitch  DIO-wired beam-break at the bottom of the left
   *                               arm
   * @param rightBottomLimitSwitch DIO-wired beam-break at the bottom of the right
   *                               arm
   */
  public ClimberSubsystem(
      SparkMax leftMotor, SparkMaxConfig leftConfig,
      SparkMax rightMotor, SparkMaxConfig rightConfig,
      DigitalInput leftBottomLimitSwitch,
      DigitalInput rightBottomLimitSwitch) {

    this.leftArm = new ClimberArm(leftMotor, leftConfig);
    this.rightArm = new ClimberArm(rightMotor, rightConfig);
    this.leftBottomLimitSwitch = leftBottomLimitSwitch;
    this.rightBottomLimitSwitch = rightBottomLimitSwitch;
  }

  /**
   * Sends the same percent output to both arms independently.
   *
   * <p>
   * If an arm's beam-break is triggered and percent is negative (retracting),
   * that arm is stopped while the other continues. This allows one arm to finish
   * seating while the other is already at the bottom.
   *
   * @param percent output fraction in {@code [-1, 1]}; positive = up, negative =
   *                down
   */
  public void setPercentOutput(double percent) {
    if (percent < 0 && isLeftAtBottom()) {
      leftArm.stop();
    } else {
      leftArm.setPercentOutput(percent);
    }

    if (percent < 0 && isRightAtBottom()) {
      rightArm.stop();
    } else {
      rightArm.setPercentOutput(percent);
    }

    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
  }

  /**
   * Commands both arms to the same position setpoint via their onboard PID
   * controllers. The target is clamped to {@code [kMinLength, kMaxLength]}.
   *
   * @param position desired arm extension
   */
  public void setTargetPosition(Distance position) {
    Distance clamped = (Distance) UnitHelpers.clamp(
        position,
        ClimberConstants.kMinLength,
        ClimberConstants.kMaxLength);
    lastSetpoint = clamped;
    leftArm.setTargetPosition(clamped);
    rightArm.setTargetPosition(clamped);
    Logger.recordOutput("Climber/Setpoint/Inches", clamped.in(Inches));
    Logger.recordOutput("Climber/Setpoint/Meters", clamped.in(Meters));
  }

  /**
   * Drives both arms upward independently, stopping each arm the moment it
   * reaches or exceeds the target. The other arm continues until it also
   * reaches the target.
   *
   * <p>
   * This is the bang-bang upward drive used by
   * {@link frc.robot.commands.climber.ClimbUpCmd}. There are no upper limit
   * switches, so position is checked in software.
   *
   * @param targetPosition the desired arm extension
   */
  public void driveUpToPosition(Distance targetPosition) {
    if (getLeftPosition().gte(targetPosition))
      leftArm.stop();
    else
      leftArm.setPercentOutput(ClimberConstants.kOutputRangeMax);

    if (getRightPosition().gte(targetPosition))
      rightArm.stop();
    else
      rightArm.setPercentOutput(ClimberConstants.kOutputRangeMax);
  }

  /** Stops both arms immediately. */
  public void stop() {
    leftArm.stop();
    rightArm.stop();
  }

  /**
   * Zeroes the left arm encoder.
   * Called automatically in {@link #periodic()} when the left beam-break
   * triggers.
   */
  public void resetLeftEncoder() {
    leftArm.resetEncoder();
    Logger.recordOutput("Climber/LeftEncoderReset", true);
  }

  /**
   * Zeroes the right arm encoder.
   * Called automatically in {@link #periodic()} when the right beam-break
   * triggers.
   */
  public void resetRightEncoder() {
    rightArm.resetEncoder();
    Logger.recordOutput("Climber/RightEncoderReset", true);
  }

  /**
   * Returns {@code true} when the left bottom limit switch is triggered.
   * On real hardware the switch is active-low (reads {@code false} when closed).
   * In simulation, proximity to zero is used instead.
   *
   * @return {@code true} if the left arm is at the bottom
   */
  public boolean isLeftAtBottom() {
    if (RobotBase.isReal()) {
      return !leftBottomLimitSwitch.get();
    }
    return leftArm.getPosition().isNear(Inches.of(0), SimulationConstants.kSimulatedClimberBottomTolerance);
  }

  /**
   * Returns {@code true} when the right bottom limit switch is triggered.
   * On real hardware the switch is active-low (reads {@code false} when closed).
   * In simulation, proximity to zero is used instead.
   *
   * @return {@code true} if the right arm is at the bottom
   */
  public boolean isRightAtBottom() {
    if (RobotBase.isReal()) {
      return !rightBottomLimitSwitch.get();
    }
    return rightArm.getPosition().isNear(Inches.of(0), SimulationConstants.kSimulatedClimberBottomTolerance);
  }

  /**
   * Returns {@code true} when either bottom limit switch is triggered.
   *
   * @return {@code true} if at least one arm is at the bottom
   */
  public boolean isEitherAtBottom() {
    return isLeftAtBottom() || isRightAtBottom();
  }

  /**
   * Returns {@code true} when both bottom limit switches are triggered.
   *
   * @return {@code true} if both arms are at the bottom
   */
  public boolean areBothAtBottom() {
    return isLeftAtBottom() && isRightAtBottom();
  }

  /** @return current left arm extension */
  public Distance getLeftPosition() {
    return leftArm.getPosition();
  }

  /** @return current right arm extension */
  public Distance getRightPosition() {
    return rightArm.getPosition();
  }

  /**
   * Returns the average extension of both arms.
   *
   * @return average arm extension
   */
  public Distance getPosition() {
    return Inches.of(
        (getLeftPosition().in(Inches) + getRightPosition().in(Inches)) / 2.0);
  }

  /**
   * Returns the current left arm velocity.
   * Negative values indicate the arm is moving downward.
   *
   * @return left arm velocity
   */
  public LinearVelocity getLeftVelocity() {
    return leftArm.getVelocity();
  }

  /**
   * Returns the current right arm velocity.
   * Negative values indicate the arm is moving downward.
   *
   * @return right arm velocity
   */
  public LinearVelocity getRightVelocity() {
    return rightArm.getVelocity();
  }

  /**
   * Returns the average velocity of both arms.
   *
   * @return average arm velocity
   */
  public LinearVelocity getVelocity() {
    return InchesPerSecond.of(
        (getLeftVelocity().in(InchesPerSecond) + getRightVelocity().in(InchesPerSecond)) / 2.0);
  }

  /**
   * Returns {@code true} when both arms are within
   * {@link ClimberConstants#kPIDPositionTolerance} of the last commanded
   * setpoint.
   *
   * @return {@code true} if both arms are at the setpoint
   */
  public boolean atSetpoint() {
    return leftArm.isAtPosition(lastSetpoint) && rightArm.isAtPosition(lastSetpoint);
  }

  /**
   * Returns {@code true} when both arms have reached or exceeded the target.
   * Used by {@link frc.robot.commands.climber.ClimbUpCmd} to determine when
   * to end.
   *
   * @param targetPosition the desired arm extension
   * @return {@code true} when both arms are at or above the target
   */
  public boolean bothArmsAtOrAbove(Distance targetPosition) {
    return getLeftPosition().gte(targetPosition) && getRightPosition().gte(targetPosition);
  }

  @Override
  public void periodic() {
    // Reset each encoder independently when its beam-break triggers.
    if (isLeftAtBottom()) {
      resetLeftEncoder();
    }
    if (isRightAtBottom()) {
      resetRightEncoder();
    }

    Distance leftPos = leftArm.getPosition();
    Distance rightPos = rightArm.getPosition();
    Distance avgPos = getPosition();

    LinearVelocity leftVel = leftArm.getVelocity();
    LinearVelocity rightVel = rightArm.getVelocity();
    LinearVelocity avgVel = getVelocity();

    Logger.recordOutput("Climber/Measured/LeftPositionInches", leftPos.in(Inches));
    Logger.recordOutput("Climber/Measured/RightPositionInches", rightPos.in(Inches));
    Logger.recordOutput("Climber/Measured/AvgPositionInches", avgPos.in(Inches));
    Logger.recordOutput("Climber/Measured/Pose3d",
        new Pose3d(0, 0, avgPos.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/LeftVelocityInchesPerSec", leftVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/RightVelocityInchesPerSec", rightVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AvgVelocityInchesPerSec", avgVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/LeftAppliedOutput", leftArm.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/RightAppliedOutput", rightArm.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/LeftCurrentAmps", leftArm.getCurrent());
    Logger.recordOutput("Climber/Measured/RightCurrentAmps", rightArm.getCurrent());
    Logger.recordOutput("Climber/Measured/PositionDeltaInches",
        Math.abs(leftPos.in(Inches) - rightPos.in(Inches)));
    Logger.recordOutput("Climber/AtSetpoint", atSetpoint());
    Logger.recordOutput("Climber/LimitSwitch/LeftAtBottom", isLeftAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/RightAtBottom", isRightAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/BothAtBottom", areBothAtBottom());
  }

  @Override
  public void simulationPeriodic() {
    leftArm.simulationPeriodic();
    rightArm.simulationPeriodic();
  }
}