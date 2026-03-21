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
    Right;

    /**
     * Gets the side of the climbers that will be used to climb this side of the
     * tower
     * 
     * @param isFacingDriverstation Whether the drivetrain is facing the
     *                              driverstation
     * @return The corresponding {@link ClimberArmSide}
     */
    public ClimberArmSide getCorrespondingArmSide(boolean isFacingDriverstation) {
      return switch (this) {
        case Left -> isFacingDriverstation ? ClimberArmSide.Left : ClimberArmSide.Right;
        case Right -> isFacingDriverstation ? ClimberArmSide.Right : ClimberArmSide.Left;
      };
    }
  }

  /**
   * Side of the climber arm(s)
   */
  public static enum ClimberArmSide {
    Left, Right, Both
  }

  private final ClimberArmInterface leftArm;
  private final ClimberArmInterface rightArm;

  /** Used to determine if left arm encoder can be trusted */
  private boolean leftZeroed = false;
  /** Used to determine if right arm encoder can be trusted */
  private boolean rightZeroed = false;

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
    setPercentOutput(percent, ClimberArmSide.Both);
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
   * @param side    Side to set
   */
  public void setPercentOutput(double percent, ClimberArmSide side) {
    if (side != ClimberArmSide.Right && canLeftMove(Math.signum(percent)))
      leftArm.setPercentOutput(percent);
    else
      leftArm.stop();

    if (side != ClimberArmSide.Left && canRightMove(Math.signum(percent)))
      rightArm.setPercentOutput(percent);
    else
      rightArm.stop();

    Logger.recordOutput("Climber/Setpoint/Percent/Side", side.name());
    Logger.recordOutput("Climber/Setpoint/Percent/Output", percent);
  }

  /**
   * Commands both arms to the same position setpoint via their onboard PID
   * controllers. The target is clamped to {@code [kMinLength, kMaxLength]}.
   *
   * @param position desired arm extension
   */
  public void setTargetPosition(Distance position) {
    setTargetPosition(position, ClimberArmSide.Both);
  }

  /**
   * Commands both arms to the same position setpoint via their onboard PID
   * controllers. The target is clamped to {@code [kMinLength, kMaxLength]}.
   *
   * @param position desired arm extension
   * @param side     desired arm side
   */
  public void setTargetPosition(Distance position, ClimberArmSide side) {
    Distance clamped = (Distance) UnitHelpers.clamp(
        position,
        ClimberConstants.kMinLength,
        ClimberConstants.kMaxLength);
    lastSetpoint = clamped;
    if (side != ClimberArmSide.Right) {
      leftArm.setTargetPosition(clamped);
    } else {
      leftArm.stop();
    }

    if (side != ClimberArmSide.Left) {
      rightArm.setTargetPosition(clamped);
    } else {
      rightArm.stop();
    }

    Logger.recordOutput("Climber/Setpoint/Position/Inches", clamped.in(Inches));
    Logger.recordOutput("Climber/Setpoint/Position/Meters", clamped.in(Meters));
    Logger.recordOutput("Climber/Setpoint/Position/Side", side.name());
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

  // /**
  // * Drives both arms upward independently, stopping each arm the moment it
  // * reaches or exceeds the target. The other arm continues until it also
  // * reaches the target.
  // *
  // * <p>
  // * This is the bang-bang upward drive used by
  // * {@link frc.robot.commands.climber.ClimbUpCmd}. There are no upper limit
  // * switches, so position is checked in software.
  // *
  // * @param targetPosition the desired arm extension
  // */
  // public void driveUpToPosition(Distance targetPosition, ArmSide side) {
  // if (getLeftPosition().gte(targetPosition))
  // leftArm.stop();
  // else
  // leftArm.setPercentOutput(ClimberConstants.kOutputRangeMax);

  // if (getRightPosition().gte(targetPosition))
  // rightArm.stop();
  // else
  // rightArm.setPercentOutput(ClimberConstants.kOutputRangeMax);
  // }

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
   * Returns {@code true} when the left arm position is at or above the maximum
   *
   * @return {@code true} if the left arm is at the top
   */
  public boolean isLeftAtTop() {
    return leftArm.getPosition().gte(ClimberConstants.kMaxLength);
  }

  /**
   * Returns {@code true} when the right arm position is at or above the maximum
   *
   * @return {@code true} if the right arm is at the top
   */
  public boolean isRightAtTop() {
    return rightArm.getPosition().gte(ClimberConstants.kMaxLength);
  }

  /**
   * returns {@code true} if the left arm can safely move in the provided
   * direction
   * 
   * @param desiredVelocitySign The sign of the desired velocity, should be 0, -1,
   *                            or 1
   * @return {@code true} if the left arm can safely move
   */
  public boolean canLeftMove(double desiredVelocitySign) {
    return !((isLeftAtTop() && desiredVelocitySign > 0) || (isLeftAtBottom() && desiredVelocitySign < 0));
  }

  /**
   * returns {@code true} if the right arm can safely move in the provided
   * direction
   * 
   * @param desiredVelocitySign The sign of the desired velocity, should be 0, -1,
   *                            or 1
   * @return {@code true} if the right arm can safely move
   */
  public boolean canRightMove(double desiredVelocitySign) {
    return !((isRightAtTop() && desiredVelocitySign > 0) || (isRightAtBottom() && desiredVelocitySign < 0));
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
   * Returns {@code true} when left arm is within
   * {@link ClimberConstants#kPositionTolerance} of the last commanded
   * setpoint.
   *
   * @return {@code true} if left arm is at the setpoint
   */
  public boolean leftAtSetpoint() {
    return leftArm.isAtPosition(lastSetpoint);
  }

  /**
   * Returns {@code true} when right arm is within
   * {@link ClimberConstants#kPositionTolerance} of the last commanded
   * setpoint.
   *
   * @return {@code true} if right arm is at the setpoint
   */
  public boolean rightAtSetpoint() {
    return rightArm.isAtPosition(lastSetpoint);
  }

  /**
   * Returns {@code true} when both arms are within
   * {@link ClimberConstants#kPositionTolerance} of the last commanded
   * setpoint.
   *
   * @return {@code true} if both arms are at the setpoint
   */
  public boolean atSetpoint() {
    return leftAtSetpoint() && rightAtSetpoint();
  }

  /**
   * Returns {@code true} when left arm has reached or exceeded the target.
   * Used by {@link frc.robot.commands.climber.ClimbUpCmd} to determine when
   * to end.
   *
   * @param targetPosition the desired arm extension
   * @return {@code true} when left arm is at or above the target
   */
  public boolean leftArmAtOrAbove(Distance targetPosition) {
    return getLeftPosition().gte(targetPosition);
  }

  /**
   * Returns {@code true} when right arm has reached or exceeded the target.
   * Used by {@link frc.robot.commands.climber.ClimbUpCmd} to determine when
   * to end.
   *
   * @param targetPosition the desired arm extension
   * @return {@code true} when right arm is at or above the target
   */
  public boolean rightArmAtOrAbove(Distance targetPosition) {
    return getRightPosition().gte(targetPosition);
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
    return leftArmAtOrAbove(targetPosition) && rightArmAtOrAbove(targetPosition);
  }

  @Override
  public void periodic() {
    // Reset each encoder independently when its beam-break triggers.
    if (isLeftAtBottom())
      resetLeftEncoder();

    if (isRightAtBottom())
      resetRightEncoder();

    // Stop the arms if they are at the limits
    if (!(canLeftMove(leftArm.getDesiredVelocitySign())
        && canLeftMove(Math.signum(leftArm.getVelocity().in(InchesPerSecond))))) {
      leftArm.stop();
    }

    if (!(canRightMove(rightArm.getDesiredVelocitySign())
        && canRightMove(Math.signum(rightArm.getVelocity().in(InchesPerSecond))))) {
      rightArm.stop();
    }

    // Check if we have zeroed the arms yet
    if (!leftZeroed && isLeftAtBottom()) {
      leftZeroed = true;
    }
    // Check if we have zeroed the arms yet
    if (!rightZeroed && isRightAtBottom()) {
      rightZeroed = true;
    }

    Distance leftPos = leftArm.getPosition();
    Distance rightPos = rightArm.getPosition();
    Distance avgPos = getPosition();

    LinearVelocity leftVel = leftArm.getVelocity();
    LinearVelocity rightVel = rightArm.getVelocity();
    LinearVelocity avgVel = getVelocity();

    Logger.recordOutput("Climber/Measured/Left/PositionInches", leftPos.in(Inches));
    Logger.recordOutput("Climber/Measured/Right/PositionInches", rightPos.in(Inches));
    Logger.recordOutput("Climber/Measured/Left/PositionPose3d", new Pose3d(0, 0, leftPos.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/Right/PositionPose3d",
        new Pose3d(0, 0, rightPos.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/AvgPositionInches", avgPos.in(Inches));
    Logger.recordOutput("Climber/Measured/Pose3d",
        new Pose3d(0, 0, avgPos.in(Meters), Rotation3d.kZero));
    Logger.recordOutput("Climber/Measured/Left/VelocityInchesPerSec", leftVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/Right/VelocityInchesPerSec", rightVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/AvgVelocityInchesPerSec", avgVel.in(InchesPerSecond));
    Logger.recordOutput("Climber/Measured/Left/AppliedOutput", leftArm.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/Right/AppliedOutput", rightArm.getAppliedOutput());
    Logger.recordOutput("Climber/Measured/Left/CurrentAmps", leftArm.getCurrent());
    Logger.recordOutput("Climber/Measured/Right/CurrentAmps", rightArm.getCurrent());
    Logger.recordOutput("Climber/Measured/PositionDeltaInches",
        Math.abs(leftPos.in(Inches) - rightPos.in(Inches)));
    Logger.recordOutput("Climber/AtSetpoint", atSetpoint());
    Logger.recordOutput("Climber/LimitSwitch/Left/AtBottom", isLeftAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/Right/AtBottom", isRightAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/BothAtBottom", areBothAtBottom());
    Logger.recordOutput("Climber/LimitSwitch/Left/Zeroed", leftZeroed);
    Logger.recordOutput("Climber/LimitSwitch/Right/Zeroed", rightZeroed);
  }

  @Override
  public void simulationPeriodic() {
    leftArm.simulationPeriodic();
    rightArm.simulationPeriodic();
  }
}