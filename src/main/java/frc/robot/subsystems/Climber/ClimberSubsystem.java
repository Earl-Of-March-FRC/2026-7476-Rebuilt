package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements ClimberSubsystemInterface {

  public static enum ClimbSide {
    Left, Right
  }

  private final SparkMax leftClimberSparkMax;
  private final SparkMax rightClimberSparkMax;

  public ClimberSubsystem(SparkMax leftClimberSparkMax, SparkMax rightClimberSparkMax) {
    this.leftClimberSparkMax = leftClimberSparkMax;
    this.rightClimberSparkMax = rightClimberSparkMax;

    leftClimberSparkMax.configure(ClimberConstants.kConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    rightClimberSparkMax.configure(ClimberConstants.kConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    Distance leftPosition = Inches.of(leftClimberSparkMax.getEncoder().getPosition());
    LinearVelocity leftVelocity = InchesPerSecond.of(leftClimberSparkMax.getEncoder().getVelocity());
    Distance rightPosition = Inches.of(rightClimberSparkMax.getEncoder().getPosition());
    LinearVelocity rightVelocity = InchesPerSecond.of(rightClimberSparkMax.getEncoder().getVelocity());

    Logger.recordOutput("Climber/Left/Measured/PositionInches", leftPosition.in(Inches));
    Logger.recordOutput("Climber/Left/Measured/VelocityInchesPerSec", leftVelocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Left/Measured/AppliedOutput", leftClimberSparkMax.getAppliedOutput());
    Logger.recordOutput("Climber/Left/Measured/CurrentAmps", leftClimberSparkMax.getOutputCurrent());

    Logger.recordOutput("Climber/Right/Measured/PositionInches", rightPosition.in(Inches));
    Logger.recordOutput("Climber/Right/Measured/VelocityInchesPerSec", rightVelocity.in(InchesPerSecond));
    Logger.recordOutput("Climber/Right/Measured/AppliedOutput", rightClimberSparkMax.getAppliedOutput());
    Logger.recordOutput("Climber/Right/Measured/CurrentAmps", rightClimberSparkMax.getOutputCurrent());
  }

  @Override
  public void setPercentOutput(double percent) {
    Logger.recordOutput("Climber/Setpoint/PercentOutput", percent);
    leftClimberSparkMax.set(percent);
    rightClimberSparkMax.set(percent);
  }

  @Override
  public void setTargetPosition(double inches) {
    Distance target = Inches.of(inches);
    Logger.recordOutput("Climber/Setpoint/TargetInches", target.in(Inches));
    leftClimberSparkMax.getClosedLoopController().setSetpoint(target.in(Inches), SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
    rightClimberSparkMax.getClosedLoopController().setSetpoint(target.in(Inches), SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    setPercentOutput(0);
  }

  /** @return Velocity in inches per second. (left side default) */
  @Override
  public double getVelocity() {
    return leftClimberSparkMax.getEncoder().getVelocity();
  }

  /**
   * Sets the climber to a percent output on the specified side.
   * 
   * @param percent Output in range [-1.0, 1.0].
   * @param side    The side of the climber to control.
   */
  public void setPercentOutput(double percent, ClimbSide side) {
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/PercentOutput", percent);

    if (side == ClimbSide.Left) {
      leftClimberSparkMax.set(percent);
    } else if (side == ClimbSide.Right) {
      rightClimberSparkMax.set(percent);
    }
  }

  /**
   * Sets the climber to a target position on the specified side.
   * 
   * @param inches Target position in inches.
   * @param side   The side of the climber to control.
   */
  public void setTargetPosition(double inches, ClimbSide side) {
    Distance target = Inches.of(inches);
    Logger.recordOutput("Climber/" + side.name() + "/Setpoint/TargetInches", target.in(Inches));

    if (side == ClimbSide.Left) {
      leftClimberSparkMax.getClosedLoopController().setSetpoint(target.in(Inches), SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0);
    } else if (side == ClimbSide.Right) {
      rightClimberSparkMax.getClosedLoopController().setSetpoint(target.in(Inches), SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0);
    }
  }

  /**
   * @return Velocity in inches per second.
   * 
   * @param side The side of the climber to query.
   */
  public LinearVelocity getVelocity(ClimbSide side) {
    if (side == ClimbSide.Left) {
      return InchesPerSecond.of(leftClimberSparkMax.getEncoder().getVelocity());
    } else if (side == ClimbSide.Right) {
      return InchesPerSecond.of(rightClimberSparkMax.getEncoder().getVelocity());
    } else {
      return InchesPerSecond.of(0);
    }
  }
}