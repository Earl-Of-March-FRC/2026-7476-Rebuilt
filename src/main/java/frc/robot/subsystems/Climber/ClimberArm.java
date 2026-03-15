package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.UnitHelpers;

/**
 * SparkMax-backed implementation of {@link ClimberArmInterface}.
 *
 * <p>
 * Wraps one {@link SparkMax} and its simulation counterpart.
 * All simulation state is self-contained so both arms can be simulated
 * independently. On real hardware the simulation fields are never touched.
 */
public class ClimberArm implements ClimberArmInterface {

  private final SparkMax motor;
  private final SparkMaxSim motorSim;

  private boolean usingPercent = false;

  private double simPercentSetpoint = 0;

  private Distance simSetpoint = Inches.of(0);

  /**
   * Constructs a {@code ClimberArm}, applies configuration to the motor, and
   * (in simulation) creates the {@link SparkMaxSim} wrapper.
   *
   * @param motor  the SparkMax controller for this arm
   * @param config the SparkMax configuration to apply
   */
  public ClimberArm(SparkMax motor, SparkMaxConfig config) {
    this.motor = motor;
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motorSim = RobotBase.isSimulation()
        ? new SparkMaxSim(motor, SimulationConstants.kSimulatedSparkMaxClimberMotor)
        : null;
  }

  @Override
  public void setPercentOutput(double percent) {
    motor.set(percent);
    usingPercent = true;
    simPercentSetpoint = percent;
  }

  @Override
  public void setTargetPosition(Distance position) {
    usingPercent = false;
    simSetpoint = position;
    motor.getClosedLoopController()
        .setSetpoint(position.in(Inches), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    motor.set(0);
    usingPercent = true;
    simPercentSetpoint = 0;
  }

  @Override
  public Distance getPosition() {
    double raw = motorSim != null
        ? motorSim.getPosition()
        : motor.getEncoder().getPosition();
    return Inches.of(raw);
  }

  @Override
  public LinearVelocity getVelocity() {
    double raw = motorSim != null
        ? motorSim.getVelocity()
        : motor.getEncoder().getVelocity();
    return InchesPerSecond.of(raw);
  }

  @Override
  public boolean isAtPosition(Distance setpoint) {
    return getPosition().isNear(setpoint, ClimberConstants.kPositionTolerance);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public void resetEncoder() {
    motor.getEncoder().setPosition(0);
    if (motorSim != null) {
      motorSim.setPosition(0);
    }
  }

  /**
   * Advances the simulation model by one robot loop cycle.
   * Integrates velocity into position and clamps the result to the physical
   * travel limits defined in {@link SimulationConstants}.
   */
  @Override
  public void simulationPeriodic() {
    if (motorSim == null) {
      return;
    }

    final double dt = 0.02;

    motorSim.setBusVoltage(RobotController.getBatteryVoltage());

    final LinearVelocity maxSpeed = SimulationConstants.kSimulatedMaxClimberSpeed;

    LinearVelocity velocity;
    if (usingPercent) {
      velocity = maxSpeed.times(simPercentSetpoint);
    } else {
      Distance error = simSetpoint.minus(getPosition());
      velocity = (LinearVelocity) UnitHelpers.clamp(
          InchesPerSecond.of(error.in(Inches) / dt),
          maxSpeed.times(-1),
          maxSpeed);
    }

    motorSim.setVelocity(velocity.in(InchesPerSecond));

    Distance newPos = getPosition().plus(Inches.of(velocity.in(InchesPerSecond) * dt));
    Distance clampedPos = (Distance) UnitHelpers.clamp(
        newPos,
        ClimberConstants.kMinLength,
        SimulationConstants.kSimulatedMaxClimberHeight);

    motorSim.setPosition(clampedPos.in(Inches));
  }
}