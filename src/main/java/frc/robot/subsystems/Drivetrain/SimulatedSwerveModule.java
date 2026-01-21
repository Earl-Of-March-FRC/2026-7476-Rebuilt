package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ModuleConstants;

public class SimulatedSwerveModule implements SwerveModule {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController turnController;

  private final Angle m_chassisAngularOffset = Radians.of(0);

  public SimulatedSwerveModule(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    drivePID = new PIDController(ModuleConstants.kDrivingPSim, ModuleConstants.kDrivingISim,
        ModuleConstants.kDrivingDSim);
    driveFeedforward = new SimpleMotorFeedforward(0.0, ModuleConstants.kDrivingFFSim);
    turnController = new PIDController(ModuleConstants.kTurningPSim, ModuleConstants.kTurningISim,
        ModuleConstants.kTurningDSim);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    driveMotor = moduleSimulation.useGenericMotorControllerForDrive();
    turnMotor = moduleSimulation.useGenericControllerForSteer();
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        moduleSimulation.getDriveWheelFinalSpeed().times(ModuleConstants.kWheelDiameter).div(2.0)
            .in(MultUnit.combine(RadiansPerSecond, Meters)),
        moduleSimulation.getSteerAbsoluteFacing());
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        moduleSimulation.getDriveWheelFinalPosition().times(ModuleConstants.kWheelDiameter).div(2.0)
            .in(MultUnit.combine(Radians, Meters)),
        moduleSimulation.getSteerAbsoluteFacing());
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(new Rotation2d(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(moduleSimulation.getSteerAbsoluteFacing());

    driveMotor.requestVoltage(
        Volts.of(driveFeedforward.calculate(
            correctedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameter.div(2).in(Meters))))
            .plus(
                Volts.of(drivePID.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
                    correctedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameter.div(2).in(Meters))))));
    turnMotor.requestVoltage(Volts.of(
        turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians(),
            correctedDesiredState.angle.getRadians())));
  }
}
