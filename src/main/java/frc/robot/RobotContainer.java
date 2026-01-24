// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.subsystems.Drivetrain.SimulatedGyro;
import frc.robot.subsystems.Drivetrain.SimulatedSwerveModule;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.intake.IntakeSubsystem;
import static edu.wpi.first.units.Units.Degrees;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.RestrictedDriveCmd;
import frc.robot.commands.intake.IntakeCmd;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final IntakeSubsystem intakeSub;
  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  public RobotContainer() {

    if (Robot.isReal()) { // If it's not a simulation, make real subsystems :)
      gyro = new GyroNavX();
      intakeSub = new IntakeSubsystem(
          new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless)); // kMotorCanId is -1 currently
      driveSub = new DrivetrainSubsystem(new MAXSwerveModule[] {
          new MAXSwerveModule(
              Constants.DriveConstants.kFrontLeftDrivingCanId,
              Constants.DriveConstants.kFrontLeftTurningCanId,
              Constants.DriveConstants.kFrontLeftChassisAngularOffset),
          new MAXSwerveModule(
              Constants.DriveConstants.kFrontRightDrivingCanId,
              Constants.DriveConstants.kFrontRightTurningCanId,
              Constants.DriveConstants.kFrontRightChassisAngularOffset),
          new MAXSwerveModule(
              Constants.DriveConstants.kBackLeftDrivingCanId,
              Constants.DriveConstants.kBackLeftTurningCanId,
              Constants.DriveConstants.kBackLeftChassisAngularOffset),
          new MAXSwerveModule(
              Constants.DriveConstants.kBackRightDrivingCanId,
              Constants.DriveConstants.kBackRightTurningCanId,
              Constants.DriveConstants.kBackRightChassisAngularOffset)
      }, gyro);
    } else { // If the robot is simulated, make simulated subs :P
      final SwerveDriveSimulation simulatedSwerveDrive = new SwerveDriveSimulation(Configs.Simulation.drivetrainConfig,
          SimulationConstants.kStartingPose);

      gyro = new SimulatedGyro(simulatedSwerveDrive.getGyroSimulation());

      intakeSub = new IntakeSubsystem(
          new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless));
      driveSub = new DrivetrainSubsystem(new SwerveModule[] {
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[0]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[1]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[2]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[3]) }, gyro, simulatedSwerveDrive);

      SimulatedArena.getInstance().addDriveTrainSimulation(simulatedSwerveDrive);
    }

    configureBindings();

  }

  private void configureBindings() {
    driveSub.setDefaultCommand(new DriveCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis),
            OIConstants.kDriveDeadband)));

    driverController.a().toggleOnTrue(
        new RestrictedDriveCmd(
            driveSub,
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
                OIConstants.kDriveDeadband),
            new Rotation2d(DriveConstants.kHeadingRestriction)));

    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));

    driverController.button(5).whileTrue(new IntakeCmd(intakeSub, IntakeConstants.kPlowSpeed));

    driverController.button(6).whileTrue(new IntakeCmd(intakeSub, IntakeConstants.kIntakeSpeed));

    // Binding for Plow (Button 5 is usually Left Bumper)

  }
}
