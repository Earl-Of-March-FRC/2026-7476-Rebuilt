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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import static org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.swerve.SwerveDriveProfile;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.util.PoseHelpers;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Get profile from Elastic dashboard selector
    SwerveDriveProfile activeProfile = ProfileSelector.getSelectedOrDefault(SwerveDriveProfile.COMP_BOT);
    SwerveConfig.applyProfile(activeProfile);
    // SwerveDriveProfile activeProfile = SwerveProfiles.COMP_BOT;
    // SwerveDriveProfile activeProfile = SwerveProfiles.SPONGE_BOT;
    // SwerveDriveProfile activeProfile = SwerveProfiles.OFF_SEASON_SWERVE;

    if (Robot.isReal()) {
      gyro = SwerveConfig.gyro;

      driveSub = new DrivetrainSubsystem(new MAXSwerveModule[] {
          new MAXSwerveModule(
              SwerveConfig.kFrontLeftDrivingCanId,
              SwerveConfig.kFrontLeftTurningCanId,
              Constants.DriveConstants.kFrontLeftChassisAngularOffset),
          new MAXSwerveModule(
              SwerveConfig.kFrontRightDrivingCanId,
              SwerveConfig.kFrontRightTurningCanId,
              Constants.DriveConstants.kFrontRightChassisAngularOffset),
          new MAXSwerveModule(
              SwerveConfig.kBackLeftDrivingCanId,
              SwerveConfig.kBackLeftTurningCanId,
              Constants.DriveConstants.kBackLeftChassisAngularOffset),
          new MAXSwerveModule(
              SwerveConfig.kBackRightDrivingCanId,
              SwerveConfig.kBackRightTurningCanId,
              Constants.DriveConstants.kBackRightChassisAngularOffset)
      }, gyro);
    } else {
      final SwerveDriveSimulation simulatedSwerveDrive = new SwerveDriveSimulation(Configs.Simulation.drivetrainConfig,
          SimulationConstants.kStartingPose);

      gyro = new SimulatedGyro(simulatedSwerveDrive.getGyroSimulation());

      // Override bump collision (on by default)
      SimulatedArena.overrideInstance(
          new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt(
              SimulationConstants.kSimBumpCollision));

      driveSub = new DrivetrainSubsystem(new SwerveModule[] {
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[0]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[1]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[2]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[3]) }, gyro, simulatedSwerveDrive);
      SimulatedArena.getInstance().addDriveTrainSimulation(simulatedSwerveDrive);
    }

    PathGenerator.setDrivetrain(driveSub);

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    DriveCmd driveCmd = new DriveCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis),
            OIConstants.kDriveDeadband));

    DriveAtLaunchingRangeCmd driveAtLaunchingRangeCmd = new DriveAtLaunchingRangeCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        Constants.LauncherConstants.kLaunchRadius,
        true);

    driveSub.setDefaultCommand(driveCmd);

    driverController.a().toggleOnTrue(new DriveLockedHeadingCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband),
        new Rotation2d(DriveConstants.kBumpHeadingRestriction)));

    // Only schedule when in Launching zone
    driverController.x().and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch)
        .toggleOnTrue(driveAtLaunchingRangeCmd);

    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));

    driverController.y().onTrue(Commands.runOnce(() -> driveSub.toggleFieldRelative(), driveSub));

    driverController.rightBumper().onTrue(Commands.defer(
        () -> PathGenerator.crossNearestBump(MetersPerSecond.of(0)),
        Set.of(driveSub)));

    driverController.leftBumper().onTrue(Commands.defer(
        () -> PathGenerator.crossNearestTrench(MetersPerSecond.of(0)),
        Set.of(driveSub)));

    // Lock Y coordinate to the nearest bump and align heading
    driverController.povRight().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> PoseHelpers.nearestBumpY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kBumpHeadingRestriction)));

    // Lock Y coordinate to the nearest trench and align heading
    driverController.povLeft().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> PoseHelpers.nearestTrenchY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kTrenchHeadingRestriction)));

    driverController.povUp().and(() -> driveSub.getCurrentBotZone() == FieldZones.Neutral).onTrue(
        Commands.defer(
            () -> PathGenerator.driveToLaunchZoneCommandBump(MetersPerSecond.of(0)),
            Set.of(driveSub)).andThen(driveAtLaunchingRangeCmd.asProxy()));

    driverController.povDown().and(() -> driveSub.getCurrentBotZone() == FieldZones.Neutral).onTrue(
        Commands.defer(
            () -> PathGenerator.driveToLaunchZoneCommandTrench(MetersPerSecond.of(0)),
            Set.of(driveSub)).andThen(driveAtLaunchingRangeCmd.asProxy()));

    // Cancel all driveSub commands, returning manual control
    driverController.button(7).onTrue(
        Commands.defer(() -> new InstantCommand(), Set.of(driveSub)));
  }

  public Gyro getGyro() {
    return gyro;
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("CalibrateGyro", new CalibrateGyroCmd(driveSub));

    autoChooser.addOption("BumpRightL1", new PathPlannerAuto("BumpRightL1"));
    autoChooser.addOption("BumpLeftL1", new PathPlannerAuto("BumpLeftL1"));
    autoChooser.addOption("TrenchRightL1", new PathPlannerAuto("TrenchRightL1"));
    autoChooser.addOption("TrenchLeftL1", new PathPlannerAuto("TrenchLeftL1"));

    autoChooser.addOption("BumpRightL2", PathGenerator.loadL1ClimbCommand("Bump - Left(L1 Climb)"));

    SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    Logger.recordOutput("Drivetrain/SelectedAuto", selectedAuto == null ? "Null" : selectedAuto.getName());
    return selectedAuto;
  }
}
