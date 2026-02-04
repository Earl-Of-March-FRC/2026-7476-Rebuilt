// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroADXRS450;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.subsystems.Drivetrain.SimulatedGyro;
import frc.robot.subsystems.Drivetrain.SimulatedSwerveModule;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.LauncherSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.swerve.SwerveDriveProfile;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.RestrictedDriveCmd;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;
import frc.robot.commands.intake.IntakeCmd;
import frc.robot.commands.intake.PlowCmd;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final IntakeSubsystem intakeSub;
  public final LauncherSubsystem launcherSub;
  public final IndexerSubsystem indexerSub;
  public final ClimberSubsystem ClimberSub;
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
      gyro = new GyroNavX();
      intakeSub = new IntakeSubsystem(
          new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless)); // kMotorCanId is -1 currently
      launcherSub = new LauncherSubsystem(null); // set when we have more information
      indexerSub = new IndexerSubsystem(null);
      ClimberSub = new ClimberSubsystem(null);
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
    } else { // If the robot is simulated, make simulated subs :P
      final SwerveDriveSimulation simulatedSwerveDrive = new SwerveDriveSimulation(Configs.Simulation.drivetrainConfig,
          SimulationConstants.kStartingPose);

      gyro = new SimulatedGyro(simulatedSwerveDrive.getGyroSimulation());

      intakeSub = new IntakeSubsystem(
          new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless));
      launcherSub = new LauncherSubsystem(new SparkMax(0, null));
      indexerSub = new IndexerSubsystem(new SparkMax(0, null));
      ClimberSub = new ClimberSubsystem(new SparkMax(0, null));
      driveSub = new DrivetrainSubsystem(new SwerveModule[] {
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[0]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[1]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[2]),
          new SimulatedSwerveModule(simulatedSwerveDrive.getModules()[3]) }, gyro, simulatedSwerveDrive);

      SimulatedArena.getInstance().addDriveTrainSimulation(simulatedSwerveDrive);
    }

    configureBindings();
    configureAutos();
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

    // Only schedule when in Launching zone
    driverController.x().and(driveSub::isBotInLaunchingZone).toggleOnTrue(
        new DriveAtLaunchingRangeCmd(
            driveSub,
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerYAxis),
                OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(
                -driverController.getRawAxis(OIConstants.kDriverControllerXAxis),
                OIConstants.kDriveDeadband),
            Constants.LauncherConstants.kLaunchRadius,
            true));

    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));

    driverController.y().onTrue(Commands.runOnce(() -> driveSub.toggleFieldRelative(), driveSub));

    // Binding for Plow (Button 5 is usually Left Bumper)
    driverController.button(5).whileTrue(new IntakeCmd(intakeSub, IntakeConstants.kPlowSpeed));

    // Binding for Intake (Button 6 is usually Right Bumper)
    driverController.button(6).whileTrue(new PlowCmd(intakeSub, IntakeConstants.kIntakeSpeed));

    // driverController.y().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> 0.75));
    // driverController.leftBumper().whileTrue(
    // new IndexerSetVelocityManualCmd(indexerSub, () -> -0.75));
    // driverController.rightBumper().whileTrue(
    // new IndexerSetVelocityManualCmd(indexerSub, () -> 1));

    // // Launcher commands
    // driverController.leftTrigger().whileTrue(
    // new LauncherSetVelocityPIDCmd(launcherSub, () ->
    // -launcherSub.getPreferredFrontVelocity(),
    // () -> -launcherSub.getPreferredBackVelocity()));
    // driverController.rightTrigger().toggleOnTrue(
    // new LauncherSetVelocityPIDCmd(launcherSub, () ->
    // launcherSub.getPreferredFrontVelocity(),
    // () -> launcherSub.getPreferredBackVelocity()));
    // driverController.povDown().toggleOnTrue(
    // new LauncherSetVelocityPIDCmd(launcherSub, () ->
    // LauncherConstants.kVelocityYeetForward,
    // () -> LauncherConstants.kVelocityYeetBack));

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
