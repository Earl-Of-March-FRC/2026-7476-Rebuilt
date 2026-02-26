// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.subsystems.Drivetrain.SimulatedGyro;
import frc.robot.subsystems.Drivetrain.SimulatedSwerveModule;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OTBIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.swerve.SwerveDriveProfile;
import frc.robot.commands.OTBIntake.IntakeCmd;
import frc.robot.commands.OTBIntake.PlowCmd;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.util.PoseHelpers;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;

import com.revrobotics.spark.SparkMax;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final OTBIntakeSubsystem otbIntakeSub;
  public final IndexerSubsystem indexerSub;
  public final LauncherSubsystem launcherSub;
  public final ClimberSubsystem climberSub;

  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Get profile from Elastic dashboard selector
    SwerveDriveProfile activeProfile = ProfileSelector.getSelectedOrDefault(SwerveDriveProfile.COMP_BOT);
    SwerveConfig.applyProfile(activeProfile);

    if (Robot.isReal()) { // This is if the Robot is
      gyro = SwerveConfig.gyro;

      otbIntakeSub = new OTBIntakeSubsystem(
          new SparkMax(OTBIntakeConstants.kShoulderCanId, OTBIntakeConstants.kMotorType),
          new SparkMax(OTBIntakeConstants.kRollerCanId, OTBIntakeConstants.kMotorType));

      launcherSub = new LauncherSubsystem(
          new LauncherSubsystem.SparkMaxLauncherMotor(
              new SparkMax(Constants.LauncherAndIntakeConstants.kLeaderCanSparkId,
                  Constants.LauncherAndIntakeConstants.kMotorType),
              new SparkMax(Constants.LauncherAndIntakeConstants.kFollowerCanSparkId,
                  Constants.LauncherAndIntakeConstants.kMotorType)));

      climberSub = new ClimberSubsystem(
          new ClimberSubsystem.SparkMaxClimberMotor(
              new SparkMax(Constants.ClimberConstants.kLeftId, Constants.ClimberConstants.kMotorType)),
          new ClimberSubsystem.SparkMaxClimberMotor(
              new SparkMax(Constants.ClimberConstants.kRightId, Constants.ClimberConstants.kMotorType)));
      indexerSub = new IndexerSubsystem(
          new SparkMax(Constants.IndexerConstants.kWheelCanId, Constants.IndexerConstants.kMotorType),
          new SparkMax(Constants.IndexerConstants.kTreadmillCanId, Constants.IndexerConstants.kMotorType));

      // RPM tuning interface — constructing registers the SmartDashboard key
      new LauncherCmd(launcherSub, () -> RPM.of(SmartDashboard.getNumber("RPM", 0)));
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
      final SwerveDriveSimulation simulatedSwerveDrive = new SwerveDriveSimulation(
          DriveTrainSimulationConfig.Default()
              .withGyro(SimulationConstants.kSimulatedGyro)
              .withSwerveModule(SimulationConstants.kSwerveModuleSimConfig)
              .withTrackLengthTrackWidth(SwerveConfig.kWheelBase, SwerveConfig.kTrackWidth)
              .withBumperSize(SwerveConfig.kBumperLength, SwerveConfig.kBumperWidth),
          SimulationConstants.kStartingPose);

      gyro = new SimulatedGyro(simulatedSwerveDrive.getGyroSimulation());

      otbIntakeSub = new OTBIntakeSubsystem(
          new SparkMax(OTBIntakeConstants.kShoulderCanId, OTBIntakeConstants.kMotorType),
          new SparkMax(OTBIntakeConstants.kRollerCanId, OTBIntakeConstants.kMotorType));

      launcherSub = new LauncherSubsystem(
          new LauncherSubsystem.TalonFXLauncherMotor(
              new TalonFX(Constants.LauncherAndIntakeConstants.kMotorCanTalonId)));

      climberSub = new ClimberSubsystem(
          new ClimberSubsystem.SparkMaxClimberMotor(
              new SparkMax(Constants.ClimberConstants.kLeftId, Constants.ClimberConstants.kMotorType)),
          new ClimberSubsystem.SparkMaxClimberMotor(
              new SparkMax(Constants.ClimberConstants.kRightId, Constants.ClimberConstants.kMotorType)));
      indexerSub = new IndexerSubsystem(
          new SparkMax(Constants.IndexerConstants.kWheelCanId, Constants.IndexerConstants.kMotorType),
          new SparkMax(Constants.IndexerConstants.kTreadmillCanId, Constants.IndexerConstants.kMotorType));

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
        this::getDriverVx,
        this::getDriverVy,
        this::getDriverOmega);

    DriveAtLaunchingRangeCmd driveAtLaunchingRangeCmd = new DriveAtLaunchingRangeCmd(
        driveSub,
        this::getDriverVx,
        this::getDriverVy,
        Constants.LauncherAndIntakeConstants.kLaunchRadius,
        true);

    driveSub.setDefaultCommand(driveCmd);

    driverController.a().toggleOnTrue(new DriveLockedHeadingCmd(
        driveSub,
        this::getDriverVx,
        this::getDriverVy,
        new Rotation2d(DriveConstants.kBumpHeadingRestriction),
        DriveConstants.kBumpLinearVelocity));

    // Only schedule when in Launching zone
    driverController.x().and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch)
        .toggleOnTrue(driveAtLaunchingRangeCmd);

    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));

    driverController.y().onTrue(Commands.runOnce(() -> driveSub.toggleFieldRelative(), driveSub));

    // Binding for Plow (Button 5 is usually Left Bumper)
    driverController.button(5).whileTrue(new IntakeCmd(otbIntakeSub, () -> OTBIntakeConstants.kIntakeSpeed));

    // Binding for Intake (Button 6 is usually Right Bumper)
    driverController.button(6).whileTrue(new PlowCmd(otbIntakeSub, () -> OTBIntakeConstants.kPlowSpeed));

    driverController.rightBumper().onTrue(Commands.defer(
        () -> PathGenerator.crossNearestBump(MetersPerSecond.of(0)),
        Set.of(driveSub)));

    driverController.leftBumper().onTrue(Commands.defer(
        () -> PathGenerator.crossNearestTrench(MetersPerSecond.of(0)),
        Set.of(driveSub)));

    // Lock Y coordinate to the nearest bump and align heading
    driverController.povRight().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        this::getDriverVx,
        () -> PoseHelpers.nearestBumpY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kBumpHeadingRestriction),
        DriveConstants.kBumpLinearVelocity));

    // Lock Y coordinate to the nearest trench and align heading
    driverController.povLeft().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        this::getDriverVx,
        () -> PoseHelpers.nearestTrenchY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kTrenchHeadingRestriction),
        DriveConstants.kTrenchLinearVelocity));

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

  // Helper methods to reduce repetition
  private double getDriverVx() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerYAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kDriverSlowModeMultiplier : 1),
        OIConstants.kDriveDeadband);
  }

  private double getDriverVy() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerXAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kDriverSlowModeMultiplier : 1),
        OIConstants.kDriveDeadband);
  }

  private double getDriverOmega() {
    // Use low sensitivity by default, and use max speed when right stick is pushed
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis)
            * (driverController.rightStick().getAsBoolean() ? 1 : OIConstants.kDriverTurnSensitivity),
        OIConstants.kDriveDeadband);
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