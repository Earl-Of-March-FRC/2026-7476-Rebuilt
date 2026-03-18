// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.subsystems.Drivetrain.SimulatedGyro;
import frc.robot.subsystems.Drivetrain.SimulatedSwerveModule;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.swerve.SwerveDriveProfile;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.commands.climber.ClimbPercentCmd;
import frc.robot.commands.climber.ClimbToHeightCmd;
import frc.robot.commands.climber.ClimbUpCmd;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.commands.drivetrain.DriveXLockCmd;
import frc.robot.commands.groups.DriveAndClimbCmd;
import frc.robot.commands.groups.DriveAndLaunchCmd;
import frc.robot.commands.groups.DriveToTowerSideCmd;
import frc.robot.commands.groups.LaunchAndClimbCmd;
import frc.robot.commands.groups.LaunchAndIndexCmd;
import frc.robot.commands.groups.XLockAndLaunchCmd;
import frc.robot.commands.indexer.IndexerCmd;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;

import com.revrobotics.spark.SparkMax;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  // public final OTBIntakeSubsystem otbIntakeSub;
  public final IndexerSubsystem indexerSub;
  public final LauncherAndIntakeSubsystem launcherAndIntakeSub;
  public final ClimberSubsystem climberSub;

  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);
  private final CommandXboxController testController = new CommandXboxController(
      OIConstants.kTestControllerPort);

  private LoggedDashboardChooser<Command> autoChooser, debugChooser;

  public RobotContainer() {
    // Get profile from Elastic dashboard selector
    SwerveDriveProfile activeProfile = ProfileSelector.getSelectedOrDefault(SwerveDriveProfile.COMP_BOT);
    SwerveConfig.applyProfile(activeProfile);

    launcherAndIntakeSub = new LauncherAndIntakeSubsystem(
        new LauncherAndIntakeSubsystem.SparkMaxLauncherAndIntakeMotor(
            new SparkMax(Constants.LauncherAndIntakeConstants.kLeaderCanSparkId,
                Constants.LauncherAndIntakeConstants.kMotorType),
            new SparkMax(Constants.LauncherAndIntakeConstants.kFollowerCanSparkId,
                Constants.LauncherAndIntakeConstants.kMotorType)));

    climberSub = new ClimberSubsystem(
        new SparkMax(Constants.ClimberConstants.kLeftId, Constants.ClimberConstants.kMotorType),
        Constants.ClimberConstants.kConfigLeft,
        new SparkMax(Constants.ClimberConstants.kRightId, Constants.ClimberConstants.kMotorType),
        Constants.ClimberConstants.kConfigRight,
        new DigitalInput(Constants.ClimberConstants.kLeftBottomLimitSwitchDIOPort),
        new DigitalInput(Constants.ClimberConstants.kRightBottomLimitSwitchDIOPort));
    indexerSub = new IndexerSubsystem(
        new SparkMax(Constants.IndexerConstants.kWheelCanId, Constants.IndexerConstants.kMotorType),
        new SparkMax(Constants.IndexerConstants.kTreadmillCanId, Constants.IndexerConstants.kMotorType));

    if (Robot.isReal()) { // This is if the Robot is
      gyro = SwerveConfig.gyro;

      // RPM tuning interface — constructing registers the SmartDashboard key
      new LauncherCmd(launcherAndIntakeSub, () -> RPM.of(SmartDashboard.getNumber("RPM", 0)));
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
      final Pose2d startPose;
      if (PoseHelpers.getAlliance() == Alliance.Blue) {
        startPose = SimulationConstants.kStartingPose;
      } else {
        // Red alliance
        double alternateX = FieldConstants.kFieldLengthX.in(Meters) - SimulationConstants.kStartingPose.getX();
        double alternateY = FieldConstants.kFieldWidthY.in(Meters) - SimulationConstants.kStartingPose.getY();
        Rotation2d alternateTheta = SimulationConstants.kStartingPose.getRotation().plus(Rotation2d.k180deg);
        startPose = new Pose2d(alternateX, alternateY, alternateTheta);
      }
      final SwerveDriveSimulation simulatedSwerveDrive = new SwerveDriveSimulation(
          DriveTrainSimulationConfig.Default()
              .withGyro(SimulationConstants.kSimulatedGyro)
              .withSwerveModule(SimulationConstants.kSwerveModuleSimConfig)
              .withTrackLengthTrackWidth(SwerveConfig.kWheelBase, SwerveConfig.kTrackWidth)
              .withBumperSize(SwerveConfig.kBumperLength, SwerveConfig.kBumperWidth),
          startPose);

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
    LaunchHelpers.setSubsystems(driveSub, launcherAndIntakeSub);

    NamedCommands.registerCommand("Drive to Launching Arc", new DriveAtLaunchingRangeCmd(
        driveSub,
        () -> 0.0,
        () -> 0.0,
        Constants.LauncherAndIntakeConstants.kTestLaunchRadius,
        true).until(() -> driveSub.isRadialControllerAtSetpoint()));

    NamedCommands.registerCommand("Launch", new LauncherCmd(launcherAndIntakeSub, AutoConstants.kLauncherRPM));
    NamedCommands.registerCommand("Drive to Launch", new DriveAtLaunchingRangeCmd(
        driveSub,
        () -> 0.0,
        () -> 0.0,
        Constants.LauncherAndIntakeConstants.kTestLaunchRadius,
        true).until(() -> driveSub.isRadialControllerAtSetpoint()));
    NamedCommands.registerCommand("Nearest Climb", PathGenerator.loadL1ClimbCommand());
    NamedCommands.registerCommand("Cross Bump", PathGenerator.crossBumpAuto(FieldConstants.kBumpPathWaypoints));
    NamedCommands.registerCommand("Cross Trench", PathGenerator.crossTrenchAuto(FieldConstants.kTrenchPathWaypoints));

    // Logger.recordOutput("Temp/UpPos", new
    // Pose2d(FieldConstants.kFieldLengthX.minus(Meters.of(15.334)),
    // FieldConstants.kFieldWidthY.minus(Meters.of(3.583)), Rotation2d.kZero));

    // Logger.recordOutput("Temp/DownPos", new
    // Pose2d(FieldConstants.kFieldLengthX.minus(Meters.of(15.334)),
    // FieldConstants.kFieldWidthY.minus(Meters.of(5.134)), Rotation2d.kZero));

    // NamedCommands.registerCommand("Launch Once Connecting Path",
    // AutoBuilder.pathfindToPose(
    // new Pose2d(AutoConstants.depotStartPoint, new Rotation2d(0, 0)),
    // AutoConstants.L1ClimbConstraints));

    NamedCommands.registerCommand("Intake Left Connecting Path",
        Commands.defer(
            () -> AutoBuilder.pathfindToPose(new Pose2d(AutoConstants.intakeLeftStartPoint, new Rotation2d(0, 0)),
                AutoConstants.L1ClimbConstraints),
            Set.of(driveSub)));

    NamedCommands.registerCommand("Launch Once Connecting Path",
        Commands.defer(
            () -> AutoBuilder.pathfindThenFollowPath(AutoConstants.depotClimbPath, AutoConstants.L1ClimbConstraints),
            Set.of(driveSub)));

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    // Resets the climber encoders when the bottom limitswitch hits (won't do this
    // in simulation to avoid some issues with climber getting stuck at bottom)
    // if (RobotBase.isReal()) {
    // new Trigger(() -> climberSub.isLeftAtBottom())
    // .onTrue(Commands.runOnce(() -> climberSub.resetLeftEncoder()));
    // new Trigger(() -> climberSub.isRightAtBottom())
    // .onTrue(Commands.runOnce(() -> climberSub.resetRightEncoder()));
    // }

    DriveCmd driveCmd = new DriveCmd(
        driveSub,
        this::getDriverVx,
        this::getDriverVy,
        this::getDriverOmega);

    DriveAtLaunchingRangeCmd driveAtLaunchingRangeCmd = new DriveAtLaunchingRangeCmd(
        driveSub,
        this::getDriverVx,
        this::getDriverVy,
        Constants.LauncherAndIntakeConstants.kTestLaunchRadius,
        Constants.LauncherAndIntakeConstants.kLeadShots);

    // Toggle locked range with the left trigger, so the driver can choose to
    // maintain optimal launching distance while lining up shots
    BooleanSupplier distanceLockSupplier = new BooleanSupplier() {
      private boolean wasPressed = driverController.leftTrigger(Constants.OIConstants.kTriggerThreshold).getAsBoolean();
      private boolean toggleOnTrue = false;

      @Override
      public boolean getAsBoolean() {
        boolean isPressed = driverController.leftTrigger(Constants.OIConstants.kTriggerThreshold).getAsBoolean();
        if (isPressed && !wasPressed) {
          toggleOnTrue = !toggleOnTrue;// toggle on rising edges
        }
        wasPressed = isPressed;
        return toggleOnTrue;
      }
    };

    Logger.recordOutput("Drivetrain/LockSupplier", distanceLockSupplier.getAsBoolean());

    BooleanSupplier launchSupplier = driverController
        .rightTrigger(Constants.OIConstants.kTriggerThreshold)::getAsBoolean;

    // Drive while tracking hub and automatically launching balls if we think they
    // will
    // go in, an additional trigger can used to lock distance
    Command driveAndAutoShootCmd = new DriveAndLaunchCmd(
        driveSub,
        indexerSub,
        launcherAndIntakeSub,
        this::getDriverVx,
        this::getDriverVy,
        distanceLockSupplier,
        Constants.LauncherAndIntakeConstants.kLeadShots);

    // Drive while tracking hub and launching balls based on an additional trigger
    // another trigger can used to lock distance
    Command driveAndManualShootCmd = new DriveAndLaunchCmd(
        driveSub,
        indexerSub,
        launcherAndIntakeSub,
        this::getDriverVx,
        this::getDriverVy,
        launchSupplier,
        distanceLockSupplier,
        Constants.LauncherAndIntakeConstants.kLeadShots);

    // Meant for use in autonomous
    // Turn to the hub and launch all balls, maintain current distance but do not
    // provide any drive input
    Command autoLaunchCmd = new DriveAndLaunchCmd(
        driveSub,
        indexerSub,
        launcherAndIntakeSub,
        () -> 0.0,
        () -> 0.0,
        // Always lock distance in auto, since the driver isn't controlling movement
        () -> true,
        Constants.LauncherAndIntakeConstants.kLeadShots)
        .withTimeout(Constants.LauncherAndIntakeConstants.kAutoLaunchTime);

    Command intakeToHopperCmd = new PulsingTreadmillCmd(
        indexerSub,
        IndexerConstants.kWheelSpeed,
        IndexerConstants.kTreadmillSpeed)
        .alongWith(new LauncherCmd(launcherAndIntakeSub, LauncherAndIntakeConstants.kIntakeRPMSetpoint));

    Command reverseIntakeCmd = new PulsingTreadmillCmd(
        indexerSub,
        -IndexerConstants.kWheelSpeed,
        -IndexerConstants.kTreadmillSpeed)
        .alongWith(new LauncherCmd(launcherAndIntakeSub, LauncherAndIntakeConstants.kIntakeRPMSetpoint.times(-1)));

    driveSub.setDefaultCommand(driveCmd);

    indexerSub.setDefaultCommand(
        new IndexerCmd(indexerSub, () -> testController.getLeftY() * IndexerConstants.kWheelSpeed,
            () -> testController.getRightY() * IndexerConstants.kTreadmillSpeed));

    // Negate so up is positive
    climberSub.setDefaultCommand(new ClimbPercentCmd(climberSub,
        () -> MathUtil.applyDeadband(-operatorController.getLeftY(), OIConstants.kDeadband)));

    // Left arm only: left stick Y on test controller
    testController.povLeft().whileTrue(
        new ClimbPercentCmd(climberSub, () -> testController.getLeftY() * 0.1));

    // Right arm only: right stick Y on test controller
    testController.povRight().whileTrue(
        new ClimbPercentCmd(climberSub, () -> testController.getRightY() * 0.1));

    // Both arms together; verify they move in the same direction
    testController.povUp().whileTrue(
        new ClimbPercentCmd(climberSub, () -> 0.1)); // should both go up

    testController.povDown().whileTrue(
        new ClimbPercentCmd(climberSub, () -> -0.1)); // should both go down

    driverController.a().toggleOnTrue(new DriveLockedHeadingCmd(
        driveSub,
        this::getDriverVx,
        this::getDriverVy,
        new Rotation2d(DriveConstants.kBumpHeadingRestriction),
        DriveConstants.kBumpLinearVelocity));

    // Lock Y coordinate to the nearest bump and align heading
    driverController.x().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        this::getDriverVx,
        () -> PoseHelpers.nearestBumpY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kBumpHeadingRestriction),
        DriveConstants.kBumpLinearVelocity));

    // Lock Y coordinate to the nearest trench and align heading
    driverController.b().toggleOnTrue(new DriveLockedHeadingAndYCmd(
        driveSub,
        this::getDriverVx,
        () -> PoseHelpers.nearestTrenchY(driveSub.getPose()),
        new Rotation2d(DriveConstants.kTrenchHeadingRestriction),
        DriveConstants.kTrenchLinearVelocity));

    driverController.y().onTrue(new CalibrateGyroCmd(driveSub));
    // driverController.y().onTrue(Commands.runOnce(() ->
    // driveSub.toggleFieldRelative(), driveSub));

    driverController.leftBumper().toggleOnTrue(intakeToHopperCmd);
    driverController.rightBumper().toggleOnTrue(reverseIntakeCmd);

    driverController.povLeft().whileTrue(new DriveToTowerSideCmd(driveSub,
        TowerSide.Left));
    driverController.povRight().whileTrue(new DriveToTowerSideCmd(driveSub,
        TowerSide.Right));

    // Cancel all driveSub commands, returning manual control
    driverController.button(7).onTrue(
        Commands.defer(() -> new InstantCommand(), Set.of(driveSub)));

    // // Binding for Plow (Button 5 is usually Left Bumper)
    // driverController.button(5).whileTrue(new IntakeCmd(otbIntakeSub, () ->
    // OTBIntakeConstants.kIntakeSpeed));

    // // Binding for Intake (Button 6 is usually Right Bumper)
    // driverController.button(6).whileTrue(new PlowCmd(otbIntakeSub, () ->
    // OTBIntakeConstants.kPlowSpeed));

    // driverController.rightBumper().onTrue(Commands.defer(
    // () -> PathGenerator.crossNearestBump(MetersPerSecond.of(0)),
    // Set.of(driveSub)));

    // driverController.leftBumper().onTrue(Commands.defer(
    // () -> PathGenerator.crossNearestTrench(MetersPerSecond.of(0)),
    // Set.of(driveSub)));

    // Lock Y coordinate to the nearest bump and align heading
    // driverController.povRight().toggleOnTrue(new DriveLockedHeadingAndYCmd(
    // driveSub,
    // this::getDriverVx,
    // () -> PoseHelpers.nearestBumpY(driveSub.getPose()),
    // new Rotation2d(DriveConstants.kBumpHeadingRestriction),
    // DriveConstants.kBumpLinearVelocity));

    // // Lock Y coordinate to the nearest trench and align heading
    // driverController.povLeft().toggleOnTrue(new DriveLockedHeadingAndYCmd(
    // driveSub,
    // this::getDriverVx,
    // () -> PoseHelpers.nearestTrenchY(driveSub.getPose()),
    // new Rotation2d(DriveConstants.kTrenchHeadingRestriction),
    // DriveConstants.kTrenchLinearVelocity));

    // driverController.povUp().and(() -> driveSub.getCurrentBotZone() ==
    // FieldZones.Neutral).onTrue(
    // Commands.defer(
    // () -> PathGenerator.driveToLaunchZoneCommandBump(MetersPerSecond.of(0)),
    // Set.of(driveSub)).andThen(driveAtLaunchingRangeCmd.asProxy()));

    // driverController.povDown().and(() -> driveSub.getCurrentBotZone() ==
    // FieldZones.Neutral).onTrue(
    // Commands.defer(
    // () -> PathGenerator.driveToLaunchZoneCommandTrench(MetersPerSecond.of(0)),
    // Set.of(driveSub)).andThen(driveAtLaunchingRangeCmd.asProxy()));

    operatorController.a().whileTrue(
        new ClimbDownCmd(climberSub));
    operatorController.b().whileTrue(
        new ClimbToHeightCmd(climberSub, ClimberConstants.kRaisePosition));
    operatorController.y().whileTrue(
        new ClimbToHeightCmd(climberSub, ClimberConstants.kLatchPosition));
    // Pass setpoint
    operatorController.x().whileTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kPassRPMSetpoint));

    driverController.povUp()
        .and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch).toggleOnTrue(
            driveAndManualShootCmd);
    driverController.povDown()
        .and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch).toggleOnTrue(
            driveAndAutoShootCmd);

    operatorController.leftBumper().debounce(OIConstants.kButtonPressDebounceSeconds)
        .and(operatorController.rightBumper())
        .and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch)
        .toggleOnTrue(new XLockAndLaunchCmd(driveSub, indexerSub, launcherAndIntakeSub));

    // RPM setpoints for visionless backups
    operatorController.povUp().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kCornerRPMSetpoint));
    operatorController.povDown().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kTowerRPMSetpoint));
    operatorController.povLeft().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kBumpRPMSetpoint));
    operatorController.povRight().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kTrenchRPMSetpoint));

    operatorController.button(7).onTrue(Commands.runOnce(launcherAndIntakeSub::stop, launcherAndIntakeSub));
    operatorController.button(8).toggleOnTrue(new DriveXLockCmd(driveSub));

    // Allow operator to apply RPM offset
    Trigger rpmTrimTrigger = new Trigger(() -> Math.abs(
        operatorController.getRightTriggerAxis()
            - operatorController.getLeftTriggerAxis()) > OIConstants.kTriggerDeadband);

    rpmTrimTrigger.whileTrue(
        Commands.run(() -> {
          double trigger = MathUtil.applyDeadband(
              operatorController.getRightTriggerAxis()
                  - operatorController.getLeftTriggerAxis(),
              OIConstants.kTriggerDeadband);

          launcherAndIntakeSub.offsetReferenceVelocity(
              LauncherAndIntakeConstants.kManualRPMOffsetPerSecond
                  .times(trigger)
                  .times(edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod));
        }));

    operatorController.rightStick().onTrue(Commands.runOnce(() -> launcherAndIntakeSub.resetVelocityOffset()));

    testController.x().whileTrue(new DriveToTowerSideCmd(driveSub, TowerSide.Left));
    testController.b().whileTrue(new DriveToTowerSideCmd(driveSub,
        TowerSide.Right));
  }

  // Helper methods to reduce repetition
  private double getDriverVx() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerYAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kDriverSlowModeMultiplier : 1),
        OIConstants.kDeadband);
  }

  private double getDriverVy() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerXAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kDriverSlowModeMultiplier : 1),
        OIConstants.kDeadband);
  }

  private double getDriverOmega() {
    // Use low sensitivity by default, and use max speed when right stick is pushed
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis)
            * (driverController.rightStick().getAsBoolean() ? 1 : OIConstants.kDriverTurnSensitivity),
        OIConstants.kDeadband);
  }

  public Gyro getGyro() {
    return gyro;
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    // autoChooser = new LoggedDashboardChooser<>("Auto Routine",
    // AutoBuilder.buildAutoChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("CalibrateGyro", new CalibrateGyroCmd(driveSub));

    // autoChooser.addOption("Cross Bump Auto",
    // Commands.defer(
    // () -> PathGenerator.crossBumpAuto(FieldConstants.kBumpPathWaypoints),
    // Set.of(driveSub)));

    // autoChooser.addOption("Cross Trench Auto",
    // Commands.defer(
    // () -> PathGenerator.crossTrenchAuto(FieldConstants.kTrenchPathWaypoints),
    // Set.of(driveSub)));

    autoChooser.addOption("Launch Then Climb Left",
        new LaunchAndClimbCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub, TowerSide.Left));

    autoChooser.addOption("Launch Then Climb Right",
        new LaunchAndClimbCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub, TowerSide.Right));

    // autoChooser.addOption("Align to Climb",
    // Commands.defer(
    // () -> PathGenerator.findL1ClimbPath(AutoConstants.crossingEndVelocity,
    // "Bump"),
    // Set.of(driveSub)));

    // autoChooser.addOption("Align to Tower Then Climb",
    // new NearestClimbCmd(driveSub, climberSub));

    autoChooser.addOption("Align to Tower Left",
        new DriveToTowerSideCmd(driveSub, TowerSide.Left));

    autoChooser.addOption("Align to Tower Right",
        new DriveToTowerSideCmd(driveSub, TowerSide.Right));

    autoChooser.addOption("Align to Tower Left & Climb",
        new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Left));

    autoChooser.addOption("Align to Tower Right & Climb",
        new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Right));

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