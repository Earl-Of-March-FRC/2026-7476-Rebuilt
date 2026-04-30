// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OTBIntakeConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.OTBIntake.IntakeCmd;
import frc.robot.commands.climber.ClimbDownCmd;
import frc.robot.commands.climber.ClimbPercentCmd;
import frc.robot.commands.climber.ClimbToHeightCmd;
import frc.robot.commands.drivetrain.ActiveXLockCmd;
import frc.robot.commands.drivetrain.CalibrateGyroCmd;
import frc.robot.commands.drivetrain.DriveAtLaunchingRangeCmd;
import frc.robot.commands.drivetrain.DriveCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingAndYCmd;
import frc.robot.commands.drivetrain.DriveLockedHeadingCmd;
import frc.robot.commands.drivetrain.DriveStopCmd;
import frc.robot.commands.groups.AutoDeployIntakeCmd;
import frc.robot.commands.groups.DepotAndClimbCmd;
import frc.robot.commands.groups.DepotAndNeutralZoneBumpCmd;
import frc.robot.commands.groups.DepotAndNeutralZoneTrenchCmd;
import frc.robot.commands.groups.DriveAndClimbCmd;
import frc.robot.commands.groups.DriveAndLaunchCmd;
import frc.robot.commands.groups.DriveToCornerDepotBumpCmd;
import frc.robot.commands.groups.DriveToCornerDepotTrenchCmd;
import frc.robot.commands.groups.DriveToCornerOutpostBumpCmd;
import frc.robot.commands.groups.DriveToCornerOutpostTrenchCmd;
import frc.robot.commands.groups.LaunchAndClimbCmd;
import frc.robot.commands.groups.LaunchAndDelayedNeutralZoneBumpCmd;
import frc.robot.commands.groups.LaunchAndDelayedNeutralZoneTrenchCmd;
import frc.robot.commands.groups.LaunchAndIndexCmd;
import frc.robot.commands.groups.OutpostAndClimbCmd;
import frc.robot.commands.groups.OutpostAndNeutralZoneBumpCmd;
import frc.robot.commands.groups.OutpostAndNeutralZoneTrenchCmd;
import frc.robot.commands.groups.XLockAndLaunchCmd;
import frc.robot.commands.groups.ZonePassCmd;
import frc.robot.commands.indexer.TreadmillOnCmd;
import frc.robot.commands.launcherAndIntake.LauncherCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.MAXSwerveModule;
import frc.robot.subsystems.Drivetrain.SimulatedGyro;
import frc.robot.subsystems.Drivetrain.SimulatedSwerveModule;
import frc.robot.subsystems.Drivetrain.SwerveModule;
import frc.robot.subsystems.OTBIntake.OTBIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.PoseHelpers;
import frc.robot.util.launcher.LaunchHelpers;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;
import frc.robot.util.swerve.SwerveDriveProfile;

public class RobotContainer {
  public final DrivetrainSubsystem driveSub;
  public final OTBIntakeSubsystem otbIntakeSub;
  public final IndexerSubsystem indexerSub;
  public final LauncherAndIntakeSubsystem launcherAndIntakeSub;
  public final ClimberSubsystem climberSub;

  public final Gyro gyro;
  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);
  // Only initialize in test mode
  private CommandXboxController testController;

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
    otbIntakeSub = new OTBIntakeSubsystem(
        new SparkMax(Constants.OTBIntakeConstants.kRollerCanId, Constants.OTBIntakeConstants.kMotorType));
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

    Logger.recordOutput("Temp/UpPos", new Pose2d(FieldConstants.kFieldLengthX.minus(Meters.of(15.524)),
        FieldConstants.kFieldWidthY.minus(Meters.of(3.504)), Rotation2d.kZero));

    Logger.recordOutput("Temp/DownPos", new Pose2d(FieldConstants.kFieldLengthX.minus(Meters.of(15.465)),
        FieldConstants.kFieldWidthY.minus(Meters.of(5.141)), Rotation2d.kZero));

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    // Resets the climber encoders when the bottom limitswitch hits (won't do this
    // in simulation to avoid some issues with climber getting stuck at bottom)
    // if (RobotBase.isReal()) {
    new Trigger(() -> climberSub.isLeftAtBottom())
        .onTrue(Commands.runOnce(() -> climberSub.resetLeftEncoder()));
    new Trigger(() -> climberSub.isRightAtBottom())
        .onTrue(Commands.runOnce(() -> climberSub.resetRightEncoder()));

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
        this::getLaunchVx,
        this::getLaunchVy,
        distanceLockSupplier,
        Constants.LauncherAndIntakeConstants.kLeadShots);

    // Drive while tracking hub and launching balls based on an additional trigger
    // another trigger can used to lock distance
    Command driveAndManualShootCmd = new DriveAndLaunchCmd(
        driveSub,
        indexerSub,
        launcherAndIntakeSub,
        this::getLaunchVx,
        this::getLaunchVy,
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
        .withTimeout(Constants.AutoConstants.kAutoLaunch8Time);

    // Do not use parallel compostion so that each subsystem can be cancelled
    // independantly for launching while intaking
    Command outakeFrontTreadmillCmd = new TreadmillOnCmd(
        indexerSub,
        () -> -IndexerConstants.kWheelSpeed,
        () -> -IndexerConstants.kTreadmillSpeed);
    Command outakeFrontCmd = new LauncherCmd(launcherAndIntakeSub,
        LauncherAndIntakeConstants.kIntakeRPMSetpoint.times(-1));
    Command intakeFrontTreadmillCmd = new TreadmillOnCmd(
        indexerSub,
        () -> IndexerConstants.kWheelSpeed,
        () -> IndexerConstants.kTreadmillSpeed);
    Command intakeFrontCmd = new LauncherCmd(launcherAndIntakeSub,
        LauncherAndIntakeConstants.kIntakeRPMSetpoint);

    Command outakeBackTreadmillCmd = new TreadmillOnCmd(
        indexerSub,
        () -> 0.0,
        () -> IndexerConstants.kTreadmillSpeed);
    // Use the name to differentiate the purpose of the treadmill command (launch vs
    // intake)
    outakeBackTreadmillCmd.setName("OTBTreadmill");
    Command outakeBackCmd = new IntakeCmd(otbIntakeSub, () -> OTBIntakeConstants.kOuttakeSpeed);
    Command intakeBackTreadmillCmd = new TreadmillOnCmd(
        indexerSub,
        () -> 0.0,
        () -> -IndexerConstants.kTreadmillSpeed);
    // Use the name to differentiate the purpose of the treadmill command (launch vs
    // intake)
    intakeBackTreadmillCmd.setName("OTBTreadmill");
    Command intakeBackCmd = new IntakeCmd(otbIntakeSub, () -> OTBIntakeConstants.kIntakeSpeed);

    Command zonePassCmd = new ZonePassCmd(
        driveSub,
        indexerSub,
        launcherAndIntakeSub,
        this::getLaunchVx,
        this::getLaunchVy, launchSupplier,
        Constants.LauncherAndIntakeConstants.kLeadShots);

    driveSub.setDefaultCommand(driveCmd);

    // Negate so up is positive
    climberSub.setDefaultCommand(new ClimbPercentCmd(climberSub,
        () -> MathUtil.applyDeadband(-operatorController.getLeftY(),
            OIConstants.kDeadband)));

    driverController.a().toggleOnTrue(new DriveLockedHeadingCmd(driveSub, this::getDriverVx, this::getDriverVy,
        new Rotation2d(DriveConstants.kBumpHeadingRestriction), DriveConstants.kBumpLinearVelocity));

    // Lock Y coordinate to the nearest bump and align heading
    driverController.x()
        .toggleOnTrue(new DriveLockedHeadingAndYCmd(driveSub, this::getDriverVx,
            () -> PoseHelpers.nearestBumpY(driveSub.getPose()), new Rotation2d(DriveConstants.kBumpHeadingRestriction),
            DriveConstants.kBumpLinearVelocity));

    // Supplier to detect if we're near the trench and prevent climbers from getting
    // caught by automatically lowering them
    // TO-DO: fix up this chopped code with better logic, using robot pose velocity
    // instead of stupid driver velocity
    Supplier<Double> trenchCrossXSupplier = () -> {
      if (climberSub.areBothAtBottom()) {
        return getDriverVx();
      }

      Distance x = driveSub.getPose().getMeasureX();

      Distance blueTrench = FieldConstants.kAllianceWallToHubCenter;
      Distance redTrench = FieldConstants.kFieldLengthX.minus(FieldConstants.kAllianceWallToHubCenter);

      double Vx = getDriverVx() * (PoseHelpers.getAlliance() == Alliance.Blue ? 1 : -1); // Account for reversed field
                                                                                         // coordinates on red

      if (Math.abs(x.minus(blueTrench).in(Meters)) < DriveConstants.kTrenchSafetyMargin.in(Meters)) {
        if ((Math.signum(Vx) == 1 && x.lt(blueTrench))
            || (Math.signum(Vx) == -1 && x.gt(blueTrench))) {
          return 0.0;
        } else {
          return getDriverVx();
        }
      }
      if (Math.abs(x.minus(redTrench).in(Meters)) < DriveConstants.kTrenchSafetyMargin.in(Meters)) {
        if ((Math.signum(Vx) == 1 && x.lt(redTrench))
            || (Math.signum(Vx) == -1 && x.gt(redTrench))) {
          return 0.0;
        } else {
          return getDriverVx();
        }
      }
      return getDriverVx();
    };

    // Lock Y coordinate to the nearest trench and align heading
    driverController.b()
        .toggleOnTrue(new DriveLockedHeadingAndYCmd(driveSub, trenchCrossXSupplier,
            () -> PoseHelpers.nearestTrenchY(driveSub.getPose()),
            new Rotation2d(DriveConstants.kTrenchHeadingRestriction), DriveConstants.kTrenchLinearVelocity)
            .alongWith(new ClimbDownCmd(climberSub)));

    driverController.y().onTrue(new CalibrateGyroCmd(driveSub));
    operatorController.button(8).onTrue(Commands.runOnce(() -> driveSub.toggleFieldRelative(), driveSub));

    driverController.rightBumper().toggleOnTrue(outakeBackCmd);
    driverController.rightBumper().onTrue(Commands.runOnce(() -> {
      CommandScheduler commandScheduler = CommandScheduler.getInstance();
      Command currentIndexCmd = indexerSub.getCurrentCommand();

      // Toggle the back treadmill only if the indexer subsystem is available
      // Do not schedule the treadmill if this command has "desynced" with the intake
      if ((currentIndexCmd == null || currentIndexCmd.getName().equals("OTBTreadmill"))
          && commandScheduler.isScheduled(outakeBackCmd)) {
        commandScheduler.schedule(outakeBackTreadmillCmd);
      }
      if (currentIndexCmd == outakeBackTreadmillCmd) {
        commandScheduler.cancel(outakeBackTreadmillCmd);
      }
    }));

    driverController.leftBumper().toggleOnTrue(intakeBackCmd);
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      CommandScheduler commandScheduler = CommandScheduler.getInstance();
      Command currentIndexCmd = indexerSub.getCurrentCommand();

      // Toggle the back treadmill only if the indexer subsystem is available
      // Do not schedule the treadmill if this command has "desynced" with the intake
      if ((currentIndexCmd == null || currentIndexCmd.getName().equals("OTBTreadmill"))
          && commandScheduler.isScheduled(intakeBackCmd)) {
        commandScheduler.schedule(intakeBackTreadmillCmd);
      }
      if (currentIndexCmd == intakeBackTreadmillCmd) {
        commandScheduler.cancel(intakeBackTreadmillCmd);
      }
    }));

    // Whenever we exit launching mode, resync the intake with the treadmill
    new Trigger(
        () -> indexerSub.getCurrentCommand() != null
            && !indexerSub.getCurrentCommand().getName().equals("OTBTreadmill"))
        .onFalse(Commands.runOnce(
            () -> {
              CommandScheduler commandScheduler = CommandScheduler.getInstance();
              Command currentIntakeCmd = otbIntakeSub.getCurrentCommand();
              if (currentIntakeCmd == null)
                return;
              if (currentIntakeCmd == outakeBackCmd)
                commandScheduler.schedule(outakeBackTreadmillCmd);
              else if (currentIntakeCmd == intakeBackCmd)
                commandScheduler.schedule(intakeBackTreadmillCmd);
            }));

    driverController.povUp().and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch)
        .toggleOnTrue(driveAndManualShootCmd);
    driverController.povDown().and(() -> driveSub.getCurrentBotZone() == FieldZones.Launch)
        .toggleOnTrue(driveAndAutoShootCmd);

    driverController.povLeft().whileTrue(new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Left));
    driverController.povRight().whileTrue(new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Right));

    // Cancel all driveSub commands and disables xLock, returning manual control
    driverController.button(7).onTrue(Commands.runOnce(() -> driveSub.setXLock(false), driveSub));
    driverController.button(8)
        .toggleOnTrue(new ActiveXLockCmd(driveSub, this::getDriverVx, this::getDriverVy, this::getDriverOmega));

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
    // Pass
    operatorController.x().toggleOnTrue(
        Commands.either(zonePassCmd, Commands.none(),
            () -> driveSub.getCurrentBotZone() != FieldZones.Launch
                && driveSub.getCurrentBotZone() != FieldZones.Alliance));

    // operatorController.leftBumper().debounce(OIConstants.kButtonPressDebounceSeconds)
    // .and(operatorController.rightBumper()).and(() -> driveSub.getCurrentBotZone()
    // == FieldZones.Launch)
    // .toggleOnTrue(new XLockAndLaunchCmd(driveSub, indexerSub,
    // launcherAndIntakeSub));

    // operatorController.rightBumper().toggleOnTrue(outakeFrontCmd.alongWith(outakeFrontTreadmillCmd));
    // operatorController.leftBumper().toggleOnTrue(new SequentialCommandGroup(
    // new ParallelCommandGroup(
    // new XLockAndLaunchCmd(
    // driveSub,
    // indexerSub,
    // launcherAndIntakeSub).withDeadline(
    // Commands.waitUntil(LaunchHelpers::willHitHub)
    // .andThen(Commands.waitTime(AutoConstants.kAutoLaunch8Time))),
    // new ClimbDownCmd(climberSub)),
    // Commands.defer(
    // () -> PathGenerator.crossTrenchAuto(FieldConstants.kTrenchPathWaypoints),
    // Set.of(driveSub))));
    // operatorController.leftBumper().toggleOnTrue(intakeFrontCmd.alongWith(intakeFrontTreadmillCmd));
    // operatorController.rightBumper().toggleOnTrue(
    // new SequentialCommandGroup(
    // new XLockAndLaunchCmd(
    // driveSub,
    // indexerSub,
    // launcherAndIntakeSub).withDeadline(
    // Commands.waitUntil(LaunchHelpers::willHitHub)
    // .andThen(Commands.waitTime(AutoConstants.kAutoLaunch8Time))),
    // Commands.defer(
    // () -> PathGenerator.crossBumpAuto(FieldConstants.kBumpPathWaypoints),
    // Set.of(driveSub))));
    operatorController.rightBumper()
        .toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, () -> true,
            () -> LauncherAndIntakeConstants.kUnloadRPMSetpoint)
            .alongWith(new IntakeCmd(otbIntakeSub, () -> OTBIntakeConstants.kOuttakeSpeed)));

    // RPM setpoints for visionless backups
    operatorController.povUp().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kTowerRPMSetpoint));
    operatorController.povDown().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kTrenchRPMSetpoint));
    operatorController.povLeft().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kCornerRPMSetpoint));
    operatorController.povRight().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kBumpRPMSetpoint));
    operatorController.leftBumper().toggleOnTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, launchSupplier,
        () -> LauncherAndIntakeConstants.kJuggleRPMSetpoint));

    operatorController.button(7).onTrue(Commands.runOnce(launcherAndIntakeSub::stop, launcherAndIntakeSub));

    // Allow operator to apply RPM offset
    Trigger rpmTrimTrigger = new Trigger(() -> Math.abs(
        operatorController.getRightTriggerAxis()
            - operatorController.getLeftTriggerAxis()) > OIConstants.kTriggerDeadband);

    rpmTrimTrigger.whileTrue(Commands.run(() -> {
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
  }

  /** Run in {@code Robot.testInit()} */
  public void configureTestMode() {
    testController = new CommandXboxController(
        OIConstants.kTestControllerPort);
    final LoggedNetworkNumber testRPM = new LoggedNetworkNumber("/Tuning/testRPM", 1000);

    testController.a()
        .whileTrue(new LaunchAndIndexCmd(indexerSub, launcherAndIntakeSub, () -> true, () -> RPM.of(testRPM.get())));
  }

  // Helper methods to reduce repetition
  private double getDriverVx() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerYAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kTranslationSlowModeMultiplier : 1),
        OIConstants.kDeadband);
  }

  private double getDriverVy() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerXAxis)
            * (driverController.leftStick().getAsBoolean() ? OIConstants.kTranslationSlowModeMultiplier : 1),
        OIConstants.kDeadband);
  }

  private double getDriverOmega() {
    // Use low sensitivity by default, and use max speed when right stick is pushed
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerRotAxis)
            * (driverController.rightStick().getAsBoolean() ? 1 : OIConstants.kTurnSlowModeMultiplier),
        OIConstants.kDeadband);
  }

  // Use "super slow mode" when launching or passing
  private double getLaunchVx() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerYAxis)
            * OIConstants.kTranslationSuperSlowModeMultiplier,
        OIConstants.kDeadband);
  }

  private double getLaunchVy() {
    // Apply slow-mode when left stick is pushed down
    return MathUtil.applyDeadband(
        -driverController.getRawAxis(OIConstants.kDriverControllerXAxis)
            * OIConstants.kTranslationSuperSlowModeMultiplier,
        OIConstants.kDeadband);
  }

  public Gyro getGyro() {
    return gyro;
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("CalibrateGyro", new CalibrateGyroCmd(driveSub));

    autoChooser.addOption("Launch and Cross Bump Auto",
        new SequentialCommandGroup(
            new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
            new XLockAndLaunchCmd(
                driveSub,
                indexerSub,
                launcherAndIntakeSub).withDeadline(
                    Commands.waitUntil(LaunchHelpers::willHitHub)
                        .andThen(Commands.defer(
                            () -> Commands.waitTime(Seconds.of(
                                SmartDashboard.getNumber("8 Fuel Launch Time (Auto)",
                                    AutoConstants.kAutoLaunch8Time.in(Seconds)))),
                            Set.of()))),
            Commands.waitUntil(() -> false).until(
                () -> 20 - SmartDashboard.getNumber("Delayed Crossing Time (Auto)",
                    AutoConstants.kDefaultAutoDelay.in(Seconds)) >= DriverStation
                        .getMatchTime()),
            Commands.defer(
                () -> PathGenerator.crossBumpAuto(FieldConstants.kBumpPathWaypoints),
                Set.of(driveSub))));

    autoChooser.addOption("Launch and Cross Trench Auto",
        new SequentialCommandGroup(
            new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
            new ParallelCommandGroup(
                new XLockAndLaunchCmd(
                    driveSub,
                    indexerSub,
                    launcherAndIntakeSub).withDeadline(
                        Commands.waitUntil(LaunchHelpers::willHitHub)
                            .andThen(Commands.defer(
                                () -> Commands.waitTime(Seconds.of(
                                    SmartDashboard.getNumber("8 Fuel Launch Time (Auto)",
                                        AutoConstants.kAutoLaunch8Time.in(Seconds)))),
                                Set.of()))),
                new ClimbDownCmd(climberSub)),
            Commands.defer(
                () -> PathGenerator.crossTrenchAuto(FieldConstants.kTrenchPathWaypoints),
                Set.of(driveSub))));

    // autoChooser.addOption("Cross Bump Auto", Commands.defer(() ->
    // PathGenerator.crossBumpAuto(FieldConstants.kBumpPathWaypoints),Set.of(driveSub)));

    autoChooser.addOption("Launch", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new XLockAndLaunchCmd(
            driveSub,
            indexerSub,
            launcherAndIntakeSub)));

    autoChooser.addOption("Launch Then Climb Left", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new LaunchAndClimbCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub, TowerSide.Left)));

    autoChooser.addOption("Launch Then Climb Right", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new LaunchAndClimbCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub, TowerSide.Right)));

    // autoChooser.addOption("Align to Climb",
    // Commands.defer(
    // () -> PathGenerator.findL1ClimbPath(AutoConstants.crossingEndVelocity,
    // "Bump"),
    // Set.of(driveSub)));

    // autoChooser.addOption("Align to Tower Then Climb",
    // new NearestClimbCmd(driveSub, climberSub));

    // autoChooser.addOption("Align to Tower Left", new SequentialCommandGroup(
    // new AutoDeployIntakeCmd(driveSub),
    // new DriveToTowerSideCmd(driveSub, TowerSide.Left)));

    // autoChooser.addOption("Align to Tower Right", new SequentialCommandGroup(
    // new AutoDeployIntakeCmd(driveSub),
    // new DriveToTowerSideCmd(driveSub, TowerSide.Right)));

    autoChooser.addOption("Align to Tower Left & Climb", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Left)));

    autoChooser.addOption("Align to Tower Right & Climb", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveAndClimbCmd(driveSub, climberSub, TowerSide.Right)));

    autoChooser.addOption("Depot Launch and Climb", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DepotAndClimbCmd(driveSub, indexerSub, otbIntakeSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Depot Launch and Neutral Zone Trench", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DepotAndNeutralZoneTrenchCmd(driveSub, indexerSub, otbIntakeSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Depot Launch and Neutral Zone Bump", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DepotAndNeutralZoneBumpCmd(driveSub, indexerSub, otbIntakeSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Outpost Launch and Climb", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new OutpostAndClimbCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Outpost Launch and Neutral Zone Trench", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new OutpostAndNeutralZoneTrenchCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Outpost Launch and Neutral Zone Bump", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new OutpostAndNeutralZoneBumpCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Launch and Delayed Neutral Zone Trench", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new LaunchAndDelayedNeutralZoneTrenchCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Launch and Delayed Neutral Zone Bump", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new LaunchAndDelayedNeutralZoneBumpCmd(driveSub, indexerSub, launcherAndIntakeSub, climberSub)));

    autoChooser.addOption("Drive to Corner Depot and Trench", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveToCornerDepotTrenchCmd(driveSub)));

    autoChooser.addOption("Drive to Corner Outpost and Trench", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveToCornerOutpostTrenchCmd(driveSub)));

    autoChooser.addOption("Drive to Corner Depot and Bump", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveToCornerDepotBumpCmd(driveSub)));

    autoChooser.addOption("Drive to Corner Outpost and Bump", new SequentialCommandGroup(
        new AutoDeployIntakeCmd(driveSub, otbIntakeSub),
        new DriveToCornerOutpostBumpCmd(driveSub)));

    autoChooser.addOption("Deploy intake", new AutoDeployIntakeCmd(driveSub, otbIntakeSub));

    autoChooser.addOption("Forward10Seconds",
        Commands.run(() -> driveSub.runVelocity(new ChassisSpeeds(0.4, 0, 0), false, false, false))
            .withTimeout(Seconds.of(15)).andThen(new DriveStopCmd(driveSub)));

    SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    // // if (!selectedAuto.hasRequirement(climberSub)) {
    // return selectedAuto.alongWith(new ClimbDownCmd(climberSub)); // This line is
    // causing the code to crash when
    // // autonomous phase runs twice. Prevent this by
    // // testing autos using the test controller
    // }
    Logger.recordOutput("Drivetrain/SelectedAuto", selectedAuto == null ? "Null" : selectedAuto.getName());
    return selectedAuto;

  }

}