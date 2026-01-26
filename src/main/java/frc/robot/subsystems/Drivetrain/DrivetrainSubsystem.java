// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.GregorianCalendar;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimulationConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
  private final Gyro gyro;
  private boolean gyroDisconnected = false;
  private boolean isFieldRelativeDesired = true;
  private boolean isFieldRelativeReal = !gyroDisconnected && isFieldRelativeDesired;
  private final Debouncer gyroDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // Pose estimation with vision fusion capability
  private final SwerveDrivePoseEstimator poseEstimator;

  // Simulation
  private SwerveDriveSimulation simulatedSwerveDrive = null;
  // Indicates whether getPose() should use real odometry or report actual
  // simulated pose, for testing purposes
  private boolean accurateSimOdometry = true;

  // Heading control for restricted driving
  private Rotation2d targetHeading = null;
  private final PIDController headingController;

  // Controller for radial distance from hub
  private final PIDController radialController;

  /**
   * Creates a new DrivetrainSubsystem.
   * 
   * @param modules Array of swerve modules [FL, FR, BL, BR]
   * @param gyro    Gyro sensor for orientation
   */
  public DrivetrainSubsystem(SwerveModule[] modules, Gyro gyro) {
    for (int i = 0; i < modules.length; i++) {
      this.modules[i] = modules[i];
    }
    this.gyro = gyro;

    // Initialize pose estimator with standard deviations
    // First vector: odometry std devs (x, y, theta) - lower = more trust
    // Second vector: vision std devs (x, y, theta) - higher = less trust initially
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry standard deviations
        VecBuilder.fill(0.4, 0.4, 0.4)); // Vision standard deviations

    // Initialize heading controller for auto-rotation in restricted mode
    headingController = new PIDController(
        Constants.DriveConstants.kPIDHeadingControllerP,
        Constants.DriveConstants.kPIDHeadingControllerI,
        Constants.DriveConstants.kPIDHeadingControllerD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Math.toRadians(Constants.DriveConstants.kPIDHeadingControllerTolerance));

    radialController = new PIDController(
        DriveConstants.kPIDRadialControllerP,
        DriveConstants.kPIDRadialControllerI,
        DriveConstants.kPIDRadialControllerD);
    radialController.setTolerance(DriveConstants.kPIDRadialControllerTolerance.in(Meters));

    // Do not use the auto generated robot config to allow for muiltiple profiles
    RobotConfig config = DriveConstants.kRobotConfig;

    // Set the robot's parameters for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeedsRobotRelative,
        (speeds, feedforwards) -> runVelocity(speeds, false),
        new PPHolonomicDriveController(
            new PIDConstants(DriveConstants.kPTranslationController, DriveConstants.kITranslationController,
                DriveConstants.kDTranslationController),
            new PIDConstants(DriveConstants.kPThetaController, DriveConstants.kIThetaController,
                DriveConstants.kDThetaController)),
        config,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false; // default to blue
        },
        this);
  }

  /**
   * Creates a new DrivetrainSubsystem with simulation support.
   * 
   * @param modules              Array of swerve modules
   * @param gyro                 Gyro sensor
   * @param simulatedSwerveDrive Simulation object
   */
  public DrivetrainSubsystem(SwerveModule[] modules, Gyro gyro, SwerveDriveSimulation simulatedSwerveDrive) {
    this(modules, gyro);
    this.simulatedSwerveDrive = simulatedSwerveDrive;

    SmartDashboard.putBoolean("Accurate Sim Odometry", accurateSimOdometry);
    SmartDashboard.putNumber("New Sim Pose X", SimulationConstants.kStartingPose.getX());
    SmartDashboard.putNumber("New Sim Pose Y", SimulationConstants.kStartingPose.getY());
    SmartDashboard.putNumber("New Sim Pose θ (Deg)", SimulationConstants.kStartingPose.getRotation().getDegrees());
    // Display this value as a toggle button or toggle switch in Elastic to use it
    // as a button
    SmartDashboard.putBoolean("Apply Sim Pose", true);
    resetPose(SimulationConstants.kStartingPose);
  }

  /**
   * Sets desired states for all swerve modules.
   * 
   * @param desiredStates Array of desired module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxWheelSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Runs the drivetrain at specified velocities.
   * 
   * @param speeds          Desired chassis speeds
   * @param isFieldRelative Whether speeds are field-relative
   */
  public void runVelocity(ChassisSpeeds speeds, boolean isFieldRelative) {
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          getPose().getRotation());
    }
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxWheelSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    Logger.recordOutput("Swerve/Module/Setpoint", states);
    Logger.recordOutput("Swerve/Gyro", gyro.getRotation2d());
  }

  /**
   * Runs the drivetrain using the current field-relative setting.
   * 
   * @param speeds Desired chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, this.isFieldRelativeReal);
  }

  /**
   * Gets the current estimated pose of the robot.
   * 
   * @return Current estimated pose
   */
  public Pose2d getPose() {
    if (!accurateSimOdometry && simulatedSwerveDrive != null) {
      return simulatedSwerveDrive.getSimulatedDriveTrainPose();
    } else {
      return poseEstimator.getEstimatedPosition();
    }
  }

  /**
   * Resets the pose estimator to a specific pose.
   * 
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);

    if (simulatedSwerveDrive != null && !accurateSimOdometry) {
      simulatedSwerveDrive.setSimulationWorldPose(pose);
    }

    Logger.recordOutput("Drivetrain/PoseReset", pose);
  }

  /**
   * Resets the pose estimator to the origin (0, 0, 0°).
   */
  public void resetPose() {
    resetPose(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()));
  }

  /**
   * Gets all module positions.
   * 
   * @return Array of module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets all module states.
   * 
   * @return Array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Gets the robot-relative chassis speeds.
   * 
   * @return Robot-relative chassis speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets a target heading for the robot to maintain.
   * Used in restricted drive mode.
   * 
   * @param heading The target heading
   */
  public void setTargetHeading(Rotation2d heading) {
    this.targetHeading = heading;
  }

  /**
   * Clears the target heading.
   */
  public void clearTargetHeading() {
    this.targetHeading = null;
  }

  /**
   * Resets the heading and radial controllers.
   */
  public void resetControllers() {
    headingController.reset();
    radialController.reset();
  }

  /**
   * Gets the omega correction to maintain the target heading.
   * 
   * @param desiredHeading The desired heading to maintain
   * @return The angular velocity correction
   */
  public AngularVelocity getHeadingCorrectionOmega(Rotation2d desiredHeading) {
    Rotation2d currentHeading = getPose().getRotation();
    return RadiansPerSecond.of(
        headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians()));
  }

  /**
   * Gets the translation vector from the robot to the current alliance hub.
   * 
   * @return Translation2d vector pointing from robot to hub
   */
  public Translation2d getHubTranslation2dBotRelative() {
    Pose2d currentPose = getPose();
    Translation2d hubTranslation;

    // Determine hub position based on alliance
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      hubTranslation = Constants.FieldConstants.kRedHubPose;
    } else {
      hubTranslation = Constants.FieldConstants.kBlueHubPose;
    }

    Translation2d directionToHub = hubTranslation.minus(currentPose.getTranslation());
    return directionToHub;
  }

  /**
   * Gets the linear velocity correction to maintain a desired radial distance
   * from the hub.
   * 
   * @param desiredDistance The desired distance from the hub
   * @return The linear velocity correction
   */
  public LinearVelocity getRadialDistanceCorrectionVelocity(Distance desiredDistance) {
    Translation2d hubTranslation = getHubTranslation2dBotRelative();

    double currentDistance = hubTranslation.getNorm();
    return MetersPerSecond.of(MathUtil.clamp(
        -radialController.calculate(currentDistance, desiredDistance.in(Meters)),
        -DriveConstants.kMaxSpeed.in(MetersPerSecond),
        DriveConstants.kMaxSpeed.in(MetersPerSecond)));
  }

  /**
   * Gets the velocity vector correction to maintain a desired radial distance
   * from the hub.
   * 
   * @param desiredDistance The desired distance from the hub
   * @return The a Translation2d representing the correction vector
   */
  public Translation2d getRadialDistanceCorrectionVector(Distance desiredDistance) {
    Translation2d hubTranslation = getHubTranslation2dBotRelative();

    hubTranslation = hubTranslation.div(hubTranslation.getNorm()); // Normalize to unit vector
    double correctionMagnitude = getRadialDistanceCorrectionVelocity(desiredDistance).in(MetersPerSecond);

    return hubTranslation.times(correctionMagnitude);
  }

  /**
   * Checks if the robot is within the accepted shooting zone.
   * 
   * @return True if in shooting zone, false otherwise
   */
  public boolean isBotInShootingZone() {
    Pose2d pose = getPose();
    return isInShootingZone(pose);
  }

  /**
   * Checks if the given pose is within the accepted shooting zone.
   * 
   * @param pose The pose to check
   * @return True if in shooting zone, false otherwise
   */
  public boolean isInShootingZone(Pose2d pose) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return pose.getX() < FieldConstants.kAcceptedShootingZone.in(Meters);
    } else {
      return pose.getX() > (FieldConstants.kFieldLengthX.minus(FieldConstants.kAcceptedShootingZone).in(Meters));
    }
  }

  /**
   * Gets the current chassis speeds.
   * 
   * @return Current chassis speeds (robot-relative)
   */
  public ChassisSpeeds getChassisSpeedsRobotRelative() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the current used gyro
   * 
   * Do not use this to get the robot heading, as the gyro offset is applied later
   * in the pose estimator,
   * use getPose().getRotation() instead
   * 
   * @return Gyro object
   */
  public Gyro getGyro() {
    return gyro;
  }

  /**
   * Toggles between field-relative and robot-relative control.
   */
  public void toggleFieldRelative() {
    isFieldRelativeDesired = !isFieldRelativeDesired;
  }

  @Override
  public void periodic() {
    // Check gyro connection with debouncing
    if (!gyroDebouncer.calculate(gyro.isConnected())) {
      gyroDisconnected = true;
    } else {
      gyroDisconnected = false;
    }

    // Update drive mode based on desired setting and gyro status
    isFieldRelativeReal = !gyroDisconnected && isFieldRelativeDesired;

    // Update pose estimator with odometry
    if (!gyroDisconnected) {
      poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    }

    // Get current states and pose
    SwerveModuleState[] states = getModuleStates();
    SwerveModulePosition[] positions = getModulePositions();
    Pose2d currentPose = getPose();

    // Log everything
    Logger.recordOutput("Drivetrain/GyroDisconnected", gyroDisconnected);
    Logger.recordOutput("Drivetrain/IsFieldRelativeReal", isFieldRelativeReal);
    Logger.recordOutput("Drivetrain/IsFieldRelativeDesired", isFieldRelativeDesired);
    Logger.recordOutput("Drivetrain/Pose", currentPose);
    Logger.recordOutput("Drivetrain/Rotation", gyro.getRotation2d().getDegrees());
    Logger.recordOutput("Drivetrain/Swerve/Module/State", states);
    Logger.recordOutput("Drivetrain/Swerve/Module/Position", positions);

    // Retrieve SmartDashboard settings
    if (simulatedSwerveDrive != null) {
      accurateSimOdometry = SmartDashboard.getBoolean("Accurate Sim Odometry", accurateSimOdometry);
      boolean applyNewSimPose = !SmartDashboard.getBoolean("Apply Sim Pose", true);

      // Apply new sim pose if requested, then reset the trigger
      if (applyNewSimPose) {
        SmartDashboard.putBoolean("Apply Sim Pose", true);
        double x = SmartDashboard.getNumber("New Sim Pose X", simulatedSwerveDrive.getSimulatedDriveTrainPose().getX());
        double y = SmartDashboard.getNumber("New Sim Pose Y", simulatedSwerveDrive.getSimulatedDriveTrainPose().getY());
        double theta = SmartDashboard.getNumber("New Sim Pose θ (Deg)",
            simulatedSwerveDrive.getSimulatedDriveTrainPose().getRotation().getDegrees());
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
        simulatedSwerveDrive.setSimulationWorldPose(newPose);
        resetPose(newPose);
      }
    }

    if (this.targetHeading != null) {
      Logger.recordOutput("Drivetrain/TargetHeading", this.targetHeading.getDegrees());
    }
  }

  @Override
  public void simulationPeriodic() {
    if (this.simulatedSwerveDrive != null) {
      Logger.recordOutput("FieldSimulation/PhysicalRobotPose",
          this.simulatedSwerveDrive.getSimulatedDriveTrainPose());
    }
  }
}