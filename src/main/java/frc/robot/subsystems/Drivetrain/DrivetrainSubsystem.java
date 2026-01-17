// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
  private final Gyro gyro;
  private boolean gyroDisconnected = false;
  private boolean isFieldRelative = true;
  private final Debouncer gyroDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // Pose estimation with vision fusion capability
  private final SwerveDrivePoseEstimator poseEstimator;

  // Simulation
  private SwerveDriveSimulation simulatedSwerveDrive = null;

  // Heading control for restricted driving
  private Rotation2d targetHeading = null;
  private final PIDController headingController;

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
  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
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
    runVelocity(speeds, this.isFieldRelative);
  }

  /**
   * Gets the current estimated pose of the robot.
   * 
   * @return Current estimated pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to a specific pose.
   * 
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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
   * Gets the omega correction to maintain the target heading.
   * 
   * @param desiredHeading The desired heading to maintain
   * @return The angular velocity correction
   */
  public AngularVelocity getHeadingCorrectionOmega(Rotation2d desiredHeading) {
    Rotation2d currentHeading = gyro.getRotation2d();
    return RadiansPerSecond.of(
        headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians()));
  }

  /**
   * Gets the current chassis speeds.
   * 
   * @return Current chassis speeds (robot-relative)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the current used gyro
   * 
   * @return Gyro object
   */
  public Gyro getGyro() {
    return gyro;
  }

  @Override
  public void periodic() {
    // Check gyro connection with debouncing
    if (!gyroDebouncer.calculate(gyro.isConnected())) {
      gyroDisconnected = true;
      isFieldRelative = false;
    } else {
      gyroDisconnected = false;
      isFieldRelative = true;
    }

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
    Logger.recordOutput("Drivetrain/IsFieldRelative", isFieldRelative);
    Logger.recordOutput("Drivetrain/Pose", currentPose);
    Logger.recordOutput("Drivetrain/Rotation", gyro.getRotation2d().getDegrees());
    Logger.recordOutput("Swerve/Module/State", states);
    Logger.recordOutput("Swerve/Module/Position", positions);

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