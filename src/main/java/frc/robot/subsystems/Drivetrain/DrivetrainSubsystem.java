// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonConstants;
import frc.utils.PoseHelpers;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
  private final Gyro gyro;
  private boolean gyroDisconnected = false;
  private boolean isFieldRelative = true;
  private final Debouncer gyroDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // Pose estimation with vision fusion capability
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveDrivePoseEstimator visionlessOdometry;
  private final PhotonCamera[] cameras = new PhotonCamera[PhotonConstants.numCameras];
  private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[PhotonConstants.numCameras];

  // Current pose of the robot
  private Pose2d robotPose = new Pose2d();
  private Pose2d visionlessPose = new Pose2d();

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
    odometry = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry standard deviations
        VecBuilder.fill(0.4, 0.4, 0.4)); // Vision standard deviations
    visionlessOdometry = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d());

    // Setup cameras to see april tags. Wow! That makes me really happy.
    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      cameras[i] = new PhotonCamera(PhotonConstants.kCameras[i]);
      photonPoseEstimators[i] = new PhotonPoseEstimator(FieldConstants.kfieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PhotonConstants.kRobotToCams[i]);
    }

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
    return odometry.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to a specific pose.
   * 
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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

  public Rotation2d getTargetHeading() {
    return this.targetHeading;
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

  /**
   * Get an estimate of the robot's pose using vision data from PhotonVisision.
   * This method will filter out invalid and unprobable results.
   * 
   * @param poseEstimator          The PhotonPoseEstimator object to use for
   *                               estimating the robot's pose
   * @param camera                 The PhotonCamera object to use for getting
   *                               vision data
   * @param robotToCam             The Transform3d object representing the
   *                               transformation
   *                               from the robot to the camera
   * @param prevEstimatedRobotPose The previous estimated pose object
   */
  public List<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator, PhotonCamera camera,
      Transform3d robotToCam,
      Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    List<EstimatedRobotPose> results = new ArrayList<>();
    List<PhotonPipelineResult> camResults = camera.getAllUnreadResults();

    for (PhotonPipelineResult camResult : camResults) {
      if (!camResult.hasTargets()) {
        continue;
      }

      // check if the built in pose estimator pose is reasonable
      Optional<EstimatedRobotPose> optionalEstimation = poseEstimator.update(camResult);
      if (optionalEstimation.isPresent()) {
        EstimatedRobotPose estimation = optionalEstimation.get();

        Logger.recordOutput("Vision/" + camera.getName() + "/RawEstimatedPose", estimation.estimatedPose);

        if (PoseHelpers.isInField(estimation.estimatedPose) &&
            PoseHelpers.isOnGround(estimation.estimatedPose, PhotonConstants.kHeightTolerance)) {

          // ignore the result if it only has one tag and the tag is too small
          if (camResult.getTargets().size() == 1
              && camResult.getTargets().get(0).area <= PhotonConstants.kMinSingleTagArea) {
            continue;
          }

          results.add(estimation);
          continue;
        }
      }

      double timestamp = camResult.getTimestampSeconds();
      List<PhotonTrackedTarget> targetsUsed = new ArrayList<>();

      // if the built in pose estimator is not reasonable, compute it ourselves
      if (camResult.hasTargets()) {
        List<PhotonTrackedTarget> targets = camResult.getTargets();
        List<Pose3d> validPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {

          // ignore targets with high pose ambiguity
          if (target.getPoseAmbiguity() > PhotonConstants.kAmbiguityDiscardThreshold) {
            continue;
          }

          int targetId = target.fiducialId;
          Optional<Pose3d> optionalTagPose = FieldConstants.kfieldLayout.getTagPose(targetId);
          // it should never be empty, but just in case
          if (optionalTagPose.isEmpty()) {
            continue;
          }
          Pose3d tagPose = optionalTagPose.get();

          // if the ambiguity is high, only use the pose that is reasonable
          if (target.getPoseAmbiguity() > PhotonConstants.kAmbiguityThreshold) {
            Transform3d bestCamToTarget = target.getBestCameraToTarget();
            Transform3d altCamToTarget = target.getAlternateCameraToTarget();

            // robotTransform = tagTransform - camToTarget - robotToCam
            Pose3d bestRobotPose = tagPose.transformBy(bestCamToTarget.inverse())
                .transformBy(robotToCam.inverse());
            Pose3d altRobotPose = tagPose.transformBy(altCamToTarget.inverse()).transformBy(robotToCam.inverse());

            Logger.recordOutput("Vision/" + camera.getName() + "/FallbackBestPose", bestRobotPose);
            Logger.recordOutput("Vision/" + camera.getName() + "/FallbackAltPose", altRobotPose);

            // check if they are reasonable
            boolean isBestPoseValid = PoseHelpers.isInField(bestRobotPose) &&
                PoseHelpers.isOnGround(bestRobotPose, PhotonConstants.kHeightTolerance);
            boolean isAltPoseValid = PoseHelpers.isInField(altRobotPose)
                && PoseHelpers.isOnGround(altRobotPose, PhotonConstants.kHeightTolerance);
            if (isBestPoseValid && isAltPoseValid) {
              targetsUsed.add(target);
              // if both are valid, use the one that is closer to the previous estimation
              double bestDistance = PoseHelpers.distanceBetween(bestRobotPose, new Pose3d(prevEstimatedRobotPose));
              double altDistance = PoseHelpers.distanceBetween(altRobotPose, new Pose3d(prevEstimatedRobotPose));
              if (bestDistance < altDistance) {
                validPoses.add(bestRobotPose);
              } else {
                validPoses.add(altRobotPose);
              }
            } else if (isBestPoseValid) {
              targetsUsed.add(target);
              validPoses.add(bestRobotPose);
            } else if (isAltPoseValid) {
              targetsUsed.add(target);
              validPoses.add(altRobotPose);
            }
            continue;
          }

          // if the ambiguity is low, use the pose directly
          Transform3d camToTarget = target.getBestCameraToTarget();
          // robotTransform = tagTransform - camToTarget - robotToCam
          Pose3d robotPose = tagPose.transformBy(camToTarget.inverse()).transformBy(robotToCam.inverse());
          // check if the pose is reasonable
          if (PoseHelpers.isInField(robotPose) && PoseHelpers.isOnGround(robotPose, PhotonConstants.kHeightTolerance)) {
            validPoses.add(robotPose);
            targetsUsed.add(target);
          }
        }

        // if there are no valid poses, ignore this frame
        if (validPoses.isEmpty()) {
          continue;
        }

        double totalX = 0;
        double totalY = 0;
        double totalZRot = 0;

        for (Pose3d pose : validPoses) {
          totalX += pose.getX();
          totalY += pose.getY();

          totalZRot += pose.getRotation().getZ();
        }

        final int count = validPoses.size();
        Pose3d averagePose = new Pose3d(
            totalX / count, // X average
            totalY / count, // Y average
            0, // Z forced to 0 (validated by isOnGround)
            new Rotation3d(0, 0, totalZRot / count) // Average Z rotation
        );

        Logger.recordOutput("Vision/" + camera.getName() + "/FallbackPose", averagePose);

        // ignore the result if it only has one tag and the tag is too small
        if (camResult.getTargets().size() == 1
            && camResult.getTargets().get(0).area <= PhotonConstants.kMinSingleTagArea) {
          continue;
        }
        results
            .add(new EstimatedRobotPose(averagePose, timestamp, targetsUsed,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE));
      }
    }
    return results;
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

    // Get current states
    SwerveModuleState[] states = getModuleStates();
    SwerveModulePosition[] positions = getModulePositions();

    // Update pose estimator with odometry
    if (!gyroDisconnected) {
      robotPose = odometry.update(gyro.getRotation2d(), positions);
      visionlessPose = visionlessOdometry.update(gyro.getRotation2d(), positions);
    }

    // Iterate through each camera
    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      // Get all poses from camera
      List<EstimatedRobotPose> visionPoses = getEstimatedGlobalPose(photonPoseEstimators[i], cameras[i],
          PhotonConstants.kRobotToCams[i],
          robotPose);

      List<Integer> fiducialIds = new ArrayList<>();
      List<Pose3d> fiducialIdPoses = new ArrayList<>();
      List<Double> tagAreas = new ArrayList<>();
      List<Double> tagAmbiguities = new ArrayList<>();

      // Iterate through each pose to check for ambiguity
      for (EstimatedRobotPose visionPose : visionPoses) {
        // Iterate through the targets used to estimate the pose
        for (PhotonTrackedTarget target : visionPose.targetsUsed) {
          fiducialIds.add(target.fiducialId);
          if (FieldConstants.kfieldLayout.getTagPose(target.fiducialId).isPresent()) {
            fiducialIdPoses.add(FieldConstants.kfieldLayout.getTagPose(target.fiducialId).get());
          }
          tagAreas.add(target.area);
          tagAmbiguities.add(target.poseAmbiguity);
        }

        Pose2d estimatedPose = PoseHelpers.toPose2d(visionPose.estimatedPose);
        odometry.addVisionMeasurement(estimatedPose, visionPose.timestampSeconds);

        // Log vision poses
        Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/EstimatedPose", visionPose.estimatedPose);
        Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/Timestamp", visionPose.timestampSeconds);

      }

      // Log targets estimated from robot
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/TargetIds",
          fiducialIds.stream().mapToInt(n -> n).toArray());
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/TargetPoses",
          fiducialIdPoses.toArray(new Pose3d[fiducialIdPoses.size()]));
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/TargetAreas",
          tagAreas.stream().mapToDouble(n -> n).toArray());
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/TargetAmbiguities",
          tagAmbiguities.stream().mapToDouble(n -> n).toArray());
    }

    // Log everything
    Logger.recordOutput("Drivetrain/GyroDisconnected", gyroDisconnected);
    Logger.recordOutput("Drivetrain/IsFieldRelative", isFieldRelative);
    Logger.recordOutput("Drivetrain/Pose", robotPose);
    Logger.recordOutput("Drivetrain/Rotation", gyro.getRotation2d().getDegrees());
    Logger.recordOutput("Drivetrain/Swerve/Module/State", states);
    Logger.recordOutput("Drivetrain/Swerve/Module/Position", positions);

    if (this.targetHeading != null) {
      Logger.recordOutput("Drivetrain/TargetHeading", this.getTargetHeading().getDegrees());
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