// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Vector;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.util.PoseHelpers;
import frc.robot.util.swerve.SwerveConfig;
import frc.robot.util.vision.CameraProfile;
import frc.robot.util.vision.VisionStdDevCalculator;
import frc.robot.util.swerve.FieldZones;

public class DrivetrainSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private static final SwerveDriveKinematics kinematics = SwerveConfig.kDriveKinematics;
  private final Gyro gyro;
  private boolean gyroDisconnected = false;
  private boolean isFieldRelativeDesired = true;
  private boolean isFieldRelativeReal = !gyroDisconnected && isFieldRelativeDesired;
  private final Debouncer gyroDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
  private final Field2d field = new Field2d(); // make Field2d to put on the DriverStation

  // Low pas filters for velocity logging
  private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.15, 0.02);
  private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.15, 0.02);
  private final LinearFilter omegaFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  // Pose estimation with vision fusion capability
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDrivePoseEstimator visionlessPoseEstimator;
  private final PhotonCamera[] cameras = new PhotonCamera[PhotonConstants.numCameras];
  private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[PhotonConstants.numCameras];
  private final VisionSystemSim simulatedVision;

  // Current pose of the robot
  private Pose2d robotPose = new Pose2d();
  private Pose2d visionlessPose = new Pose2d();

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

  // X and Y translation controllers
  private final PIDController xController, yController;

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
    visionlessPoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getModulePositions(),
        new Pose2d());

    if (Robot.isSimulation()) {
      simulatedVision = new VisionSystemSim("main");
      simulatedVision.addAprilTags(FieldConstants.kfieldLayout);
    } else {
      simulatedVision = null;
    }

    // Setup cameras to see april tags. Wow! That makes me really happy.
    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      CameraProfile currentProfile = PhotonConstants.kCameras[i];
      cameras[i] = new PhotonCamera(currentProfile.name());
      photonPoseEstimators[i] = new PhotonPoseEstimator(FieldConstants.kfieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          currentProfile.getRobotToCameraTransform());

      if (Robot.isSimulation()) {
        if (simulatedVision == null) {
          throw new IllegalStateException(
              "simulatedVision is null... Please ensure the VisionSystemSim is initialized when simulating the robot.");
        }
        // try {
        System.out.println(currentProfile.calibrationFile().getPath());
        // SimCameraProperties simulatedProperties = new
        // SimCameraProperties(currentProfile.calibrationFile().getPath(),
        // currentProfile.resolution()[0], currentProfile.resolution()[1]);
        SimCameraProperties simulatedProperties = new SimCameraProperties();
        simulatedProperties.setCalibration(currentProfile.resolution()[0], currentProfile.resolution()[1],
            Rotation2d.fromDegrees(70));
        simulatedProperties.setFPS(30);
        simulatedProperties.setAvgLatencyMs(35);
        simulatedProperties.setLatencyStdDevMs(5);
        PhotonCameraSim simulatedCamera = new PhotonCameraSim(cameras[i], simulatedProperties);
        simulatedVision.addCamera(simulatedCamera, currentProfile.getRobotToCameraTransform());
        // } catch (IOException e) {
        // System.err.println(
        // "Could not read camera configuration file to simulate camera " + i + ": " +
        // e.getMessage());
        // }
      }

    }

    // Initialize Controllers
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

    xController = new PIDController(
        DriveConstants.kPIDXControllerP,
        DriveConstants.kPIDXControllerI,
        DriveConstants.kPIDXControllerD);
    xController.setTolerance(DriveConstants.kPIDXControllerTolerance.in(Meters));

    yController = new PIDController(
        DriveConstants.kPIDYControllerP,
        DriveConstants.kPIDYControllerI,
        DriveConstants.kPIDYControllerD);
    yController.setTolerance(DriveConstants.kPIDYControllerTolerance.in(Meters));

    // Debug feature to teleport bot odometry
    SmartDashboard.putNumber("New Pose X", getPose().getX());
    SmartDashboard.putNumber("New Pose Y", getPose().getY());
    SmartDashboard.putNumber("New Pose θ (Deg)", getPose().getRotation().getDegrees());
    // Display this value as a toggle button or toggle switch in Elastic to use it
    // as a button
    SmartDashboard.putBoolean("Apply Pose", true);

    // Do not use the auto generated robot config to allow for muiltiple profiles
    RobotConfig config = SwerveConfig.kRobotConfig;

    // Set the robot's parameters for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeedsRobotRelative,
        (speeds, feedforwards) -> runVelocity(speeds, false, false),
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
    resetPose(SimulationConstants.kStartingPose);
  }

  /**
   * Sets desired states for all swerve modules.
   * 
   * @param desiredStates Array of desired module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.kMaxWheelSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Runs the drivetrain at specified velocities.
   * 
   * @param speeds          Desired chassis speeds
   * @param isFieldRelative Whether speeds are field-relative
   * @param isManualControl Whether the control is manual (should an offset be
   *                        applied for red alliance)
   */
  public void runVelocity(ChassisSpeeds speeds, boolean isFieldRelative, boolean isManualControl) {
    runVelocity(speeds, isFieldRelative, isManualControl, isManualControl);
  }

  /**
   * Runs the drivetrain at specified velocities.
   * 
   * @param speeds          Desired chassis speeds
   * @param isFieldRelative Whether speeds are field-relative
   * @param isManualX       Whether the x velocity is manually controlled (should
   *                        an inversion be applied for red alliance)
   * @param isManualY       Whether the y velocity is manually controlled (should
   *                        an inversion be applied for red alliance)
   */
  public void runVelocity(ChassisSpeeds speeds, boolean isFieldRelative, boolean isManualX, boolean isManualY) {
    if (isFieldRelative) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      boolean isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond * (isRedAlliance && isManualX ? -1 : 1),
          speeds.vyMetersPerSecond * (isRedAlliance && isManualY ? -1 : 1),
          speeds.omegaRadiansPerSecond,
          getPose().getRotation());
    }
    SwerveModuleState[] states = SwerveConfig.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConfig.kMaxWheelSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    Logger.recordOutput("Swerve/Module/Setpoint", states);
    Logger.recordOutput("Swerve/Gyro", gyro.getRotation2d());
  }

  /**
   * Runs the drivetrain using the current field-relative setting. With manual
   * control.
   * Runs the drivetrain using the current field-relative setting. With manual
   * control.
   * 
   * @param speeds Desired chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, this.isFieldRelativeReal, true);
    runVelocity(speeds, this.isFieldRelativeReal, true);
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
   * Set the X setpoint in field relative coordinates (Blue origin, regardless of
   * alliance)
   * 
   * @param setpoint The desired X coordinate
   */
  public void setXSetpoint(Distance setpoint) {
    xController.setSetpoint(setpoint.in(Meters));
  }

  /**
   * Set the Y setpoint in field relative coordinates (Blue origin, regardless of
   * alliance)
   * 
   * @param setpoint The desired X coordinate
   */
  public void setYSetpoint(Distance setpoint) {
    yController.setSetpoint(setpoint.in(Meters));
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
   * Calculates the nearest acceptable bot heading from a desired locked angle.
   * 
   * For example, if locked angle is 45 degrees and force off is true, the
   * returned angle will be the
   * nearest heading among the following options:
   * 45, -45, 135, or -135
   * 
   * @param lockedAngle The heading the bot should be locked at
   * @param forceOdd    Whether the returned heading must be an odd multiple
   * @return The nearest acceptable heading
   */
  public Rotation2d getNearestTargetAngle(Rotation2d lockedAngle, boolean forceOdd) {
    // Get current robot heading in radians
    double currentAngleRadians = getPose().getRotation().getRadians();

    // Calculate nearest ODD multiple of the locked angle
    double angleIncrement = lockedAngle.getRadians();

    // Divide by increment, round to nearest integer, then make it odd
    int multiple = (int) Math.round(currentAngleRadians / angleIncrement);

    // Force to nearest odd number: if even, add 1
    if (forceOdd && multiple % 2 == 0) {
      multiple += (currentAngleRadians > 0) ? 1 : -1;
    }

    double nearestAngle = multiple * angleIncrement;

    return Rotation2d.fromRadians(nearestAngle);
  }

  /**
   * Resets the heading and radial controllers.
   */
  public void resetControllers() {
    headingController.reset();
    radialController.reset();
    xController.reset();
    yController.reset();
  }

  /**
   * Gets the vx correction to maintain the x setpoint, if using both x and y
   * controllers, call getChassisSpeedsCorrection() and get the components instead
   * to ensure max speed is correct
   * 
   * @param maxSpeed The maximum allowed speed
   * @return The x velocity to maintain the current setpoint
   */
  public LinearVelocity getXCorrectionMetersPerSecond(LinearVelocity maxSpeed) {
    return maxSpeed.times(xController.calculate(getPose().getX()));
  }

  /**
   * Gets the vx correction to maintain the x setpoint, if using both x and y
   * controllers, call getChassisSpeedsCorrection() and get the components instead
   * to ensure max speed is correct
   * 
   * @param maxSpeed The maximum allowed speed
   * @return The x velocity to maintain the current setpoint
   */
  public LinearVelocity getYCorrectionMetersPerSecond(LinearVelocity maxSpeed) {
    return maxSpeed.times(yController.calculate(getPose().getY()));
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
   * Returns a ChassisSpeeds object to maintaint the desired x and y setpoints, as
   * well as the desired heading, while respecting a maxSpeed
   * 
   * @param desired  The desired heading to maintain
   * @param maxSpeed The maximum allowed speed
   * @return
   */
  public ChassisSpeeds getChassisSpeedsCorrection(Rotation2d desired, LinearVelocity maxSpeed) {
    Translation2d velocity = new Translation2d(
        maxSpeed.times(xController.calculate(getPose().getX())).in(MetersPerSecond),
        maxSpeed.times(yController.calculate(getPose().getY())).in(MetersPerSecond));

    if (velocity.getNorm() > maxSpeed.in(MetersPerSecond)) {
      velocity.div(velocity.getNorm()).times(maxSpeed.in(MetersPerSecond));
    }

    return new ChassisSpeeds(velocity.getX(), velocity.getY(), getHeadingCorrectionOmega(desired).in(RadiansPerSecond));
  }

  /**
   * Gets the translation vector from the robot to the current alliance hub.
   * 
   * @return Translation2d vector pointing from robot to hub
   */
  public Translation2d getHubTranslation2dBotRelative() {
    Pose2d currentPose = getPose();

    // Determine hub position based on alliance, default to blue
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
    Translation2d hubTranslation = isRedAlliance
        ? Constants.FieldConstants.kRedHubPose
        : Constants.FieldConstants.kBlueHubPose;

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
        -SwerveConfig.kMaxSpeed.in(MetersPerSecond),
        SwerveConfig.kMaxSpeed.in(MetersPerSecond)));
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

    double norm = hubTranslation.getNorm();
    if (norm < 1e-6) {
      // Robot is at the hub, no meaningful direction to correct
      return new Translation2d();
    }

    hubTranslation = hubTranslation.div(norm); // Normalize to unit vector
    double correctionMagnitude = getRadialDistanceCorrectionVelocity(desiredDistance).in(MetersPerSecond);

    return hubTranslation.times(correctionMagnitude);
  }

  /**
   * Calculate what zone of the field a pose is in
   * 
   * @param pose The pose to check
   * @return The zone the pose is in
   */
  public FieldZones getPoseZone(Pose2d pose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    if (alliance.isPresent()) {
      Logger.recordOutput("Drivetrain/Alliance",
          isBlueAlliance ? "Blue" : "Red");
    } else {
      Logger.recordOutput("Drivetrain/Alliance",
          "Unknown");
    }

    if (pose.getX() > FieldConstants.kAllianceZoneXLength.in(Meters)
        && pose.getX() < FieldConstants.kFieldLengthX.minus(FieldConstants.kAllianceZoneXLength).in(Meters)) {
      return FieldZones.Neutral;
    }

    if ((pose.getX() < FieldConstants.kAcceptedLaunchingZone.in(Meters) && isBlueAlliance) ||
        (pose.getX() > FieldConstants.kFieldLengthX.minus(FieldConstants.kAcceptedLaunchingZone).in(Meters)
            && !isBlueAlliance)) {
      return FieldZones.Launch;
    }

    if ((pose.getX() < FieldConstants.kAllianceZoneXLength.in(Meters) && isBlueAlliance) ||
        (pose.getX() > FieldConstants.kFieldLengthX.minus(FieldConstants.kAllianceZoneXLength).in(Meters)
            && !isBlueAlliance)) {
      return FieldZones.Alliance;
    }

    return FieldZones.Enemy;
  }

  /**
   * Calculate what zone of the field the bot is in
   * 
   * @return The current zone the bot is in
   */
  public FieldZones getCurrentBotZone() {
    return getPoseZone(getPose());
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

        if (PoseHelpers.isInField(estimation.estimatedPose)
            && PoseHelpers.isOnGround(estimation.estimatedPose, PhotonConstants.kHeightTolerance)) {

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

    // Get current states
    SwerveModuleState[] states = getModuleStates();
    SwerveModulePosition[] positions = getModulePositions();

    // Update drive mode based on desired setting and gyro status
    isFieldRelativeReal = !gyroDisconnected && isFieldRelativeDesired;

    // Update pose estimator with odometry
    if (!gyroDisconnected) {
      robotPose = poseEstimator.update(gyro.getRotation2d(), positions);
      visionlessPose = visionlessPoseEstimator.update(gyro.getRotation2d(), positions);
    }

    // Iterate through each camera
    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      // Get all poses from camera
      List<EstimatedRobotPose> visionPoses = getEstimatedGlobalPose(photonPoseEstimators[i], cameras[i],
          PhotonConstants.kCameras[i].getRobotToCameraTransform(),
          robotPose);

      List<Integer> fiducialIds = new ArrayList<>();
      List<Pose3d> fiducialIdPoses = new ArrayList<>();
      List<Double> tagAreas = new ArrayList<>();
      List<Double> tagAmbiguities = new ArrayList<>();
      List<Double> stdDevsX = new ArrayList<>();
      List<Double> stdDevsY = new ArrayList<>();
      List<Double> stdDevsTheta = new ArrayList<>();

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

        // Calculate dynamic standard deviations based on measurement quality
        Vector<N3> stdDevs = VisionStdDevCalculator.calculateStdDevs(
            visionPose,
            PhotonConstants.kCameras[i].standardDeviation());

        // Add vision measurement with dynamic standard deviations
        poseEstimator.addVisionMeasurement(
            estimatedPose,
            visionPose.timestampSeconds,
            stdDevs);

        // Store std devs for logging
        stdDevsX.add(stdDevs.get(0));
        stdDevsY.add(stdDevs.get(1));
        stdDevsTheta.add(stdDevs.get(2));

        // Log vision poses and standard deviations
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

      // Log dynamic standard deviations for tuning
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/StdDevsX",
          stdDevsX.stream().mapToDouble(n -> n).toArray());
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/StdDevsY",
          stdDevsY.stream().mapToDouble(n -> n).toArray());
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/StdDevsTheta",
          stdDevsTheta.stream().mapToDouble(n -> n).toArray());
      Logger.recordOutput("Drivetrain/Vision/" + cameras[i].getName() + "/NumTargets",
          visionPoses.stream().mapToInt(p -> p.targetsUsed.size()).toArray());
    }

    // Calculate velocity and acceleration
    ChassisSpeeds speedsRaw = getChassisSpeedsRobotRelative();
    double dt = 0.02;

    double vxPrev = vxFilter.lastValue();
    double vyPrev = vyFilter.lastValue();

    double omegaPrev = omegaFilter.lastValue();
    double vx = vxFilter.calculate(speedsRaw.vxMetersPerSecond);
    double vy = vyFilter.calculate(speedsRaw.vyMetersPerSecond);
    double omega = omegaFilter.calculate(speedsRaw.omegaRadiansPerSecond);

    ChassisSpeeds speedsFiltered = new ChassisSpeeds(vx, vy, omega);

    double velocityNormRaw = Math
        .sqrt(Math.pow(speedsRaw.vxMetersPerSecond, 2) + Math.pow(speedsRaw.vyMetersPerSecond, 2));
    double velocityNormFiltered = Math
        .sqrt(Math.pow(speedsFiltered.vxMetersPerSecond, 2) + Math.pow(speedsFiltered.vyMetersPerSecond, 2));

    // use a chassis speed object to represent translational and angular
    // acceleration, this means that the units in the attribute names are
    // innacurate, (m/s -> m/s2, Rad/s -> rad/s2)
    // (do not log raw acceleration as it is too noisy)
    ChassisSpeeds acceleration = new ChassisSpeeds((vx - vxPrev) / dt, (vy - vyPrev) / dt, (omega - omegaPrev) / dt);
    double accelerationNorm = Math
        .sqrt(Math.pow(acceleration.vxMetersPerSecond, 2) + Math.pow(acceleration.vyMetersPerSecond, 2));

    // Log everything
    Logger.recordOutput("Drivetrain/GyroDisconnected", gyroDisconnected);
    Logger.recordOutput("Drivetrain/IsFieldRelativeReal", isFieldRelativeReal);
    Logger.recordOutput("Drivetrain/IsFieldRelativeDesired", isFieldRelativeDesired);
    Logger.recordOutput("Drivetrain/Pose", getPose());
    Logger.recordOutput("Drivetrain/VisionlessPose", visionlessPose);
    Logger.recordOutput("Drivetrain/Zone", getCurrentBotZone().name());
    Logger.recordOutput("Drivetrain/GyroRotation", gyro.getRotation2d().getDegrees());
    Logger.recordOutput("Drivetrain/Kinematics/VelocityRaw", speedsRaw);
    Logger.recordOutput("Drivetrain/Kinematics/VelocityNormRaw", velocityNormRaw);
    Logger.recordOutput("Drivetrain/Kinematics/VelocityFiltered", speedsFiltered);
    Logger.recordOutput("Drivetrain/Kinematics/VelocityNormFiltered", velocityNormFiltered);
    Logger.recordOutput("Drivetrain/Kinematics/Accelerations", acceleration);
    Logger.recordOutput("Drivetrain/Kinematics/AccelerationNorm", accelerationNorm);
    Logger.recordOutput("Drivetrain/Swerve/Module/State", states);
    Logger.recordOutput("Drivetrain/Swerve/Module/Position", positions);

    // Retrieve SmartDashboard settings
    if (simulatedSwerveDrive != null) {
      accurateSimOdometry = SmartDashboard.getBoolean("Accurate Sim Odometry", accurateSimOdometry);
    }

    // Apply new sim pose if requested, then reset the trigger
    // Check if button has been pressed
    boolean applyNewSimPose = !SmartDashboard.getBoolean("Apply Pose", true);
    if (applyNewSimPose) {
      SmartDashboard.putBoolean("Apply Pose", true);
      double x = SmartDashboard.getNumber("New Pose X", getPose().getX());
      double y = SmartDashboard.getNumber("New Pose Y", getPose().getY());
      double theta = SmartDashboard.getNumber("New Pose θ (Deg)", getPose().getRotation().getDegrees());
      Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
      resetPose(newPose);
      if (simulatedSwerveDrive != null) {
        simulatedSwerveDrive.setSimulationWorldPose(newPose);
      }
    }

    if (this.targetHeading != null) {
      Logger.recordOutput("Drivetrain/TargetHeading", this.getTargetHeading().getDegrees());
    }

    field.setRobotPose(getPose());
    SmartDashboard.putData("Field", field); // puts the field into SmartDashboard
    SmartDashboard.putBoolean("Gyro Connected", !gyroDisconnected);
    SmartDashboard.putBoolean("Is Field Relative Desired", isFieldRelativeDesired);
    SmartDashboard.putBoolean("Is Field Relative Real", isFieldRelativeReal);
  }

  @Override
  public void simulationPeriodic() {
    if (this.simulatedSwerveDrive != null) {
      Pose2d simulatedPose = simulatedSwerveDrive.getSimulatedDriveTrainPose();
      if (this.simulatedVision != null) {
        this.simulatedVision.update(simulatedPose);
      }
      Logger.recordOutput("FieldSimulation/PhysicalRobotPose", simulatedPose);
    }

  }
}