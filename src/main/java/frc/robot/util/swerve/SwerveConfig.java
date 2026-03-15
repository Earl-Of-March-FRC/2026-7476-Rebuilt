package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroADXRS450;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.util.swerve.SwerveDriveProfile.SwerveDriveProfileID;
import frc.robot.util.vision.CameraProfile;

public final class SwerveConfig {

  public static SwerveDriveProfileID profileId;

  public static Gyro gyro;

  public static RobotConfig kRobotConfig;

  public static LinearVelocity kMaxWheelSpeed;
  public static LinearVelocity kMaxSpeed;
  public static AngularVelocity kMaxAngularSpeed;
  public static LinearAcceleration kMaxAcceleration;

  public static LinearVelocity kMaxSpeedPathfinding;
  public static AngularVelocity kMaxAngularSpeedPathfinding;
  public static LinearAcceleration kMaxAccelerationPathfinding;

  public static PathConstraints kPathfindingConstraints;

  public static Distance kTrackWidth;
  public static Distance kWheelBase;
  public static SwerveDriveKinematics kDriveKinematics;

  public static Distance kBumperLength;
  public static Distance kBumperWidth;

  public static int kFrontLeftDrivingCanId;
  public static int kFrontRightDrivingCanId;
  public static int kBackLeftDrivingCanId;
  public static int kBackRightDrivingCanId;

  public static int kFrontLeftTurningCanId;
  public static int kFrontRightTurningCanId;
  public static int kBackLeftTurningCanId;
  public static int kBackRightTurningCanId;

  // Swerve module drive motor PID (populated from active profile)
  public static double kDrivingP;
  public static double kDrivingI;
  public static double kDrivingD;
  public static int kDriveSmartCurrentLimit;
  public static int kDriveFreeCurrentLimit;

  // Swerve module turn motor PID (populated from active profile)
  public static double kTurningP;
  public static double kTurningI;
  public static double kTurningD;
  public static int kTurnSmartCurrentLimit;

  public static CameraProfile[] kCameraProfiles;
  public static int kNumCameras;

  public static boolean kUseDynamicStandardDeviations;

  /**
   * Applies the given swerve drive profile to the robot's drive and module
   * constants. This updates all relevant static fields including camera profiles
   * and swerve module PID values.
   *
   * @param profile The swerve drive profile to apply
   */
  public static void applyProfile(SwerveDriveProfile profile) {
    SwerveConfig.profileId = profile.profileId();

    SwerveConfig.gyro = switch (profile.gyro()) {
      case ADXRS450 -> new GyroADXRS450();
      case NavX_MXP_SPI -> new GyroNavX(NavXComType.kMXP_SPI);
      case NavX_USB1 -> new GyroNavX(NavXComType.kUSB1);
      default -> new GyroNavX(NavXComType.kMXP_SPI);
    };

    SwerveConfig.kMaxSpeed = profile.maxSpeedMps();
    SwerveConfig.kTrackWidth = profile.trackWidthMeters();
    SwerveConfig.kWheelBase = profile.wheelBaseMeters();
    SwerveConfig.kBumperLength = profile.bumperLength();
    SwerveConfig.kBumperWidth = profile.bumperWidth();

    ModuleConstants.kWheelDiameter = profile.wheelDiameterMeters();
    ModuleConstants.kWheelCircumference = profile.wheelDiameterMeters().times(Math.PI);
    ModuleConstants.kDrivingMotorReduction = profile.driveReduction();

    ModuleConstants.kDriveWheelFreeSpeed = RotationsPerSecond.of(
        ModuleConstants.kDrivingMotorFreeSpeed
            .times(profile.wheelDiameterMeters().times(Math.PI))
            .div(profile.driveReduction())
            .in(MultUnit.combine(RotationsPerSecond, Meters)));

    SwerveConfig.kMaxWheelSpeed = profile.maxSpeedMps();
    SwerveConfig.kMaxAngularSpeed = profile.maxAngularSpeedRps();
    SwerveConfig.kMaxAcceleration = profile.maxLinearAccelerationRps2();

    SwerveConfig.kMaxSpeedPathfinding = SwerveConfig.kMaxSpeed.times(DriveConstants.kSpeedPathfindingRatio);
    SwerveConfig.kMaxAngularSpeedPathfinding = SwerveConfig.kMaxAngularSpeed
        .times(DriveConstants.kAngularSpeedPathfindingRatio);
    SwerveConfig.kMaxAccelerationPathfinding = SwerveConfig.kMaxAcceleration
        .times(DriveConstants.kAccelerationPathfindingRatio);

    SwerveConfig.kPathfindingConstraints = new PathConstraints(
        SwerveConfig.kMaxSpeedPathfinding.in(MetersPerSecond),
        SwerveConfig.kMaxAccelerationPathfinding.in(MetersPerSecondPerSecond),
        SwerveConfig.kMaxAngularSpeedPathfinding.in(RadiansPerSecond),
        DriveConstants.kMaxAngularAccelerationPathfinding.in(RadiansPerSecondPerSecond));

    double halfWheelBase = profile.wheelBaseMeters().div(2).in(Meters);
    double halfTrackWidth = profile.trackWidthMeters().div(2).in(Meters);

    Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(halfWheelBase, halfTrackWidth), // Front Left
        new Translation2d(halfWheelBase, -halfTrackWidth), // Front Right
        new Translation2d(-halfWheelBase, halfTrackWidth), // Back Left
        new Translation2d(-halfWheelBase, -halfTrackWidth) // Back Right
    };

    SwerveConfig.kDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    SwerveConfig.kFrontLeftDrivingCanId = profile.driveCanIds()[0];
    SwerveConfig.kFrontRightDrivingCanId = profile.driveCanIds()[1];
    SwerveConfig.kBackLeftDrivingCanId = profile.driveCanIds()[2];
    SwerveConfig.kBackRightDrivingCanId = profile.driveCanIds()[3];

    SwerveConfig.kFrontLeftTurningCanId = profile.turnCanIds()[0];
    SwerveConfig.kFrontRightTurningCanId = profile.turnCanIds()[1];
    SwerveConfig.kBackLeftTurningCanId = profile.turnCanIds()[2];
    SwerveConfig.kBackRightTurningCanId = profile.turnCanIds()[3];

    ModuleConstants.kDrivingFFSim = 1.0 / ModuleConstants.kDriveWheelFreeSpeed.in(RotationsPerSecond);

    // Apply swerve module PID values from profile
    SwerveConfig.kDrivingP = profile.drivingP();
    SwerveConfig.kDrivingI = profile.drivingI();
    SwerveConfig.kDrivingD = profile.drivingD();
    SwerveConfig.kDriveSmartCurrentLimit = profile.driveSmartCurrentLimit();
    SwerveConfig.kDriveFreeCurrentLimit = profile.driveFreeCurrentLimit();

    SwerveConfig.kTurningP = profile.turningP();
    SwerveConfig.kTurningI = profile.turningI();
    SwerveConfig.kTurningD = profile.turningD();
    SwerveConfig.kTurnSmartCurrentLimit = profile.turnSmartCurrentLimit();

    SwerveConfig.kCameraProfiles = profile.cameraProfiles();
    SwerveConfig.kNumCameras = profile.getNumCameras();

    SwerveConfig.kUseDynamicStandardDeviations = profile.visionUsesDynamicStandardDeviations();

    DCMotor gearbox = DCMotor.getNEO(1).withReduction(profile.driveReduction());
    ModuleConfig moduleConfig = new ModuleConfig(
        profile.wheelDiameterMeters(),
        profile.maxSpeedMps(),
        profile.wheelCof(),
        gearbox,
        profile.driveCurrentLimitAmps(),
        1);

    SwerveConfig.kRobotConfig = new RobotConfig(profile.robotMass(), profile.robotMOI(), moduleConfig,
        moduleTranslations);
  }

  private SwerveConfig() {
    throw new UnsupportedOperationException(
        this.getClass().getName() + " is a utility class and cannot be instantiated");
  }
}