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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroADXRS450;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.subsystems.Drivetrain.GyroNavX;
import frc.robot.util.swerve.SwerveDriveProfile.SwerveDriveProfileID;

public final class SwerveConfig {

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static SwerveDriveProfileID profileId; // 1: Comp 2: SpongeBot 3: OffSeasonSwerve

  public static Gyro gyro;

  public static RobotConfig kRobotConfig;

  public static LinearVelocity kMaxWheelSpeed; // Max possible speed for wheel
  public static LinearVelocity kMaxSpeed; // Default 4.8 - Max net robot translational speed
  public static AngularVelocity kMaxAngularSpeed; // radians per second
  public static LinearAcceleration kMaxAcceleration;

  public static LinearVelocity kMaxSpeedPathfinding;
  public static AngularVelocity kMaxAngularSpeedPathfinding;
  public static LinearAcceleration kMaxAccelerationPathfinding;

  public static PathConstraints kPathfindingConstraints;

  // Chassis configuration
  public static Distance kTrackWidth;
  // Distance between centers of right and left wheels on robot
  public static Distance kWheelBase;;
  // Distance between front and back wheels on robot
  public static SwerveDriveKinematics kDriveKinematics;

  public static Distance kBumperLength; // Front to back
  public static Distance kBumperWidth; // Left to right

  public static int kFrontLeftDrivingCanId;
  public static int kFrontRightDrivingCanId;
  public static int kBackLeftDrivingCanId;
  public static int kBackRightDrivingCanId;

  public static int kFrontLeftTurningCanId;
  public static int kFrontRightTurningCanId;
  public static int kBackLeftTurningCanId;
  public static int kBackRightTurningCanId;

  /**
   * Applies the given swerve drive profile to the robot's drive and module
   * constants.
   * This updates all relevant static fields in DriveConstants and
   * ModuleConstants.
   * 
   * @param profile The swerve drive profile to apply
   */
  public static void applyProfile(SwerveDriveProfile profile) {
    // Apply profile ID
    SwerveConfig.profileId = profile.profileId();

    // Apply new Gyro
    SwerveConfig.gyro = switch (profile.gyro()) {
      case ADXRS450 -> new GyroADXRS450();
      case NavX_MXP_SPI -> new GyroNavX(NavXComType.kMXP_SPI);
      case NavX_USB1 -> new GyroNavX(NavXComType.kUSB1);
      default -> new GyroNavX(NavXComType.kMXP_SPI);
    };

    // Apply speed and dimension constants
    SwerveConfig.kMaxSpeed = profile.maxSpeedMps();
    SwerveConfig.kTrackWidth = profile.trackWidthMeters();
    SwerveConfig.kWheelBase = profile.wheelBaseMeters();
    SwerveConfig.kBumperLength = profile.bumperLength();
    SwerveConfig.kBumperWidth = profile.bumperWidth();

    // Apply wheel constants
    ModuleConstants.kWheelDiameter = profile.wheelDiameterMeters();
    ModuleConstants.kWheelCircumference = profile.wheelDiameterMeters().times(Math.PI);
    ModuleConstants.kDrivingMotorReduction = profile.driveReduction();

    // Calculate drive wheel free speed
    ModuleConstants.kDriveWheelFreeSpeed = RotationsPerSecond.of(
        ModuleConstants.kDrivingMotorFreeSpeed
            .times(profile.wheelDiameterMeters().times(Math.PI))
            .div(profile.driveReduction())
            .in(MultUnit.combine(RotationsPerSecond, Meters)));

    // Apply robot limits for teleop and auto
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

    // Configure kinematics
    double halfWheelBase = profile.wheelBaseMeters().div(2).in(Meters);
    double halfTrackWidth = profile.trackWidthMeters().div(2).in(Meters);

    Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(halfWheelBase, halfTrackWidth), // Front Left
        new Translation2d(halfWheelBase, -halfTrackWidth), // Front Right
        new Translation2d(-halfWheelBase, halfTrackWidth), // Back Left
        new Translation2d(-halfWheelBase, -halfTrackWidth) // Back Right
    };

    SwerveConfig.kDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    // Apply CAN IDs
    SwerveConfig.kFrontLeftDrivingCanId = profile.driveCanIds()[0];
    SwerveConfig.kFrontRightDrivingCanId = profile.driveCanIds()[1];
    SwerveConfig.kBackLeftDrivingCanId = profile.driveCanIds()[2];
    SwerveConfig.kBackRightDrivingCanId = profile.driveCanIds()[3];

    SwerveConfig.kFrontLeftTurningCanId = profile.turnCanIds()[0];
    SwerveConfig.kFrontRightTurningCanId = profile.turnCanIds()[1];
    SwerveConfig.kBackLeftTurningCanId = profile.turnCanIds()[2];
    SwerveConfig.kBackRightTurningCanId = profile.turnCanIds()[3];

    // Calculate driving feed-forward for simulation
    ModuleConstants.kDrivingFFSim = 1.0 / ModuleConstants.kDriveWheelFreeSpeed.in(RotationsPerSecond);

    // Create path planner robot config
    DCMotor gearbox = DCMotor.getNEO(1).withReduction(profile.driveReduction());
    ModuleConfig moduleConfig = new ModuleConfig(
        profile.wheelDiameterMeters(),
        profile.maxSpeedMps(),
        profile.wheelCof(),
        gearbox,
        profile.driveCurrentLimitAmps(), // Safe limit for NEOs
        1); // 1 drive motor per module (universal for swerve)

    SwerveConfig.kRobotConfig = new RobotConfig(profile.robotMass(), profile.robotMOI(), moduleConfig,
        moduleTranslations);
  }

  // Private constructor to prevent instantiation
  private SwerveConfig() {
    throw new UnsupportedOperationException(
        this.getClass().getName() + " is a utility class and cannot be instantiated");
  }

}