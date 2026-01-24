package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.MultUnit;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Utility class for applying swerve drive profiles to the robot constants.
 * This centralizes the logic for updating all configuration values when
 * switching profiles.
 */
public final class SwerveProfileApplicator {

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
    DriveConstants.profileId = profile.profileId();

    // Apply speed and dimension constants
    DriveConstants.kMaxSpeed = profile.maxSpeedMps();
    DriveConstants.kTrackWidth = profile.trackWidthMeters();
    DriveConstants.kWheelBase = profile.wheelBaseMeters();
    DriveConstants.kBumperLength = profile.bumperLength();
    DriveConstants.kBumperWidth = profile.bumperWidth();

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
    DriveConstants.kMaxWheelSpeed = profile.maxSpeedMps();
    DriveConstants.kMaxAngularSpeed = profile.maxAngularSpeedRps();
    DriveConstants.kMaxAcceleration = profile.maxLinearAccelerationRps2();

    DriveConstants.kMaxSpeedPathfinding = DriveConstants.kMaxSpeed.times(DriveConstants.kSpeedPathfindingRatio);
    DriveConstants.kMaxAngularSpeedPathfinding = DriveConstants.kMaxAngularSpeed
        .times(DriveConstants.kAngularSpeedPathfindingRatio);
    DriveConstants.kMaxAccelerationPathfinding = DriveConstants.kMaxAcceleration
        .times(DriveConstants.kAccelerationPathfindingRatio);

    // Configure kinematics
    double halfWheelBase = profile.wheelBaseMeters().div(2).in(Meters);
    double halfTrackWidth = profile.trackWidthMeters().div(2).in(Meters);

    DriveConstants.kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(halfWheelBase, halfTrackWidth), // Front Left
        new Translation2d(halfWheelBase, -halfTrackWidth), // Front Right
        new Translation2d(-halfWheelBase, halfTrackWidth), // Back Left
        new Translation2d(-halfWheelBase, -halfTrackWidth)); // Back Right

    // Apply CAN IDs
    DriveConstants.kFrontLeftDrivingCanId = profile.driveCanIds()[0];
    DriveConstants.kFrontRightDrivingCanId = profile.driveCanIds()[1];
    DriveConstants.kBackLeftDrivingCanId = profile.driveCanIds()[2];
    DriveConstants.kBackRightDrivingCanId = profile.driveCanIds()[3];

    DriveConstants.kFrontLeftTurningCanId = profile.turnCanIds()[0];
    DriveConstants.kFrontRightTurningCanId = profile.turnCanIds()[1];
    DriveConstants.kBackLeftTurningCanId = profile.turnCanIds()[2];
    DriveConstants.kBackRightTurningCanId = profile.turnCanIds()[3];

    // Calculate driving feed-forward for simulation
    ModuleConstants.kDrivingFFSim = 1.0 / ModuleConstants.kDriveWheelFreeSpeed.in(RotationsPerSecond);
  }

  // Private constructor to prevent instantiation
  private SwerveProfileApplicator() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
}