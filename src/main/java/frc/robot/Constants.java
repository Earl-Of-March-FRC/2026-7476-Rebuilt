// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kDriverControllerXAxis = 0;
    public static final int kDriverControllerYAxis = 1;
    public static final int kDriverControllerRotAxis = 4;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final AngularVelocity kDrivingMotorFreeSpeed = NeoMotorConstants.kFreeSpeed;
    public static Distance kWheelDiameter;
    public static Distance kWheelCircumference;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static double kDrivingMotorReduction;
    public static AngularVelocity kDriveWheelFreeSpeed;

    public static final double kDrivingPSim = 0.08;
    public static final double kDrivingISim = 0;

    public static final double kDrivingDSim = 0;
    public static double kDrivingFFSim = 1;

    public static final double kTurningMinOutputSim = -1;
    public static final double kTurningMaxOutputSIm = 1;

    public static final double kTurningPSim = 8;
    public static final double kTurningISim = 0;
    public static final double kTurningDSim = 0;
    public static final double kTurningFFSim = 0;
  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity kFreeSpeed = RotationsPerSecond.of(5676.0 / 60.0);
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static int profileId; // 1: Comp 2: SpongeBot 3: OffSeasonSwerve

    public static NavXComType gyroComType;

    public static RobotConfig kRobotConfig;

    public static LinearVelocity kMaxWheelSpeed; // Max possible speed for wheel
    public static LinearVelocity kMaxSpeed; // Default 4.8 - Max net robot translational speed
    public static AngularVelocity kMaxAngularSpeed; // radians per second
    public static LinearAcceleration kMaxAcceleration;

    // Ratios between robot limits in teleop vs auto
    public static final double kSpeedPathfindingRatio = 0.625;
    public static final double kAngularSpeedPathfindingRatio = 0.5;
    public static final double kAccelerationPathfindingRatio = 1.0 / 3.0;

    public static LinearVelocity kMaxSpeedPathfinding;
    public static AngularVelocity kMaxAngularSpeedPathfinding;
    public static LinearAcceleration kMaxAccelerationPathfinding;
    // Angular acceleration is only limited in pathfinding mode
    public static AngularAcceleration kMaxAngularAccelerationPathfinding = RadiansPerSecondPerSecond.of(Math.PI);

    public static final LinearVelocity kBangBangTranslationalVelocity = MetersPerSecond.of(2.5);
    public static final AngularVelocity kBangBangRotationalVelocity = RadiansPerSecond.of((2 * Math.PI) / 10);

    // Parameteres for restricted mode heading controller
    public static final double kPIDHeadingControllerP = 3.0;
    public static final double kPIDHeadingControllerI = 0.0;
    public static final double kPIDHeadingControllerD = 0.1;
    public static final double kPIDHeadingControllerTolerance = 2.0;
    public static final Angle kHeadingRestriction = Degrees.of(45);
    public static final Angle kRecalibrateThreshold = Degrees.of(30);

    // Parameteres for restricted mode radial controller
    public static final double kPIDRadialControllerP = 0.5;
    public static final double kPIDRadialControllerI = 0.01;
    public static final double kPIDRadialControllerD = 0.0;
    public static final Distance kPIDRadialControllerTolerance = Meters.of(0.05);

    public static PathConstraints kPathfindingConstraints;

    // To be used by PathPlanner
    public static final double kPTranslationController = 1.5;
    public static final double kITranslationController = 0.75;
    public static final double kDTranslationController = 0.25;

    public static final double kPThetaController = 1;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;

    // Chassis configuration
    public static Distance kTrackWidth;
    // Distance between centers of right and left wheels on robot
    public static Distance kWheelBase;;
    // Distance between front and back wheels on robot
    public static SwerveDriveKinematics kDriveKinematics;

    public static Distance kBumperLength; // Front to back
    public static Distance kBumperWidth; // Left to right

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

    public static int kFrontLeftDrivingCanId;
    public static int kFrontRightDrivingCanId;
    public static int kBackLeftDrivingCanId;
    public static int kBackRightDrivingCanId;

    public static int kFrontLeftTurningCanId;
    public static int kFrontRightTurningCanId;
    public static int kBackLeftTurningCanId;
    public static int kBackRightTurningCanId;

    public static final boolean kGyroReversed = false;
  }

  public static final class LauncherConstants {

    public static final Distance kLaunchRadius = Meters.of(2.0); // TEST VALUE Distance from
                                                                 // center of robot to
                                                                 // launch point
    public static final Time kBallAirTime = Seconds.of(0.5); // Estimated time for ball to reach target, used to
    // leadshots
  }

  public static final class SimulationConstants {
    public static final Supplier<GyroSimulation> kSimulatedGyro = COTS.ofGenericGyro(); // Simulated instance of our
    // gyro
    public static final DCMotor kSimulatedDrivingMotor = DCMotor.getNEO(1);
    public static final DCMotor kSimulatedTurningMotor = DCMotor.getNeo550(1);
    public static final double kSimulatedCoefficentOfFriction = COTS.WHEELS.COLSONS.cof;
    public static final int kGearRatioLevel = 2;

    public static final Pose2d kStartingPose = new Pose2d(7, 4, Rotation2d.fromDegrees(180));
  }

  public static final class PhotonConstants {
    // Camera offsets. Rotations are in radians. Translations are in meters.
    // +x is in front of the robot, +y is to the left of the robot, +z is up
    public static final double camera1Roll = 0.0;
    public static final double camera1Pitch = 0.1301;
    public static final double camera1Yaw = 0.0;
    public static final double camera1X = 0.307;
    public static final double camera1Y = 0.180;
    public static final double camera1Z = 0.750;
    public static final Vector<N3> kCamera1StandardDeviation = VecBuilder.fill(0.3, 0.3, 0.3);

    public static final double camera2Roll = 0;
    public static final double camera2Pitch = 0;
    public static final double camera2Yaw = Math.PI;
    public static final double camera2X = -0.3327;
    public static final double camera2Y = 0;
    public static final double camera2Z = 0.3708;
    public static final Vector<N3> kCamera2StandardDeviation = VecBuilder.fill(0.9, 0.9, 0.9);

    public static final double camera3Roll = 0.0;
    public static final double camera3Pitch = 0.0;
    public static final double camera3Yaw = 0.7069;
    public static final double camera3X = 0.238;
    public static final double camera3Y = -0.294;
    public static final double camera3Z = 0.625;
    public static final Vector<N3> kCamera3StandardDeviation = VecBuilder.fill(0.5, 0.5, 0.5);

    public static final int kAlgaePipeline = 1;
    public static final int kAprilTagPipeline = 0;

    public static final String kCamera1 = "Arducam_1";
    public static final String kCamera2 = "Arducam_2";
    public static final String kCamera3 = "Arducam_3";
    public static final String[] kCameras = { kCamera1, kCamera2, kCamera3 };
    public static final List<Vector<N3>> kCameraStandardDeviations = List.of(
        kCamera1StandardDeviation,
        kCamera2StandardDeviation,
        kCamera3StandardDeviation);

    public static final int numCameras = kCameras.length;

    public static final Transform3d kRobotToCam1 = new Transform3d(
        new Translation3d(PhotonConstants.camera1X, PhotonConstants.camera1Y, PhotonConstants.camera1Z),
        new Rotation3d(PhotonConstants.camera1Roll, PhotonConstants.camera1Pitch, PhotonConstants.camera1Yaw));
    public static final Transform3d kRobotToCam2 = new Transform3d(
        new Translation3d(PhotonConstants.camera2X, PhotonConstants.camera2Y, PhotonConstants.camera2Z),
        new Rotation3d(PhotonConstants.camera2Roll, PhotonConstants.camera2Pitch, PhotonConstants.camera2Yaw));
    public static final Transform3d kRobotToCam3 = new Transform3d(
        new Translation3d(PhotonConstants.camera3X, PhotonConstants.camera3Y, PhotonConstants.camera3Z),
        new Rotation3d(PhotonConstants.camera3Roll, PhotonConstants.camera3Pitch, PhotonConstants.camera3Yaw));
    public static final Transform3d[] kRobotToCams = { kRobotToCam1, kRobotToCam2, kRobotToCam3 };

    public static final double kHeightTolerance = 0.5; // meters above and below ground
    public static final double kAmbiguityDiscardThreshold = 0.8; // ignore targets above this value
    public static final double kAmbiguityThreshold = 0.3; // targets above this need to be checked
    public static final double kMinSingleTagArea = 0.2;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kfieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // Mesured in blue alliance field coordinate system in units of meters, as
    // described here:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // Mesurements source:
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf

    // From driverstation wall to center of hub
    public static final Distance kAllianceZoneLength = Inches.of(182.11);
    // Defines a zone starting from our driverstation where launching commands are
    // accepted (needs testing)
    public static final Distance kAcceptedLaunchingZone = kAllianceZoneLength.minus(Meters.of(1.0));
    // From drivestation wall to drivestation wall
    public static final Distance kFieldLengthX = Meter.of(kfieldLayout.getFieldLength());
    // Parallel distance from edge to edge
    public static final Distance kFieldWidthY = Meter.of(kfieldLayout.getFieldWidth());
    // Distance from field edge to middle of hub
    public static final Distance kHubY = kFieldWidthY.div(2.0);
    // Distance from blue driverstation wall to middle of hub
    public static final Distance kHubXBlue = kAllianceZoneLength;
    public static final Distance kHubXRed = kFieldLengthX.minus(kAllianceZoneLength);

    public static final Translation2d kBlueHubPose = new Translation2d(kHubXBlue.in(Meters), kHubY.in(Meters));
    public static final Translation2d kRedHubPose = new Translation2d(kHubXRed.in(Meters), kHubY.in(Meters));
    // public static final Translation2d kBlueHubPose = new Translation2d(4.625594,
    // kHubY.in(Meters));
    // public static final Translation2d kRedHubPose = new Translation2d(11.915394,
    // 4.034663);
  }
}