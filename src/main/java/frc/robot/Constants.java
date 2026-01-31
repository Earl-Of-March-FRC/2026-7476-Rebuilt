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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.vision.CameraProfile;

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

    // Ratios between robot limits in teleop vs auto
    public static final double kSpeedPathfindingRatio = 0.625;
    public static final double kAngularSpeedPathfindingRatio = 0.5;
    public static final double kAccelerationPathfindingRatio = 1.0 / 3.0;

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

    // To be used by PathPlanner
    public static final double kPTranslationController = 1.5;
    public static final double kITranslationController = 0.75;
    public static final double kDTranslationController = 0.25;

    public static final double kPThetaController = 1;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

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

    // Camera profiles - each camera's configuration in one place
    public static final CameraProfile kCamera1Profile = new CameraProfile(
        "Arducam_1",
        0.0, // roll
        0.1301, // pitch
        0.0, // yaw
        0.307, // x
        0.180, // y
        0.750, // z
        VecBuilder.fill(0.3, 0.3, 0.3) // standard deviation
    );

    public static final CameraProfile kCamera2Profile = new CameraProfile(
        "Arducam_2",
        0.0, // roll
        0.0, // pitch
        Math.PI, // yaw
        -0.3327, // x
        0.0, // y
        0.3708, // z
        VecBuilder.fill(0.9, 0.9, 0.9) // standard deviation
    );

    public static final CameraProfile kCamera3Profile = new CameraProfile(
        "Arducam_3",
        0.0, // roll
        0.0, // pitch
        0.7069, // yaw
        0.238, // x
        -0.294, // y
        0.625, // z
        VecBuilder.fill(0.5, 0.5, 0.5) // standard deviation
    );

    public static final int kAprilTagPipeline = 0;

    public static final String kCamera1 = "Arducam_1";
    public static final String kCamera2 = "Arducam_2";
    public static final String kCamera3 = "Arducam_3";

    public static final String[] kCameras = { kCamera1, kCamera2, kCamera3 };

    public static final List<Vector<N3>> kCameraStandardDeviations = List.of(
        kCamera1Profile.standardDeviation(),
        kCamera2Profile.standardDeviation(),
        kCamera3Profile.standardDeviation());

    public static final int numCameras = kCameras.length;

    public static final Transform3d[] kRobotToCams = {
        kCamera1Profile.getRobotToCameraTransform(), // Camera 1 Transform3d
        kCamera2Profile.getRobotToCameraTransform(), // Camera 2 Transform3d
        kCamera3Profile.getRobotToCameraTransform() // Camera 3 Transform3d
    };

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