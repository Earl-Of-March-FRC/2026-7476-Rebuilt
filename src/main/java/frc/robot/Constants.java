// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.photonvision.estimation.TargetModel;

import com.pathplanner.lib.path.PathConstraints;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.swerve.SwerveConfig;
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
    public static final File kDeployDirectory = Filesystem.getDeployDirectory();

    public static final double kDriverSlowModeMultiplier = 0.3;
    public static final double kDriverTurnSensitivity = 0.4;
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

  public static final class LauncherConstants {
    public static final Distance kLaunchRadius = Meters.of(2.0);
    public static final Time kBallAirTime = Seconds.of(0.5);

    public static final AngularVelocity kVelocityLowRPM = RPM.of(0);
    // TODO COME BACK TO THIS TO CHANGE THE MAX!
    public static final AngularVelocity kVelocityHighRPM = RPM.of(0); // Fill in actual value

    public static final Current kSmartCurrentLimit = Amps.of(40);

    public static final double kPIDLauncherControllerP = 0;
    public static final double kPIDLauncherControllerI = 0;
    public static final double kPIDLauncherControllerD = 0;

    public static final double kOutputRangeMin = -1.0;
    public static final double kOutputRangeMax = 1.0;

    public static final ClosedLoopSlot kSlotHigh = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kSlotLow = ClosedLoopSlot.kSlot1;
  }

  public static final class DriveConstants {

    // Ratios between robot limits in teleop vs auto
    public static final double kSpeedPathfindingRatio = 0.8;
    public static final double kAngularSpeedPathfindingRatio = 0.8;
    public static final double kAccelerationPathfindingRatio = 0.8;

    // Angular acceleration is only limited in pathfinding mode
    public static AngularAcceleration kMaxAngularAccelerationPathfinding = RadiansPerSecondPerSecond.of(Math.PI);

    public static final LinearVelocity kBangBangTranslationalVelocity = MetersPerSecond.of(2.5);
    public static final AngularVelocity kBangBangRotationalVelocity = RadiansPerSecond.of((2 * Math.PI) / 10);

    // Parameteres for restricted mode heading controller
    public static final double kPIDHeadingControllerP = 5;
    public static final double kPIDHeadingControllerI = 0;
    public static final double kPIDHeadingControllerD = 0.15;
    public static final double kPIDHeadingControllerTolerance = 2.0;
    public static final Angle kBumpHeadingRestriction = Degrees.of(45);
    public static final Angle kTrenchHeadingRestriction = Degrees.of(0);
    public static final Angle kRecalibrateThreshold = Degrees.of(30);

    // Parameteres for restricted mode radial controller
    public static final double kPIDRadialControllerP = 1.5;
    public static final double kPIDRadialControllerI = 0.01;
    public static final double kPIDRadialControllerD = 0.0;
    public static final Distance kPIDRadialControllerTolerance = Meters.of(0.05);

    // Parameters for X and Y controllers (gains should be identical if possible,
    // but controllers shold be decoupled)
    public static final double kPIDXControllerP = 0.5;
    public static final double kPIDXControllerI = 0.01;
    public static final double kPIDXControllerD = 0.0;
    public static final Distance kPIDXControllerTolerance = Meters.of(0.05);

    public static final double kPIDYControllerP = kPIDXControllerP;
    public static final double kPIDYControllerI = kPIDXControllerI;
    public static final double kPIDYControllerD = kPIDXControllerD;
    public static final Distance kPIDYControllerTolerance = kPIDXControllerTolerance;

    // Define seperate constraints for the bump and trench, Independant of the
    // swerve profile
    public static final LinearVelocity kBumpLinearVelocity = MetersPerSecond.of(2.5);
    public static final LinearAcceleration kBumpLinearAcceleration = MetersPerSecondPerSecond.of(1.5);
    public static final AngularVelocity kBumpAngularVelocity = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration kBumpAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

    public static final PathConstraints kBumpConstraints = new PathConstraints(
        kBumpLinearVelocity,
        kBumpLinearAcceleration,
        kBumpAngularVelocity,
        kBumpAngularAcceleration);

    public static final LinearVelocity kTrenchLinearVelocity = MetersPerSecond.of(4.8);
    public static final LinearAcceleration kTrenchLinearAcceleration = MetersPerSecondPerSecond.of(1.5);
    public static final AngularVelocity kTrenchAngularVelocity = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration kTrenchAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

    public static final PathConstraints kTrenchConstraints = new PathConstraints(
        kTrenchLinearVelocity,
        kTrenchLinearAcceleration,
        kTrenchAngularVelocity,
        kTrenchAngularAcceleration);

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

  public static final class AutoConstants {
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(2.0);
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI);

    public static final double kMaxSpeedMetersPerSecond = kMaxSpeed.in(MetersPerSecond);
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double kPTranslationController = 1.5;
    public static final double kITranslationController = 0.75;
    public static final double kDTranslationController = 0.25;

    public static final double kPThetaController = 1.0;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.0;

    // AlignTowerCmd tolerances
    public static final Distance kAlignTranslationTolerance = Meters.of(0.05);
    public static final Angle kAlignRotationTolerance = Radians.of(Math.toRadians(2.0));

    // public static final class EncoderAutoDriveConstants {
    // public static final double kLeaveZoneMeters = 0.5; // Distance to travel
    // public static final double kLeaveZoneVelocity = 0.5; // Velocity (Meters/S)
    // to leave zone at
    // }

    // // Constraint for the motion profiled robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
    // = new TrapezoidProfile.Constraints(
    // kMaxAngularSpeedRadiansPerSecond,
    // kMaxAngularAccelerationRadiansPerSecondSquared);

    // public static final Pose2d kLaunchPoseBlue = new Pose2d(new
    // Translation2d(7.475, 5.37),
    // Rotation2d.fromDegrees(180));
    // public static final Pose2d kLaunchPoseRed = new Pose2d(new
    // Translation2d(10.075, 2.68), new Rotation2d(0));

  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanId = 10;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kMotorReduction = 1.0 / 10.0;

    // Conversion factors (RPM → rad/s)
    public static final double kPositionConversionFactor = 2 * Math.PI;
    public static final double kVelocityConversionFactor = 2 * Math.PI / 60.0;

    public static final AngularVelocity kMaxVelocity = RPM.of(60);

    public static final double kIntakeSpeed = 0.5;
    public static final double kPlowSpeed = 0.7;
  }

  public static final class IndexerConstants {
    public static final int kMotorCanId = 11;
    public static final MotorType kMotorType = MotorType.kBrushless;

    /**
     * Multiplier at which decides whether + or - inputs move the algae towards the
     * launcher.
     */
    public static final double kDirectionConstant = -1.0;

    public static final double kMotorReduction = 1.0;
    public static final double kWheelDiameterMeters = 0.17;

    // // Ports for sensors. TBD
    // public static final int kIntakeSensorChannel = 0;
    // public static final int kLauncherSensorChannel = 1;
  }

  public static final class ClimberConstants {
    public static final int kMotorId = 5;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final Distance kStowPosition = Inches.of(0);
    public static final Distance kClimbPosition = Inches.of(32);

    public static final double kMotorRaiseSpeed = 0.67;
    public static final double kMotorHookSpeed = 0.67;

    public static final double kTicksToInchesConversion = 0.67;

    public static final Distance kMinLength = Inches.of(-1);
    public static final Distance kMaxLength = Inches.of(33);

    public static final Current kSmartCurrentLimit = Amps.of(40);

    public static final double kOutputRangeMin = -1.0;
    public static final double kOutputRangeMax = 1.0;

    // TalonFX-specific
    public static final Current kStatorCurrentLimit = Amps.of(40);
    public static final double kSensorToMechanismRatio = 1.0; // Update with real gear ratio

    public static final double kPIDClimberControllerP = 0.1;
    public static final double kPIDClimberControllerI = 0.0;
    public static final double kPIDClimberControllerD = 0.0;
  }

  public static final class GameModelConstants {
    /** Match time thresholds for phase transitions */
    public static final int AUTO_END = 140;
    public static final int TRANSITION_END = 130;
    public static final int SHIFT_1_END = 105;
    public static final int SHIFT_2_END = 80;
    public static final int SHIFT_3_END = 55;
    public static final int SHIFT_4_END = 30;

    /** Grace window duration after hub deactivates (seconds) */
    public static final double GRACE_DURATION = 3.0;

  }

  public static final class SimulationConstants {
    public static final Supplier<GyroSimulation> kSimulatedGyro = COTS.ofGenericGyro(); // Simulated instance of our
    // gyro
    public static final DCMotor kSimulatedDrivingMotor = DCMotor.getNEO(1);
    public static final DCMotor kSimulatedTurningMotor = DCMotor.getNeo550(1);
    public static final double kSimulatedCoefficentOfFriction = COTS.WHEELS.COLSONS.cof;
    public static final int kGearRatioLevel = 2;

    public static final Pose2d kStartingPose = new Pose2d(7, 4, Rotation2d.fromRotations(Math.PI));

    // Whether the bump should have defined collision
    // Maple sim provides 2d simulation, and cannot simulate the bump accurately,
    // it can be either a wall or non-existant
    public static final boolean kSimBumpCollision = false;

    // Default vision properties
    public static final File kSimVisionConfigurationFile = new File(Filesystem.getDeployDirectory().getPath(),
        "simulated_camera_settings\\arducam_OV9281_calibration_1280x720.json");
    public static final double kSimVisionFPS = 20;
    public static final Time kSimVisionLatency = Milliseconds.of(40);
    public static final Time kSimVisionLatencyDeviation = Milliseconds.of(5);
    public static final Angle kSimVisionFOV = Degrees.of(70);
    public static final int[] kSimVisionResolution = { 1280, 720 };
    public static final Matrix<N3, N3> kSimVisionIntrinsics = MatBuilder.fill(N3.instance, N3.instance,
        940.7360710926395,
        0.0,
        615.5884770322365,
        0.0,
        939.9932393907364,
        328.53938300868,
        0.0,
        0.0,
        1.0);
    public static final Matrix<N8, N1> kSimVisionDistCoeffs = MatBuilder.fill(N8.instance, N1.instance,
        0.054834081023049625,
        -0.15994111706817074,
        -0.0017587106009926158,
        -0.0014671022483263552,
        0.049742166267499596, 0, 0, 0);
  }

  public static final class PhotonConstants {
    // Camera profiles - each camera's configuration in one place

    public static final int kAprilTagPipeline = 0;

    public static final Distance kHeightTolerance = Meters.of(0.5); // meters above and below ground
    public static final double kAmbiguityDiscardThreshold = 0.8; // ignore targets above this value
    public static final double kAmbiguityThreshold = 0.3; // targets above this need to be checked
    public static final double kMinSingleTagArea = 0.2;
    public static final TargetModel kTargetModel = TargetModel.kAprilTag36h11;

    public static final Distance kVisionBaseXYStdDev = Meters.of(0.3);
    public static final Angle kVisionBaseThetaStdDev = Radians.of(0.3);

    public static final double kVisionMaxStdDev = 5.0;

    public static final Distance kVisionCloseDistance = Meters.of(2.0); // "Close" threshold
    public static final Distance kVisionFarDistance = Meters.of(6.0); // "Far" threshold

    public static final double kVisionDistanceScaleFactor = 0.5;

    public static final double kVisionHighAmbiguityThreshold = 0.2;

    public static final double kVisionHighAmbiguityMultiplier = 1.5;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kfieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // Mesured in blue alliance field coordinate system in units of meters, as
    // described here:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // Mesurements source:
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf

    public static final Distance kAllianceWallToHubCenter = Inches.of(182.11);
    // From driverstation to coloured line
    public static final Distance kAllianceZoneXLength = Inches.of(156.61);
    // Defines a zone starting from our driverstation where launching commands are
    // accepted (needs testing)
    public static final Distance kAcceptedLaunchingZone = kAllianceZoneXLength.minus(Meters.of(1.0));
    // From drivestation wall to drivestation wall
    public static final Distance kFieldLengthX = Meter.of(kfieldLayout.getFieldLength());
    // Parallel distance from edge to edge
    public static final Distance kFieldWidthY = Meter.of(kfieldLayout.getFieldWidth());

    public static final Distance kEdgeToTrenchCenter = Inches.of(26.22);
    public static final Distance kEdgeToBumpCenter = Inches.of(104.34);
    // Cross slightly offset from the center of the bump
    public static final Distance kEdgeToBumpCrossLine = kEdgeToBumpCenter.minus(Inches.of(4));
    public static final Distance kBumpWidth = Inches.of(47.00);

    // Distance from field edge to middle of hub
    public static final Distance kHubY = kFieldWidthY.div(2.0);
    // Distance from blue driverstation wall to middle of hub
    public static final Distance kHubXBlue = kAllianceZoneXLength;
    public static final Distance kHubXRed = kFieldLengthX.minus(kAllianceZoneXLength);

    public static final Translation2d kBlueHubPose = new Translation2d(kHubXBlue.in(Meters), kHubY.in(Meters));
    public static final Translation2d kRedHubPose = new Translation2d(kHubXRed.in(Meters), kHubY.in(Meters));
    // public static final Translation2d kBlueHubPose = new Translation2d(4.625594,
    // kHubY.in(Meters));
    // public static final Translation2d kRedHubPose = new Translation2d(11.915394,
    // 4.034663);

    public static final Distance kCrossAllianceWaypointX = kAllianceZoneXLength
        .minus(SwerveConfig.kBumperWidth.div(2.0));
    public static final Distance kCrossNeutralWaypointX = kAllianceZoneXLength
        .plus(SwerveConfig.kBumperWidth.div(2.0)).plus(kBumpWidth);

    // Desired positions on either side of the 4 trenches, used for path finding
    // kTrenchPathWaypoints[i] and kTrenchPathWaypoints[i + 4] are always opposite
    public static final Translation2d[] kTrenchPathWaypoints = new Translation2d[] {
        new Translation2d(kCrossAllianceWaypointX.in(Meters),
            kFieldWidthY.minus(kEdgeToTrenchCenter).in(Meters)), // Blue Depot
        new Translation2d(kCrossAllianceWaypointX.in(Meters),
            kEdgeToTrenchCenter.in(Meters)), // Blue Outpost
        new Translation2d(kFieldLengthX.minus(kCrossAllianceWaypointX).in(Meters),
            kEdgeToTrenchCenter.in(Meters)), // Red Depot
        new Translation2d(kFieldLengthX.minus(kCrossAllianceWaypointX).in(Meters),
            kFieldWidthY.minus(kEdgeToTrenchCenter).in(Meters)), // Red Outpost
        new Translation2d(kCrossNeutralWaypointX.in(Meters),
            kFieldWidthY.minus(kEdgeToTrenchCenter).in(Meters)), // Neutral Blue Depot
        new Translation2d(kCrossNeutralWaypointX.in(Meters),
            kEdgeToTrenchCenter.in(Meters)), // Neutral Blue Outpost
        new Translation2d(kFieldLengthX.minus(kCrossNeutralWaypointX).in(Meters),
            kEdgeToTrenchCenter.in(Meters)), // Neutral Red Depot
        new Translation2d(kFieldLengthX.minus(kCrossNeutralWaypointX).in(Meters),
            kFieldWidthY.minus(kEdgeToTrenchCenter).in(Meters)), // Neutral Red Outpost
    };

    // Desired positions on either side of the 4 bumps, used for path finding
    // kBumpPathWaypoints[i] and kBumpPathWaypoints[i + 4] are always opposite
    public static final Translation2d[] kBumpPathWaypoints = new Translation2d[] {
        new Translation2d(kCrossAllianceWaypointX.in(Meters),
            kFieldWidthY.minus(kEdgeToBumpCrossLine).in(Meters)), // Blue Depot
        new Translation2d(kCrossAllianceWaypointX.in(Meters),
            kEdgeToBumpCrossLine.in(Meters)), // Blue Outpost
        new Translation2d(kFieldLengthX.minus(kCrossAllianceWaypointX).in(Meters),
            kEdgeToBumpCrossLine.in(Meters)), // Red Depot
        new Translation2d(kFieldLengthX.minus(kCrossAllianceWaypointX).in(Meters),
            kFieldWidthY.minus(kEdgeToBumpCrossLine).in(Meters)), // Red Outpost
        new Translation2d(kCrossNeutralWaypointX.in(Meters),
            kFieldWidthY.minus(kEdgeToBumpCrossLine).in(Meters)), // Neutral Blue Depot
        new Translation2d(kCrossNeutralWaypointX.in(Meters),
            kEdgeToBumpCrossLine.in(Meters)), // Neutral Blue Outpost
        new Translation2d(kFieldLengthX.minus(kCrossNeutralWaypointX).in(Meters),
            kEdgeToBumpCrossLine.in(Meters)), // Neutral Red Depot
        new Translation2d(kFieldLengthX.minus(kCrossNeutralWaypointX).in(Meters),
            kFieldWidthY.minus(kEdgeToBumpCrossLine).in(Meters)), // Neutral Red Outpost
    };
  }
}