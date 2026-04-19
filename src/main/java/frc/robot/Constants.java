// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.indexer.PulsingTreadmillCmd;
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

  public static final class PhysicsConstants {
    /** Standard gravitational acceleration at Earth's surface (m/s^2). */
    public static final LinearAcceleration kGravity = MetersPerSecondPerSecond.of(9.80665);
    /**
     * Raw gravitational acceleration as a plain double (m/s^2).
     * Use this in kinematic equations where units would be cumbersome.
     */
    public static final double kGravityMps2 = kGravity.magnitude();

    /** Nominal FRC robot battery voltage (V). */
    public static final Voltage kNominalBatteryVoltage = Volts.of(12.0);

    /**
     * Minimum acceptable battery voltage under load before brownout concern (V).
     */
    public static final Voltage kBrownoutVoltage = Volts.of(6.8);

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTestControllerPort = 2;
    public static final double kDeadband = 0.05;
    public static final double kTriggerDeadband = 0.1;
    // Threshld when using trigger axis as a button
    public static final double kTriggerThreshold = 0.5;
    public static final int kDriverControllerXAxis = 0;
    public static final int kDriverControllerYAxis = 1;
    public static final int kDriverControllerRotAxis = 4;
    public static final File kDeployDirectory = Filesystem.getDeployDirectory();

    public static final double kTranslationSlowModeMultiplier = 0.45;
    public static final double kTranslationSuperSlowModeMultiplier = 0.2;
    public static final double kTurnSlowModeMultiplier = 0.4;

    public static final double kButtonPressDebounceSeconds = 0.1;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module canf be configured with one of three pinion gears: 12T,
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

  public static final class LauncherAndIntakeConstants {

    public static final Distance kBallReleaseHeight = Inches.of(20);

    /** High arc release angle (default, tuned for hub shots). */
    public static final Angle kBallReleaseAngleHigh = Degree.of(58.016961);
    /** Low arc release angle (alternative, flatter trajectory). */
    public static final Angle kBallReleaseAngleLow = Degree.of(48.0);

    private static SendableChooser<Angle> kReleaseAngleChooser = null;

    public static void initReleaseAngleChooser() {
      if (kReleaseAngleChooser != null)
        return;
      kReleaseAngleChooser = new SendableChooser<>();
      kReleaseAngleChooser.setDefaultOption("High Arc (58 deg)", kBallReleaseAngleHigh);
      kReleaseAngleChooser.addOption("Low Arc (48 deg)", kBallReleaseAngleLow);
      SmartDashboard.putData("Launch Angle", kReleaseAngleChooser);
    }

    /**
     * Returns the currently selected ball release angle.
     * Falls back to the high-arc angle if the chooser has not been initialised.
     */
    public static Angle kBallReleaseAngle() {
      if (kReleaseAngleChooser == null)
        return kBallReleaseAngleHigh;
      Angle selected = kReleaseAngleChooser.getSelected();
      return selected != null ? selected : kBallReleaseAngleHigh;
    }

    // Launch heading relative to bot heading (0 means launching straight forward,
    // positive is counterclockwise)
    public static final Rotation2d kLauncherBotHeading = Rotation2d.fromDegrees(180);

    public static final Distance kWheelRadius = Inches.of(2);
    // Empirical constant describing the ratio between wheel linear velocity and
    // ball launch velocity
    public static final double kWheelSlipCoefficient = 0.436;

    public static final int kLeaderCanSparkId = 9;
    public static final int kFollowerCanSparkId = 10;
    public static final int kMotorCanTalonId = 21;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final Distance[] kLaunchDistancesLookup = {
        // TODO fill in values from spreadsheet
    };
    public static final AngularVelocity[] kLaunchWheelSpeedLookup = {
        // TODO fill in values from spreadsheet
    };
    // The tolerance for using lookup values, if distance is not within tolerance of
    // any lookup entry, use interpolation
    public static final Distance kLaunchLookupTolerance = Meters.of(0.1);

    private static final double kRPMCurveA = 69.9;
    private static final double kRPMCurveB = -143;
    private static final double kRPMCurveC = 2640;
    // For fine tuning due to small changes
    // TODO replace with final value after testing
    private static final LoggedNetworkNumber kRPMCurveMultiplier = new LoggedNetworkNumber("/Tuning/RPMCurveMultiplier",
        1.0);
    public static final Function<Distance, AngularVelocity> kDistanceToRPMCurve = (Distance distance) -> {
      double d = distance.in(Meters);
      double rpm = kRPMCurveA * d * d + kRPMCurveB * d + kRPMCurveC;
      return RPM.of(rpm * kRPMCurveMultiplier.getAsDouble());
    };

    // Found using polynomial regression (degree 2)
    private static final double kTOFCurveA = -0.0481;
    private static final double kTOFCurveB = 0.401;
    private static final double kTOFCurveC = 0.425;
    // For fine tuning due to small changes
    // TODO replace with final value after testing
    private static final LoggedNetworkNumber kTOFCurveMultiplier = new LoggedNetworkNumber("/Tuning/TOFCurveMultiplier",
        1.0);
    public static final Function<Distance, AngularVelocity> kDistanceToTOFCurve = (Distance distance) -> {
      double d = distance.in(Meters);
      double rpm = kTOFCurveA * d * d + kTOFCurveB * d + kTOFCurveC;
      return RPM.of(rpm * kTOFCurveMultiplier.getAsDouble());
    };

    public static final Distance kMinLaunchDistance = Meters.of(1.8);

    static {
      // Minimum vertical velocity needed to reach hub height (from energy
      // conservation,
      // accounting for release height offset)
      double minVz = Math.sqrt(2 * 9.81
          * (FieldConstants.kHubHeight.in(Meters)
              - LauncherAndIntakeConstants.kBallReleaseHeight.in(Meters)));

      // Work backwards through the launch velocity chain to find the minimum RPM:
      // Vz = ballSpeed * sin(angle)
      // ballSpeed = wheelLinearSpeed * slipCoeff
      // wheelLinearSpeed = omega * radius
      double minOmegaRPM = (minVz
          / Math.sin(LauncherAndIntakeConstants.kBallReleaseAngleHigh.in(Radians))
          / LauncherAndIntakeConstants.kWheelSlipCoefficient
          / LauncherAndIntakeConstants.kWheelRadius.in(Meters))
          * (60.0 / (2 * Math.PI));

      // Invert the quadratic RPM curve:
      // kRPMCurveA*d² + kRPMCurveB*d + kRPMCurveC = minOmegaRPM
      // kRPMCurveA*d² + kRPMCurveB*d + (kRPMCurveC - minOmegaRPM) = 0
      double a = kRPMCurveA;
      double b = kRPMCurveB;
      double c = kRPMCurveC - minOmegaRPM;

      double discriminant = b * b - 4 * a * c;
      double minDist = discriminant >= 0
          ? (-b + Math.sqrt(discriminant)) / (2 * a)
          : 1.5; // fallback if curve never reaches minOmegaRPM

      // These calculations are not accurate enough, stick with a predetermined
      // constant
      // kMinLaunchDistance = Meters.of(Math.max(0, minDist));
      // Logger.recordOutput("Commands/LauncherCmd/MinLaunchDistance",
      // kMinLaunchDistance);
    }

    public static AngularVelocity linearVelocityToAngularVelocity(LinearVelocity ballSpeed) {
      double wheelLinearMps = ballSpeed.in(MetersPerSecond)
          / kWheelSlipCoefficient
          / kWheelRadius.in(Meters);
      return RadiansPerSecond.of(wheelLinearMps);
    }

    public static final Distance kTestLaunchRadius = Meters.of(2.0);
    public static final Time kTestBallAirTime = Seconds.of(0.5);

    public static final boolean kLeadShots = true;

    public static final double kMotorReduction = 45.0 / 56.0;
    public static final Current kSmartCurrentLimit = Amps.of(40);

    public static final double kPIDLauncherControllerP = 1.2e-4;
    public static final double kPIDLauncherControllerI = 1e-10;
    public static final double kPIDLauncherControllerD = 1e-4;
    public static final double kPIDLauncherControllerFF = (1.0
        / (NeoMotorConstants.kFreeSpeed.in(RPM) * kMotorReduction)) * 1.00;
    public static final double kOutputRangeMin = -1.0;
    public static final double kOutputRangeMax = 1.0;

    // Setpoints
    public static final AngularVelocity kIntakeRPMSetpoint = RPM.of(1000);
    public static final AngularVelocity kPassRPMSetpoint = RPM.of(4500);
    // Visionless backup setpoints
    public static final AngularVelocity kBumpRPMSetpoint = RPM.of(2500);
    public static final AngularVelocity kTrenchRPMSetpoint = RPM.of(3020);
    public static final AngularVelocity kTowerRPMSetpoint = RPM.of(2910);
    public static final AngularVelocity kCornerRPMSetpoint = kDistanceToRPMCurve.apply(Meters.of(5.4539));
    public static final AngularVelocity kUnloadRPMSetpoint = RPM.of(1500);
    // RPM increment per second when doing manual offset
    public static final AngularVelocity kManualRPMOffsetPerSecond = RPM.of(50);

    public static final ClosedLoopSlot kSlotHigh = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot kSlotLow = ClosedLoopSlot.kSlot1;

    public static final SparkMaxConfig kLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kFollowerConfig = new SparkMaxConfig();

    public static final AngularVelocity kRPMTolerance = RPM.of(100);

    public static final Time kAutoAimIndexDebounce = Seconds.of(0.05);

    static {
      kLeaderConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit((int) kSmartCurrentLimit.magnitude())
          .inverted(true)
          .voltageCompensation(12.0);
      kLeaderConfig.encoder
          .velocityConversionFactor(kMotorReduction);
      kLeaderConfig.closedLoop
          .pid(kPIDLauncherControllerP, kPIDLauncherControllerI,
              kPIDLauncherControllerD)
          .outputRange(kOutputRangeMin, kOutputRangeMax).feedForward
          .kV(kPIDLauncherControllerFF);

      kFollowerConfig
          .smartCurrentLimit((int) kSmartCurrentLimit.magnitude())
          .idleMode(IdleMode.kCoast)
          .inverted(false)
          .voltageCompensation(12.0);
      kFollowerConfig.follow(kLeaderCanSparkId, true);
    }

    // Trajectory Buffer
    public static Distance kEpsilonBuffer = Meters.of(0.03);
  }

  public static final class PassConstants {

    /** Depot-side bump pass target (closer to bottom of field). */
    public static final Translation2d kBlueBumpPassPose1 = new Translation2d(2.6, 2.6);

    /** Outpost-side bump pass target (closer to top of field). */
    public static final Translation2d kBlueBumpPassPose2 = new Translation2d(2.6, 5.5);

    /**
     * Both neutral-zone pass targets as an array for convenience.
     * Index 0 = depot side, index 1 = outpost side.
     */
    public static final Translation2d[] kBlueBumpPassTargets = { kBlueBumpPassPose1, kBlueBumpPassPose2 };

    public static final Distance kPassTargetHeight = Meters.of(0.0);

    // Enemy-zone dump target.
    // When in the enemy zone we just blast the ball to the back of our
    // alliance zone so a robot can collect it.

    /**
     * x coord of the landing zone; well inside the alliance
     * zone, away from the hub.
     */
    public static final Distance kBlueDumpTargetX = Meters.of(2.0);

    /**
     * Y coordinate of the dump target; field centre line so either alliance
     * robot can reach it.
     */
    public static final Distance kBlueDumpTargetY = Meters.of(4.0); // approx field centre

    /** Height to target for the enemy-zone dump shot (ground). */
    public static final Distance kDumpTargetHeight = Meters.of(0.0);

    /**
     * Effective radius of the hub structure used when checking whether a pass
     * trajectory's ground-plane path intersects the hub.
     * Add a small safety margin on top of the physical half-width.
     */
    public static final Distance kHubLOSRadius = FieldConstants.kHubInsideWidth.div(2.0).plus(Meters.of(0.15));

    public static final Distance kPassLandingTolerance = Meters.of(0.5);
  }

  public static final class ClimbAlignConstants {
    // AprilTag IDs for the climb towers
    public static final int kBlueLeftTagId = 31;
    public static final int kBlueRightTagId = 32;
    public static final int kRedLeftTagId = 15;
    public static final int kRedRightTagId = 16;

    // How far forward from the wall (along tag facing direction) the robot stands.
    // Measure: distance from the wall to where your hook needs to be to engage the
    // rung.
    public static final Distance kStandoffDistance = Meters.of(0.55); // TODO: TUNE

    // How far outward (away from center) from the tag the robot targets on the
    // rung.
    public static final Distance kOuterRungOffset = Meters.of(0.254); // 10 inches (TUNE)

    // Extra offset PathPlanner pathfinds to before the fine-align phase takes over
    public static final Distance kCoarseApproachOffset = Meters.of(0.8);
    public static final double kCoarseApproachEndVelocity = 0.5; // m/s into fine-align

    // Fine-align PID (shared for X and Y)
    public static final double kAlignP = 2.5;
    public static final double kAlignI = 0.0;
    public static final double kAlignD = 0.05;

    // Settle tolerances
    public static final Distance kTranslationTolerance = Meters.of(0.04);
    public static final Angle kRotationTolerance = Radians.of(Math.toRadians(2.0));

    // Max speed during final PID alignment phase
    public static final LinearVelocity kAlignMaxSpeed = MetersPerSecond.of(1.2);
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
    public static final LinearVelocity kBumpLinearVelocity = MetersPerSecond.of(4.8);
    public static final LinearAcceleration kBumpLinearAcceleration = MetersPerSecondPerSecond.of(3);
    public static final AngularVelocity kBumpAngularVelocity = RadiansPerSecond.of(2 * Math.PI);
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

    // How close to the trench the robot can get before climbers automatically lower
    // to prevent getting caught under the trench
    public static final Distance kTrenchSafetyMargin = Meters.of(1.5);

    // To be used by PathPlanner
    public static final double kPTranslationController = 0.5;
    public static final double kITranslationController = 0.01;
    public static final double kDTranslationController = 0.0;

    public static final double kPThetaController = 5;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0.15;

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

    public static final Pose2d kDepotClimb = new Pose2d(Meters.of(0.746), Meters.of(4.193),
        Rotation2d.fromDegrees(180));
    public static final Pose2d kOutpostClimb = new Pose2d(Meters.of(0.746), Meters.of(2.981),
        Rotation2d.fromDegrees(180));

    public static final SwerveModuleState[] kXLockModuleStates = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
    };

    public static final LinearVelocity kTooCloseBackAwaySpeed = MetersPerSecond.of(1.5);
    public static final Rotation2d kLaunchHeadingTolerance = Rotation2d.fromDegrees(10);
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

    public static final PathConstraints L1ClimbConstraints = new PathConstraints(
        3.0, 2.5,
        3 * Math.PI, 4 * Math.PI);

    public static final AngularVelocity kLauncherRPM = RPM.of(0); // TODO replace with auto launch RPM
    public static final LinearVelocity crossingEndVelocity = MetersPerSecond.of(0); // To be updated

    public static PathPlannerPath intakeLeftPath;
    public static PathPlannerPath depotClimbPath;
    public static PathPlannerPath outpostClimbPath;
    public static PathPlannerPath depotClimbVeryCurvedPath;
    public static PathPlannerPath outpostClimbVeryCurvedPath;
    public static PathPlannerPath outpostPath;
    public static PathPlannerPath depotPath;
    public static PathPlannerPath neutralZoneTrenchDepot;
    public static PathPlannerPath neutralZoneBumpDepot;
    public static PathPlannerPath neutralZoneTrenchOutpost;
    public static PathPlannerPath neutralZoneBumpOutpost;
    public static PathConstraints kPathfindingConstraints;

    // TODO requires testing
    public static Distance kAutoNeutralZoneX = Meters.of(7.0);
    public static Pose2d outpostPose = new Pose2d(Meters.of(0.455), Meters.of(0.651), Rotation2d.kZero);
    public static Time kAutoOutpostIntakeTime = Seconds.of(2); // TODO to be updated
    public static Distance kAutoLaunchDistanceFromHubX = Meters.of(2.5);
    // TODO: Test this value
    public static final Time kAutoLaunch8Time = Seconds.of(6);
    public static final Time kAutoLaunch32Time = Seconds.of(8);
    public static final Time kDefaultAutoDelay = Seconds.of(15);

    public static Pose2d depotCorner = new Pose2d(Meters.of(0.5), Meters.of(7.5), Rotation2d.kZero);
    public static Pose2d outpostCorner = new Pose2d(Meters.of(0.5), Meters.of(0.5), Rotation2d.kZero);

    static {
      try {
        intakeLeftPath = PathPlannerPath.fromPathFile("Intake Left");
        depotClimbPath = PathPlannerPath.fromPathFile("Depot(L1 Climb)");
        outpostClimbPath = PathPlannerPath.fromPathFile("Outpost(L1 Climb)");
        depotClimbVeryCurvedPath = PathPlannerPath.fromPathFile("Depot(L1 Climb) Extra Curve");
        outpostClimbVeryCurvedPath = PathPlannerPath.fromPathFile("Outpost(L1 Climb) Extra Curve");
        depotPath = PathPlannerPath.fromPathFile("Depot Intake");
        outpostPath = PathPlannerPath.fromPathFile("Outpost Intake");
        neutralZoneTrenchDepot = PathPlannerPath.fromPathFile("Path to Neutral Zone (Depot, Trench)");
        neutralZoneBumpDepot = PathPlannerPath.fromPathFile("Path to Neutral Zone (Depot, Bump)");
        neutralZoneTrenchOutpost = PathPlannerPath.fromPathFile("Path to Neutral Zone (Outpost, Trench)");
        neutralZoneBumpOutpost = PathPlannerPath.fromPathFile("Path to Neutral Zone (Outpost, Bump)");
      } catch (Exception e) {
        DriverStation.reportError("AutoConstants: Failed to load path file: " + e.getMessage(), true);
        e.printStackTrace();
      }
    }

    // Defensive helper; avoids InInitializerError from IndexOutOfBounds
    private static Translation2d safeFirstPoint(PathPlannerPath path) {
      if (path == null)
        return new Translation2d();
      List<?> points = path.getAllPathPoints();
      if (points.isEmpty())
        return new Translation2d();
      return ((PathPoint) points.get(0)).position;
    }

    public static final Translation2d depotStartPoint = safeFirstPoint(depotClimbPath);
    public static final Translation2d outpostStartPoint = safeFirstPoint(outpostClimbPath);
    public static final Translation2d intakeLeftStartPoint = safeFirstPoint(intakeLeftPath);

    public static final PathPlannerPath[] climbPaths = { depotClimbPath, outpostClimbPath };

    public static final Translation2d[] climbPathWaypoints = new Translation2d[] {
        new Translation2d(Meters.of(depotStartPoint.getX()).in(Meters),
            Meters.of(depotStartPoint.getY()).in(Meters)),
        new Translation2d(Meters.of(outpostStartPoint.getX()).in(Meters),
            Meters.of(outpostStartPoint.getY()).in(Meters)),
        new Translation2d(FieldConstants.kFieldLengthX.minus(Meters.of(depotStartPoint.getX())).in(Meters),
            FieldConstants.kFieldWidthY.minus(Meters.of(depotStartPoint.getY())).in(Meters)),
        new Translation2d(FieldConstants.kFieldLengthX.minus(Meters.of(outpostStartPoint.getX())).in(Meters),
            FieldConstants.kFieldWidthY.minus(Meters.of(outpostStartPoint.getY())).in(Meters)),
    };

    public static final double kAlignTowerTimeoutSeconds = 3;

    public static final double kFinalAlignSpeed = -0.15;
    public static final Time kFinalAlignTime = Seconds.of(0.4);

    public static final Time kIntakeDeployDriveTime = Seconds.of(0.6);
    public static final Time kIntakeDeployStopTime = Seconds.of(0.2);

    public static final Time kIntakeDeployIntakeTime = Seconds.of(1.0);
    // public static final double kIntakeDeploySpeedX = 1.0;
    // public static final double kIntakeDeploySpeedY = 0.0;
    // public static final double kIntakeDeploySpeedTheta = 0.0;
    public static final Time kDepotIntakeTime = Seconds.of(2);
    public static final double kDepotIntakeDriveSpeed = -0.4;

  }

  public static final class OTBIntakeConstants {
    public static final int kRollerCanId = 15;
    public static final int kWinchCanId = 20;
    public static final MotorType kMotorType = MotorType.kBrushless;

    // TODO: Verify values for these reductions
    public static final double kRollerReduction = 1.0 / 10.0;

    public static double kOuttakeSpeed = 1;
    public static double kIntakeSpeed = -1;

    public static double kDeploySpeed = 0.25;
    public static double kRetractSpeed = -0.25;

    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kWinchConfig = new SparkMaxConfig();

    static {
      kRollerConfig
          .idleMode(IdleMode.kCoast)
          // TODO: set this value depending on motor type, 20 is good for a 550, too low
          // for a neo
          .smartCurrentLimit(20)
          // TODO: invert so + is intaking, - is outaking
          .inverted(false);
    }

    static {
      kWinchConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(false);
    }
  }

  public static final class IndexerConstants {
    public static final int kWheelCanId = 11;
    public static final int kTreadmillCanId = 12;
    public static final MotorType kMotorType = MotorType.kBrushless;

    /**
     * Multiplier that decides whether + or - inputs move the fuel towards the
     * launcher.
     */
    public static final double kDirectionConstant = -1.0;

    public static final double kTreadmillSpeed = 1;
    public static final double kWheelSpeed = 1.0;

    public static final double kWheelMotorReduction = 1.0;
    public static final double kWheelDiameterMeters = 0.17;

    // TODO: Tune these values
    public static final double kTreadmillLaunchIndexPercent = 1;
    public static final double kTreadmillStoreIndexPercent = 1;
    public static final double kWheelLaunchIndexPercent = 1;

    // Treadmill pulse timing
    public static final PulsingTreadmillCmd.PulseShape kDefaultPulseShape = PulsingTreadmillCmd.PulseShape.SIN_SQUARED;

    // Total period ~0.4s, 65% on / 35% off
    public static final double kPulseDutyCycle = 0.8;
    public static final double kPulsePeriod = 0.2;

    public static final double kPulseOnSeconds = kPulsePeriod * kPulseDutyCycle; // 0.26s
    public static final double kPulseOffSeconds = kPulsePeriod * (1 - kPulseDutyCycle); // 0.14s

    public static final SparkMaxConfig kWheelConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kTreadmillConfig = new SparkMaxConfig();

    static {
      kWheelConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      // kWheelConfig.encoder
      // .velocityConversionFactor(kWheelDiameterMeters * Math.PI /
      // kWheelMotorReduction / 60);

      kTreadmillConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(10);
    }
  }

  public static final class ClimberConstants {

    // CAN IDs
    public static final int kLeftId = 13; // leader
    public static final int kRightId = 14; // follower
    public static final MotorType kMotorType = MotorType.kBrushless;

    // DIO port for the bottom limit switch
    public static final int kLeftBottomLimitSwitchDIOPort = 1;
    public static final int kRightBottomLimitSwitchDIOPort = 0;

    // Travel limits
    /**
     * Time it takes to raise the climber to climbing position from the bottom
     * (where the limit switch returns true)
     */
    public static final Time kTimeFromBottomToRaisedPosition = Seconds.of(2);
    /**
     * Time it takes to pull the climber down from raised position, to make the
     * robot climb
     */
    public static final Time kTimeFromRaisedToClimbedPosition = kTimeFromBottomToRaisedPosition.times(0.75);

    // TODO: Test these setpoints (again)
    public static final Distance kRaisePosition = Inches.of(3);
    public static final Distance kLatchPosition = Inches.of(17.6);

    public static final Distance kMinLength = Inches.of(0);
    public static final Distance kMaxLength = Inches.of(17.6);

    public static final double kSettledVelocityThresholdInchesPerSec = 0.25; // arms have stopped moving into raise
                                                                             // position
    public static final double kStallVelocityThresholdInchesPerSec = 0.1; // arms have stalled on bar during pull

    // PID position tolerance
    /** Encoder error below which atSetpoint() returns true. */
    public static final Distance kPositionTolerance = Inches.of(0.2);

    // Winch / spool geometry
    public static final Distance kWhinchDrumDiameter = Inches.of(1.625);
    // Without
    public static final Distance kSpoolCableDiameter = Inches.of(0.25);
    public static final int kMaxSpoolLayers = 5;
    public static final int kMinSpoolLayers = 1;

    public static final Distance kAverageEffectiveDiameter = kWhinchDrumDiameter
        .plus(kSpoolCableDiameter.times((kMaxSpoolLayers + kMinSpoolLayers) / 2.0));

    public static final double kGearReduction = 125.0;

    public static final double kRotationsToInchesConversion = kAverageEffectiveDiameter.in(Inches) * Math.PI
        / kGearReduction;
    // Current limits
    public static final Current kSmartCurrentLimit = Amps.of(40);
    public static final Current kStatorCurrentLimit = Amps.of(40); // TalonFX only

    public static final double kOutputRangeMin = -1.0;
    public static final double kOutputRangeMax = 1.0;
    public static final double kOutputUp = 1.0;

    // TalonFX sensor ratio
    public static final double kSensorToMechanismRatio = 1.0;

    // PID gains
    public static final double kPIDClimberControllerP = 10;
    public static final double kPIDClimberControllerI = 0.0;
    public static final double kPIDClimberControllerD = 0.0;

    public static final double kStowCrawlSpeed = -1;

    // SparkMax configs
    public static final SparkMaxConfig kConfigLeft = new SparkMaxConfig();
    public static final SparkMaxConfig kConfigRight = new SparkMaxConfig();

    static {
      kConfigLeft.smartCurrentLimit((int) kSmartCurrentLimit.in(Amps));
      kConfigLeft.inverted(true); // TODO: Plug in one motor at a time and run ClimbPercentCmd with a small
                                  // positive value like 0.1. Watch which direction the arm moves: If it goes up,
                                  // that motor needs inverted(false) If it goes down, that motor needs
                                  // inverted(true)
      kConfigLeft.encoder.positionConversionFactor(kRotationsToInchesConversion);
      kConfigLeft.voltageCompensation(12.0);
      kConfigLeft.closedLoop
          .p(kPIDClimberControllerP)
          .i(kPIDClimberControllerI)
          .d(kPIDClimberControllerD)
          .outputRange(kOutputRangeMin, kOutputRangeMax);

      kConfigRight.smartCurrentLimit((int) kSmartCurrentLimit.in(Amps));
      kConfigRight.inverted(true); // TODO: Plug in one motor at a time and run ClimbPercentCmd with a small
                                   // positive value like 0.1. Watch which direction the arm moves: If it goes up,
                                   // that motor needs inverted(false) If it goes down, that motor needs
                                   // inverted(true)
      kConfigRight.encoder.positionConversionFactor(kRotationsToInchesConversion);
      kConfigRight.closedLoop
          .p(kPIDClimberControllerP)
          .i(kPIDClimberControllerI)
          .d(kPIDClimberControllerD)
          .outputRange(kOutputRangeMin, kOutputRangeMax);
      kConfigRight.voltageCompensation(12.0);
    }

  }

  public static final class ClimbAlignmentConstants {
    public static final int[] kBlueTowerTagIds = { 31, 32 };
    public static final int[] kRedTowerTagIds = { 15, 16 };

    // TODO: Measure on real field — approach distance from Tower face to robot
    // center at hook engagement
    public static final double kStandoffDistanceMeters = 0.6;

    // TODO: Measure on real robot — lateral distance from robot center to side hook
    // contact point
    public static final double kHookLateralOffsetMeters = 0.25;

    // GO thresholds
    public static final double kTranslationToleranceMeters = 0.15;
    public static final double kHeadingToleranceDegrees = 5.0;

    // Tag detection quality thresholds
    public static final double kMinTagArea = 0.1;
    public static final double kMaxAmbiguity = 0.5;
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
    public static final Supplier<GyroSimulation> kSimulatedGyro = COTS.ofGenericGyro();
    public static final DCMotor kSimulatedDrivingMotor = DCMotor.getNEO(1);
    public static final DCMotor kSimulatedTurningMotor = DCMotor.getNeo550(1);
    public static final double kSimulatedCoefficentOfFriction = COTS.WHEELS.COLSONS.cof;
    public static final int kGearRatioLevel = 2;

    /** Starting pose when in blue alliance */
    public static final Pose2d kStartingPose = new Pose2d(3.6, 0.6, Rotation2d.fromRadians(0));

    // Whether the bump should have defined collision
    public static final boolean kSimBumpCollision = false;

    // Simulation camera properties
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

    // Drivetrain simulation configs (moved here from Configs.Simulation)
    public static final SwerveModuleSimulationConfig kSwerveModuleSimConfig = COTS.ofMAXSwerve(
        kSimulatedDrivingMotor,
        kSimulatedTurningMotor,
        kSimulatedCoefficentOfFriction,
        kGearRatioLevel);

    // NOTE: drivetrainConfig depends on SwerveConfig which is set at runtime,
    // so it is built lazily in RobotContainer rather than here as a static final.

    // TODO: Measure this value
    public static final AngularVelocity kSimulatedMaxLauncherSpeed = RPM.of(6000);

    // Climber simulation
    public static final DCMotor kSimulatedSparkMaxClimberMotor = DCMotor.getNEO(1);
    public static final LinearVelocity kSimulatedMaxClimberSpeed = InchesPerSecond.of(4);
    public static final Distance kSimulatedMaxClimberHeight = Inches.of(10.0);
    // When climbers are within this from 0, the simulated bottom limit switches
    // will indicate that it's at the bottom
    public static final Distance kSimulatedClimberBottomTolerance = kSimulatedMaxClimberHeight.times(0.05);
  }

  public static final class PhotonConstants {
    // Camera profiles - each camera's configuration in one place
    public static final CameraProfile kCamera1Profile = new CameraProfile(
        "Arducam_1",
        Radians.of(0.0), // roll
        Radians.of(0.1301), // pitch
        Radians.of(0.0), // yaw
        Meters.of(0.307), // x
        Meters.of(0.180), // y
        Meters.of(0.750), // z
        VecBuilder.fill(0.3, 0.3, 0.3) // standard deviation
    );

    public static final CameraProfile kCamera2Profile = new CameraProfile(
        "Arducam_2",
        Radians.of(0.0), // roll
        Radians.of(0.0), // pitch
        Radians.of(Math.PI), // yaw
        Meters.of(-0.3327), // x
        Meters.of(0.0), // y
        Meters.of(0.3708), // z
        VecBuilder.fill(0.9, 0.9, 0.9) // standard deviation
    );

    public static final CameraProfile kCamera3Profile = new CameraProfile(
        "Arducam_3",
        Radians.of(0.0), // roll
        Radians.of(0.0), // pitch
        Radians.of(0.7069), // yaw
        Meters.of(0.238), // x
        Meters.of(-0.294), // y
        Meters.of(0.625), // z
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

    public static final Distance kHeightTolerance = Inches.of(6.5).plus(Meters.of(0.5)); // meters above and below
                                                                                         // ground (includes bump
                                                                                         // height)
    public static final double kAmbiguityDiscardThreshold = 0.2; // ignore targets above this value
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
    public static final Distance kVisionJumpDistanceThreshold = Meters.of(1.0);

    public static final double kVisionHighAmbiguityMultiplier = 1.5;

    public static final int kRejectedPosesQueueSize = 10;

    public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR; // Multi tag should work
                                                                                                // now that the field
                                                                                                // layouts are
                                                                                                // consistent
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kfieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);

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
    public static final Distance kAcceptedLaunchingZone = kAllianceZoneXLength;// .plus(Inches.of(27 / Math.sqrt(2)));
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
    public static final Distance kHubXBlue = kAllianceWallToHubCenter;
    public static final Distance kHubXRed = kFieldLengthX.minus(kAllianceWallToHubCenter);

    public static final Distance kHubHeight = Inches.of(72);
    // Distance between opposite sides of the upper hexagon
    public static final Distance kHubInsideWidth = Inches.of(41.73);

    public static final Translation3d kBlueHubTranslation3d = new Translation3d(kHubXBlue.in(Meters), kHubY.in(Meters),
        kHubHeight.in(Meters));
    public static final Translation3d kRedHubTranslation3d = new Translation3d(kHubXRed.in(Meters),
        kHubY.in(Meters), kHubHeight.in(Meters));

    public static final Translation2d kBlueHubTranslation2d = kBlueHubTranslation3d.toTranslation2d();
    public static final Translation2d kRedHubTranslation2d = kRedHubTranslation3d.toTranslation2d();

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