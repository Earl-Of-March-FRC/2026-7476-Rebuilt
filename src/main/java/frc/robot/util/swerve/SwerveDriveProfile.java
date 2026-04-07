package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.vision.CameraProfile;

/**
 * Record class representing a swerve drive configuration profile.
 * Contains all the necessary parameters to configure a swerve drive system,
 * including camera configurations for vision and per-robot swerve module PID
 * values.
 */
public record SwerveDriveProfile(
    int[] driveCanIds,
    int[] turnCanIds,
    Distance wheelDiameterMeters,
    double driveReduction,
    double wheelCof,
    Current driveCurrentLimitAmps,
    LinearVelocity maxSpeedMps,
    AngularVelocity maxAngularSpeedRps,
    LinearAcceleration maxLinearAccelerationRps2,
    Distance trackWidthMeters,
    Distance wheelBaseMeters,
    Distance bumperLength,
    Distance bumperWidth,
    Mass robotMass,
    MomentOfInertia robotMOI,
    GyroType gyro,
    CameraProfile[] cameraProfiles,
    boolean visionUsesDynamicStandardDeviations,
    SwerveDriveProfileID profileId,
    // Swerve module drive motor PID and config
    double drivingP,
    double drivingI,
    double drivingD,
    int driveSmartCurrentLimit,
    // Swerve module turn motor PID and config
    double turningP,
    double turningI,
    double turningD,
    int turnSmartCurrentLimit,
    int driveFreeCurrentLimit) {

  public static enum SwerveDriveProfileID {
    COMP_BOT,
    SPONGE_BOT,
    OFF_SEASON_SWERVE
  }

  public static enum GyroType {
    NavX_MXP_SPI,
    NavX_USB1,
    ADXRS450
  }

  public String getGyroName() {
    return gyro().toString();
  }

  /**
   * CompBot swerve drive configuration.
   */
  public static final SwerveDriveProfile COMP_BOT = new SwerveDriveProfile(
      new int[] { 6, 4, 8, 2 }, // drive CAN IDs: FL, FR, BL, BR
      new int[] { 5, 3, 7, 1 }, // turn CAN IDs: FL, FR, BL, BR
      Inches.of(3),
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
      // 12 teeth on the motor pinion, 15 teeth on the bevel pinion
      (45.0 * 22) / (12 * 15),
      1.0, // placeholder wheel COF
      Amps.of(60),
      MetersPerSecond.of(4.8),
      RadiansPerSecond.of(2 * Math.PI),
      MetersPerSecondPerSecond.of(3),
      Inches.of(23.5),
      Inches.of(23.5),
      Meters.of(0.75),
      Meters.of(0.75),
      Pounds.of(104.85),
      KilogramSquareMeters.of(5),
      GyroType.NavX_MXP_SPI,
      new CameraProfile[] {
          new CameraProfile(
              "Launcher",
              Degrees.of(0.0), // roll
              Degrees.of(-15), // pitchs
              Degrees.of(180.0), // yaw
              Meters.of(0.0187481718), // x // 0.0204789786
              Meters.of(0.0781538188), // y // -0.0190500254
              Meters.of(0.5350118586), // z // 0.4919337634
              VecBuilder.fill(0.3, 0.3, 0.3)),
          new CameraProfile(
              "Climber",
              Degrees.of(0.0), // roll
              Degrees.of(-20), // pitch
              Degrees.of(0), // yaw
              Meters.of(0.3003719164), // x // 0.3056178546
              Meters.of(-0.2467180994), // y // -0.2415122424
              Meters.of(0.4994001188), // z // 0.5052718622
              VecBuilder.fill(0.9, 0.9, 0.9)),
          new CameraProfile(
              "Side",
              Degrees.of(0.0), // roll
              Degrees.of(-12), // pitch
              Degrees.of(90), // yaw
              Meters.of(0.300912657), // x // 0.3107901836
              Meters.of(0.2467181248), // y // 0.1993096344
              Meters.of(0.405), // z // 0.413481266
              VecBuilder.fill(0.5, 0.5, 0.5))
      },
      true,
      SwerveDriveProfileID.COMP_BOT,
      // Drive motor PID
      0.04, 0.0, 0.0,
      30,
      // Turn motor PID
      1.0, 0.0, 0.0,
      15, 20);

  /**
   * SpongeBot swerve drive configuration with 3 cameras.
   */
  public static final SwerveDriveProfile SPONGE_BOT = new SwerveDriveProfile(
      new int[] { 1, 3, 5, 7 }, // drive CAN IDs: FL, FR, BL, BR
      new int[] { 2, 4, 6, 8 }, // turn CAN IDs: FL, FR, BL, BR
      Inches.of(3),
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
      // 13 teeth on the motor pinion, 15 teeth on the bevel pinion
      (45.0 * 22) / (13 * 15),
      1.0, // placeholder wheel COF
      Amps.of(60),
      MetersPerSecond.of(3),
      RadiansPerSecond.of(2 * Math.PI),
      MetersPerSecondPerSecond.of(2.5),
      Inches.of(24),
      Inches.of(24),
      Meters.of(0.75),
      Meters.of(0.75),
      Kilograms.of(74.088),
      KilogramSquareMeters.of(6.883),
      GyroType.NavX_USB1,
      new CameraProfile[] {
          new CameraProfile(
              "Arducam_1",
              Radians.of(0.0), // roll
              Radians.of(0.1301), // pitch
              Radians.of(0.0), // yaw
              Meters.of(0.307), // x
              Meters.of(0.180), // y
              Meters.of(0.750), // z
              VecBuilder.fill(0.3, 0.3, 0.3)),
          new CameraProfile(
              "Arducam_2",
              Radians.of(0.0), // roll
              Radians.of(0.0), // pitch
              Radians.of(0.7069), // yaw
              Meters.of(0.238), // x
              Meters.of(-0.294), // y
              Meters.of(0.625), // z
              VecBuilder.fill(0.9, 0.9, 0.9)),
          new CameraProfile(
              "Arducam_3",
              Radians.of(0.0), // roll
              Radians.of(0.0), // pitch
              Radians.of(Math.PI), // yaw
              Meters.of(-0.3327), // x
              Meters.of(0.0), // y
              Meters.of(0.3708), // z
              VecBuilder.fill(0.5, 0.5, 0.5))
      },
      true,
      SwerveDriveProfileID.SPONGE_BOT,
      // Drive motor PID
      0.04, 0.0, 0.0,
      50,
      // Turn motor PID
      1.0, 0.0, 0.0,
      20, 20);

  /**
   * Off-season swerve drive configuration with 0 cameras.
   */
  public static final SwerveDriveProfile OFF_SEASON_SWERVE = new SwerveDriveProfile(
      new int[] { 5, 8, 6, 7 }, // drive CAN IDs: FL, FR, BL, BR
      new int[] { 1, 4, 2, 3 }, // turn CAN IDs: FL, FR, BL, BR
      Inches.of(3),
      (45.0 * 22) / (13 * 15),
      1.0, // placeholder wheel COF
      Amps.of(60),
      MetersPerSecond.of(4.8),
      RadiansPerSecond.of(4 * Math.PI),
      MetersPerSecondPerSecond.of(4.8),
      Inches.of(24),
      Inches.of(24),
      Meters.of(0.75),
      Meters.of(0.75),
      Kilograms.of(74.088),
      KilogramSquareMeters.of(6.883),
      GyroType.ADXRS450,
      new CameraProfile[] {},
      true,
      SwerveDriveProfileID.OFF_SEASON_SWERVE,
      // Drive motor PID
      0.04, 0.0, 0.0,
      50,
      // Turn motor PID
      1.0, 0.0, 0.0,
      20, 20);

  public String getName() {
    return switch (profileId) {
      case COMP_BOT -> "CompBot";
      case SPONGE_BOT -> "SpongeBot";
      case OFF_SEASON_SWERVE -> "OffSeasonSwerve";
      default -> "CompBot";
    };
  }

  /**
   * Gets the number of cameras in this profile.
   *
   * @return Number of cameras configured
   */
  public int getNumCameras() {
    return cameraProfiles == null ? 0 : cameraProfiles.length;
  }
}