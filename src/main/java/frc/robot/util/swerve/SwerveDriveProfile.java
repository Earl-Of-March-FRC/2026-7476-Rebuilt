package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.Drivetrain.Gyro;
import frc.robot.subsystems.Drivetrain.GyroADXRS450;
import frc.robot.subsystems.Drivetrain.GyroNavX;

/**
 * Record class representing a swerve drive configuration profile.
 * Contains all the necessary parameters to configure a swerve drive system.
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
    NavXComType gyroComType,
    SwerveDriveProfileID profileId) {

  public static enum SwerveDriveProfileID {
    COMP_BOT,
    SPONGE_BOT,
    OFF_SEASON_SWERVE
  }

  /**
   * SpongeBot swerve drive configuration.
   */
  public static final SwerveDriveProfile COMP_BOT = new SwerveDriveProfile(
      new int[] { 6, 4, 8, 2 },
      new int[] { 5, 3, 7, 1 },
      Inches.of(3),
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
      // 12 teeth on the motor pinion 15 teeth on the bevel pinion
      (45.0 * 22) / (12 * 15),
      1, // placeholder wheel COF
      Amps.of(60), // Safe current limit for NEOs
      MetersPerSecond.of(4.8),
      RadiansPerSecond.of(2 * Math.PI),
      MetersPerSecondPerSecond.of(3),
      Inches.of(23.5),
      Inches.of(23.5),
      Meters.of(0.75),
      Meters.of(0.75),
      Kilograms.of(74.088), // PathPlanner default, not accurate
      KilogramSquareMeters.of(6.883), // PathPlanner default, not accurate
      NavXComType.kMXP_SPI,
      SwerveDriveProfileID.COMP_BOT);

  public static final SwerveDriveProfile SPONGE_BOT = new SwerveDriveProfile(
      new int[] { 1, 3, 5, 7 },
      new int[] { 2, 4, 6, 8 },
      Inches.of(3),
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
      // 13 teeth on the motor pinion, 15 teeth on the bevel pinion
      (45.0 * 22) / (13 * 15),
      1, // placeholder wheel COF
      Amps.of(60), // Safe current limit for NEOs
      MetersPerSecond.of(4.8),
      RadiansPerSecond.of(2 * Math.PI),
      MetersPerSecondPerSecond.of(3),
      Inches.of(24),
      Inches.of(24),
      Meters.of(0.75),
      Meters.of(0.75),
      Kilograms.of(74.088), // PathPlanner default, not accurate
      KilogramSquareMeters.of(6.883), // PathPlanner default, not accurate
      NavXComType.kUSB1,
      SwerveDriveProfileID.SPONGE_BOT);

  /**
   * Off-season swerve drive configuration.
   */
  public static final SwerveDriveProfile OFF_SEASON_SWERVE = new SwerveDriveProfile(
      new int[] { 5, 8, 6, 7 },
      new int[] { 1, 4, 2, 3 },
      Inches.of(3),
      (45.0 * 22) / (13 * 15),
      1, // placeholder wheel COF
      Amps.of(60), // Safe current limit for NEOs
      MetersPerSecond.of(4.8),
      RadiansPerSecond.of(2 * Math.PI),
      MetersPerSecondPerSecond.of(3),
      Inches.of(24),
      Inches.of(24),
      Meters.of(0.75),
      Meters.of(0.75),
      Kilograms.of(74.088), // PathPlanner default, not accurate
      KilogramSquareMeters.of(6.883), // PathPlanner default, not accurate
      NavXComType.kMXP_SPI,
      SwerveDriveProfileID.OFF_SEASON_SWERVE);

  public String getName() {
    return switch (profileId) {
      case COMP_BOT -> "CompBot";
      case SPONGE_BOT -> "SpongeBot";
      case OFF_SEASON_SWERVE -> "OffSeasonSwerve";
      default -> "CompBot";
    };
  }
}