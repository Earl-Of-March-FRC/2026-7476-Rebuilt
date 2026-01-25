package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Container class for predefined swerve drive profiles.
 * Each profile contains the complete configuration for a specific robot.
 */
public final class SwerveProfiles {

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
      1);

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
      2);

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
      3);

  // Private constructor to prevent instantiation
  private SwerveProfiles() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
}