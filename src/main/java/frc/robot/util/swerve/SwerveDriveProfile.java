package frc.robot.util.swerve;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Record class representing a swerve drive configuration profile.
 * Contains all the necessary parameters to configure a swerve drive system.
 */
public record SwerveDriveProfile(
    int[] driveCanIds,
    int[] turnCanIds,
    Distance wheelDiameterMeters,
    double driveReduction,
    LinearVelocity maxSpeedMps,
    Distance trackWidthMeters,
    Distance wheelBaseMeters,
    Distance bumperLength,
    Distance bumperWidth,
    int profileId) {
  public String getName() {
    return switch (profileId) {
      case 1 -> "CompBot";
      case 2 -> "SpongeBot";
      case 3 -> "OffSeasonSwerve";
      default -> "CompBot";
    };
  }
}