package frc.robot.util.swerve;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

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