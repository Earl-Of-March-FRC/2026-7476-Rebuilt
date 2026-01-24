package frc.robot.util.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Provides a dashboard selector for choosing swerve drive profiles.
 * Use this to select profiles via Elastic or Shuffleboard instead of
 * hardcoding.
 */
public class ProfileSelector {
  private final SendableChooser<SwerveDriveProfile> profileChooser;

  /**
   * Creates a new ProfileSelector with all available profiles.
   */
  public ProfileSelector() {
    profileChooser = new SendableChooser<>();

    // Add all available profiles
    profileChooser.setDefaultOption("CompBot", SwerveProfiles.COMP_BOT);
    profileChooser.addOption("SpongeBot", SwerveProfiles.SPONGE_BOT);
    profileChooser.addOption("OffSeasonSwerve", SwerveProfiles.OFF_SEASON_SWERVE);

    // Publish to dashboard
    SmartDashboard.putData("Swerve Profile", profileChooser);
  }

  /**
   * Gets the currently selected profile from the dashboard.
   * 
   * @return The selected SwerveDriveProfile
   */
  public SwerveDriveProfile getSelected() {
    return profileChooser.getSelected();
  }

  /**
   * Gets the selected profile, or returns a default if none is selected.
   * 
   * @param defaultProfile The profile to return if none is selected
   * @return The selected profile or the default
   */
  public SwerveDriveProfile getSelectedOrDefault(SwerveDriveProfile defaultProfile) {
    SwerveDriveProfile selected = profileChooser.getSelected();
    return selected != null ? selected : defaultProfile;
  }
}