package frc.robot.util.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Provides a dashboard selector for choosing swerve drive profiles.
 * Use this to select profiles via Elastic or Shuffleboard instead of
 * hardcoding.
 */
public final class ProfileSelector {

  private static final String PREF_KEY = "SelectedProfile";

  private static SendableChooser<String> profileChooser;
  private static String lastSavedSelection = "";

  private ProfileSelector() {
    // Prevent instantiation
  }

  /**
   * Must be called ONCE (robotInit).
   */
  public static void init() {
    if (profileChooser != null) {
      return;
    }

    profileChooser = new SendableChooser<>();

    profileChooser.setDefaultOption("CompBot", "CompBot");
    profileChooser.addOption("SpongeBot", "SpongeBot");
    profileChooser.addOption("Off Season Swerve", "OffSeasonSwerve");

    SmartDashboard.putData("Swerve Profile", profileChooser);

    lastSavedSelection = Preferences.getString(PREF_KEY, "CompBot");

    Logger.recordOutput("ProfileSelector/Initialized", true);
  }

  /**
   * Call from robotPeriodic().
   * Required for Elastic.
   */
  public static void updatePreferences() {
    if (profileChooser == null) {
      return;
    }

    String selected = profileChooser.getSelected();
    if (selected == null) {
      return;
    }

    if (!selected.equals(lastSavedSelection)) {
      Preferences.setString(PREF_KEY, selected);
      lastSavedSelection = selected;

      Logger.recordOutput("ProfileSelector/SavedNewSelection", selected);
    }
  }

  /**
   * Returns the selected profile, falling back to defaultProfile.
   */
  public static SwerveDriveProfile getSelectedOrDefault(
      SwerveDriveProfile defaultProfile) {

    String selection = Preferences.getString(
        PREF_KEY,
        defaultProfile.getName());

    Logger.recordOutput("ProfileSelector/FinalProfileName", selection);

    SwerveDriveProfile profile = getProfileByName(selection);

    Logger.recordOutput(
        "ProfileSelector/FinalProfileId",
        profile.profileId());

    return profile;
  }

  /**
   * Converts a profile name into its corresponding SwerveDriveProfile.
   */
  private static SwerveDriveProfile getProfileByName(String name) {
    if (name == null) {
      return SwerveProfiles.COMP_BOT;
    }

    switch (name) {
      case "SpongeBot":
        return SwerveProfiles.SPONGE_BOT;

      case "OffSeasonSwerve":
        return SwerveProfiles.OFF_SEASON_SWERVE;

      case "CompBot":
      default:
        return SwerveProfiles.COMP_BOT;
    }
  }
}