package frc.robot.commands.groups;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.launcherAndIntake.LauncherAndIntakeSubsystem;
import frc.robot.util.launcher.LaunchHelpers.LaunchSetpoints;
import frc.robot.util.launcher.ZonePassHelpers;
import frc.robot.util.swerve.SwerveConfig;

/**
 * Zone-aware passing command.
 *
 * <p>
 * Every cycle this command re-evaluates which pass target to use based on the
 * robot's zone (neutral vs enemy), then hands the resulting setpoints to the
 * launcher and heading controller. The driver can still translate freely while
 * the heading is corrected automatically.
 *
 * <p>
 * Balls are fed through the indexer only when {@code feedSupplier} is true, so
 * the driver controls the moment of release.
 */
public class ZonePassCmd extends Command {

  private final DrivetrainSubsystem driveSub;
  private final IndexerSubsystem indexerSub;
  private final LauncherAndIntakeSubsystem launcherSub;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final BooleanSupplier feedSupplier;
  private final boolean leadShots;

  /**
   * Creates a new ZonePassCmd.
   *
   * @param driveSub     Drivetrain subsystem
   * @param indexerSub   Indexer subsystem
   * @param launcherSub  Launcher subsystem
   * @param xSupplier    Driver X input [-1, 1] (field-relative)
   * @param ySupplier    Driver Y input [-1, 1] (field-relative)
   * @param feedSupplier Returns true when the driver requests the ball to be fed
   * @param leadShots    Whether to compensate for drivetrain velocity when
   *                     calculating heading
   */
  public ZonePassCmd(
      DrivetrainSubsystem driveSub,
      IndexerSubsystem indexerSub,
      LauncherAndIntakeSubsystem launcherSub,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      BooleanSupplier feedSupplier,
      boolean leadShots) {

    this.driveSub = driveSub;
    this.indexerSub = indexerSub;
    this.launcherSub = launcherSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.feedSupplier = feedSupplier;
    this.leadShots = leadShots;

    addRequirements(driveSub, indexerSub, launcherSub);
  }

  @Override
  public void initialize() {
    driveSub.resetControllers();
    Logger.recordOutput("Commands/ZonePassCmd/Status", "Running");
  }

  @Override
  public void execute() {
    // Re-evaluate target and setpoints every cycle so they update as the bot moves.
    LaunchSetpoints setpoints = ZonePassHelpers.calculatePassSetpoints(leadShots);

    launcherSub.setReferenceVelocity(setpoints.flywheelSpeed());

    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(setpoints.botHeading());

    ChassisSpeeds speeds = new ChassisSpeeds(
        xSupplier.get() * SwerveConfig.kMaxSpeed.in(
            MetersPerSecond),
        ySupplier.get() * SwerveConfig.kMaxSpeed.in(
            MetersPerSecond),
        omega.in(edu.wpi.first.units.Units.RadiansPerSecond));

    driveSub.runVelocity(speeds, true, true);

    boolean shotIsSafe = setpoints.isShotSafe();
    if (feedSupplier.getAsBoolean() && shotIsSafe) {
      indexerSub.setWheelPercent(-IndexerConstants.kWheelLaunchIndexPercent);
      indexerSub.setTreadmillPercent(-IndexerConstants.kTreadmillLaunchIndexPercent);
    } else {
      indexerSub.setTreadmillPercent(-IndexerConstants.kTreadmillLaunchIndexPercent);
      indexerSub.setWheelPercent(0);
    }

    Logger.recordOutput("Commands/ZonePassCmd/ShotIsSafe", shotIsSafe);
    Logger.recordOutput("Commands/ZonePassCmd/DesiredHeadingDeg", setpoints.botHeading().getDegrees());
    Logger.recordOutput("Commands/ZonePassCmd/Feeding", feedSupplier.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    launcherSub.stop();
    indexerSub.stop();
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
    Logger.recordOutput("Commands/ZonePassCmd/Status",
        interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}