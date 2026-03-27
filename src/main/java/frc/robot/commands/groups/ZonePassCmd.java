package frc.robot.commands.groups;

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

/**
 * Zone-aware passing command.
 *
 * <p>
 * Behaviour:
 * <ul>
 * <li>Continuously updates the target based on the robot's current zone
 * (neutral -> nearest unobstructed bump pose, enemy -> dump shot).</li>
 * <li>Steers the drivetrain heading toward the chosen target.</li>
 * <li>Spins the flywheel to the calculated RPM for that target.</li>
 * <li>Feeds balls through the indexer when {@code feedSupplier} is true.</li>
 * </ul>
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
    // Re-evaluate target every cycle so it updates as the robot moves.
    LaunchSetpoints setpoints = ZonePassHelpers.calculateZonePassSetpoints(leadShots);
    Translation3d target = ZonePassHelpers.selectPassTarget();

    AngularVelocity flywheelSpeed = setpoints.flywheelSpeed();
    launcherSub.setReferenceVelocity(flywheelSpeed);

    // Heading correction toward the pass target
    AngularVelocity omega = driveSub.getHeadingCorrectionOmega(setpoints.botHeading());

    // Allow driver to move tangentially while the heading is corrected.
    var speeds = new ChassisSpeeds(
        xSupplier.get() * frc.robot.util.swerve.SwerveConfig.kMaxSpeed.in(
            edu.wpi.first.units.Units.MetersPerSecond),
        ySupplier.get() * frc.robot.util.swerve.SwerveConfig.kMaxSpeed.in(
            edu.wpi.first.units.Units.MetersPerSecond),
        omega.in(edu.wpi.first.units.Units.RadiansPerSecond));

    driveSub.runVelocity(speeds, true, true);

    // Feed balls only when the driver requests it
    if (feedSupplier.getAsBoolean()) {
      indexerSub.setWheelPercent(-IndexerConstants.kWheelLaunchIndexPercent);
      indexerSub.setTreadmillPercent(-IndexerConstants.kTreadmillLaunchIndexPercent);
    } else {
      indexerSub.setTreadmillPercent(-IndexerConstants.kTreadmillLaunchIndexPercent);
      indexerSub.setWheelPercent(0);
    }

    Logger.recordOutput("Commands/ZonePassCmd/PassTarget", target);
    Logger.recordOutput("Commands/ZonePassCmd/FlywheelRPM",
        flywheelSpeed.in(edu.wpi.first.units.Units.RPM));
    Logger.recordOutput("Commands/ZonePassCmd/DesiredHeadingDeg",
        setpoints.botHeading().getDegrees());
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