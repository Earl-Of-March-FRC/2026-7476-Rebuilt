package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * Automatically aligns the robot to the correct climbing position.
 */
public class ClimbAlignCmd extends Command {

  // State
  private final DrivetrainSubsystem driveSub;
  private Pose2d targetPose = null;

  // Controllers
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController headingController;

  /**
   * @param driveSub The drivetrain subsystem.
   */
  public ClimbAlignCmd(DrivetrainSubsystem driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);

    xController = new PIDController(
        ClimbAlignConstants.kAlignP,
        ClimbAlignConstants.kAlignI,
        ClimbAlignConstants.kAlignD);
    xController.setTolerance(ClimbAlignConstants.kTranslationTolerance.in(Meters));

    yController = new PIDController(
        ClimbAlignConstants.kAlignP,
        ClimbAlignConstants.kAlignI,
        ClimbAlignConstants.kAlignD);
    yController.setTolerance(ClimbAlignConstants.kTranslationTolerance.in(Meters));

    headingController = new PIDController(
        DriveConstants.kPIDHeadingControllerP,
        DriveConstants.kPIDHeadingControllerI,
        DriveConstants.kPIDHeadingControllerD);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(ClimbAlignConstants.kRotationTolerance.in(Radians));
  }

  @Override
  public void initialize() {
    targetPose = computeTargetPose();

    xController.reset();
    yController.reset();
    headingController.reset();

    if (targetPose != null) {
      xController.setSetpoint(targetPose.getX());
      yController.setSetpoint(targetPose.getY());
      headingController.setSetpoint(targetPose.getRotation().getRadians());

      Logger.recordOutput("Commands/ClimbAlign/TargetPose", targetPose);
      Logger.recordOutput("Commands/ClimbAlign/Status", "Aligning");
    } else {
      Logger.recordOutput("Commands/ClimbAlign/Status", "No target found");
    }
  }

  @Override
  public void execute() {
    if (targetPose == null) {
      driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
      return;
    }

    Pose2d current = driveSub.getPose();
    double maxSpeedMps = ClimbAlignConstants.kAlignMaxSpeed.in(MetersPerSecond);

    // Raw PID outputs (clamped to [-1, 1] then scaled to max speed)
    double xOut = clamp(xController.calculate(current.getX()), -1.0, 1.0) * maxSpeedMps;
    double yOut = clamp(yController.calculate(current.getY()), -1.0, 1.0) * maxSpeedMps;
    double omegaOut = headingController.calculate(current.getRotation().getRadians());

    // Additional speed cap on the translational vector
    Translation2d vel = new Translation2d(xOut, yOut);
    if (vel.getNorm() > maxSpeedMps) {
      vel = vel.div(vel.getNorm()).times(maxSpeedMps);
    }

    // Field-relative speeds (no manual-control alliance inversion needed here)
    driveSub.runVelocity(
        new ChassisSpeeds(vel.getX(), vel.getY(), omegaOut),
        true, // field-relative
        false, // not manual X
        false); // not manual Y

    Logger.recordOutput("Commands/ClimbAlign/CurrentPose", current);
    Logger.recordOutput("Commands/ClimbAlign/TranslationError/X", targetPose.getX() - current.getX());
    Logger.recordOutput("Commands/ClimbAlign/TranslationError/Y", targetPose.getY() - current.getY());
    Logger.recordOutput("Commands/ClimbAlign/HeadingError",
        Math.toDegrees(targetPose.getRotation().getRadians() - current.getRotation().getRadians()));
    Logger.recordOutput("Commands/ClimbAlign/AtSetpoint", isAtSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0)); // Stop the robot
    Logger.recordOutput("Commands/ClimbAlign/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return targetPose == null || isAtSetpoint();
  }

  /**
   * Builds the full two-phase climb align command:
   * <ol>
   * <li>PathPlanner pathfind to a coarse waypoint near the climb zone.</li>
   * <li>Fine PID alignment with this command.</li>
   * </ol>
   *
   * <p>
   * The pathfind target is offset from the final climb pose so PathPlanner
   * does not need sub-centimetre accuracy, this cmd handles the last
   * metre or so of travel.
   *
   * @param driveSub The drivetrain subsystem.
   * @return A command that performs the full sequence.
   */
  public static Command fullAlignCommand(DrivetrainSubsystem driveSub) {
    // We defer the target-pose calculation so it runs at schedule time, not at
    // bind time, ensuring we always use the current alliance.
    return Commands.defer(() -> {
      Pose2d target = computeTargetPoseStatic(driveSub.getPose());
      if (target == null) {
        // No tag data; skip pathfinding, attempt fine align anyway
        return new ClimbAlignCmd(driveSub);
      }

      // Coarse waypoint: same Y, same heading, but further back by kCoarseOffset
      Pose2d coarseWaypoint = new Pose2d(
          target.getX() + ClimbAlignConstants.kCoarseApproachOffset.in(Meters)
              * -target.getRotation().getCos(),
          target.getY() + ClimbAlignConstants.kCoarseApproachOffset.in(Meters)
              * -target.getRotation().getSin(),
          target.getRotation());

      return new SequentialCommandGroup(
          AutoBuilder.pathfindToPose(
              coarseWaypoint,
              AutoConstants.L1ClimbConstraints,
              ClimbAlignConstants.kCoarseApproachEndVelocity),
          new ClimbAlignCmd(driveSub));

    }, Set.of(driveSub));
  }

  /** Instance wrapper so initialize() can call the static logic. */
  private Pose2d computeTargetPose() {
    return computeTargetPoseStatic(driveSub.getPose());
  }

  /**
   * Derives the robot's desired climb pose from the AprilTag field layout.
   *
   * <p>
   * The two climb tags mark the LEFT and RIGHT vertical bars of the tower.
   * The horizontal rung extends past each vertical bar, so we want to climb on
   * the outer portion of whichever bar the robot is closest to (L climber design
   * :skull).
   *
   * <ol>
   * <li>Look up both tag poses from the field layout (always available).</li>
   * <li>Determine which tag (left or right) the robot is currently closer
   * to.</li>
   * <li>Use that tag's XY position as the lateral reference.</li>
   * <li>Shift the reference outward (away from the other tag) by
   * {@code kOuterRungOffset} to land on the outer rung section.</li>
   * <li>Offset away from the wall by {@code kStandoffDistance} along the
   * tag's facing normal so the climber hook lines up with the rung.</li>
   * <li>Robot heading = tag yaw (face into the wall/tower).</li>
   * </ol>
   * 
   */
  static Pose2d computeTargetPoseStatic() {
    return computeTargetPoseStatic(null);
  }

  /**
   * Same as {@link #computeTargetPoseStatic()} but accepts an explicit robot
   * pose so the factory method can compute the target at defer-time without
   * needing a live drivetrain reference.
   *
   * @param robotPose Current robot pose, or {@code null} to read from
   *                  DriveStation (not available in static context — callers
   *                  that have the drivetrain should pass it in).
   */
  static Pose2d computeTargetPoseStatic(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlue = !alliance.isPresent() || alliance.get() == Alliance.Blue;

    int leftTagId = isBlue ? ClimbAlignConstants.kBlueLeftTagId : ClimbAlignConstants.kRedLeftTagId;
    int rightTagId = isBlue ? ClimbAlignConstants.kBlueRightTagId : ClimbAlignConstants.kRedRightTagId;

    Optional<Pose3d> leftTagOpt = FieldConstants.kfieldLayout.getTagPose(leftTagId);
    Optional<Pose3d> rightTagOpt = FieldConstants.kfieldLayout.getTagPose(rightTagId);

    if (leftTagOpt.isEmpty() || rightTagOpt.isEmpty()) {
      Logger.recordOutput("ClimbAlign/Status", "Tag poses not found in layout!");
      return null;
    }

    Pose3d leftTag = leftTagOpt.get();
    Pose3d rightTag = rightTagOpt.get();

    // 1. Pick nearest tag
    boolean useLeft;
    if (robotPose != null) {
      Translation2d pos = robotPose.getTranslation();
      double distLeft = pos.getDistance(new Translation2d(leftTag.getX(), leftTag.getY()));
      double distRight = pos.getDistance(new Translation2d(rightTag.getX(), rightTag.getY()));
      useLeft = distLeft <= distRight;
    } else {
      useLeft = true;
      Logger.recordOutput("ClimbAlign/Warning", "No robot pose provided, defaulting to left tag");
    }

    Pose3d nearTag = useLeft ? leftTag : rightTag;
    Pose3d farTag = useLeft ? rightTag : leftTag;

    Logger.recordOutput("ClimbAlign/NearestSide", useLeft ? "Left" : "Right");
    Logger.recordOutput("ClimbAlign/NearTagId", useLeft ? leftTagId : rightTagId);

    // 2. Compute outward unit vector (away from the other tag)
    double lateralDx = nearTag.getX() - farTag.getX();
    double lateralDy = nearTag.getY() - farTag.getY();
    double lateralNorm = Math.hypot(lateralDx, lateralDy);
    if (lateralNorm < 1e-6) {
      Logger.recordOutput("ClimbAlign/Status", "Tags are at the same position - aborting");
      return null;
    }
    double outwardUnitX = lateralDx / lateralNorm;
    double outwardUnitY = lateralDy / lateralNorm;

    // 3. Shift outward from the tag by kOuterRungOffset
    double rungOffset = ClimbAlignConstants.kOuterRungOffset.in(Meters);
    double refX = nearTag.getX() + rungOffset * outwardUnitX;
    double refY = nearTag.getY() + rungOffset * outwardUnitY;

    // 4. Standoff: move forward from the wall along the tag's facing direction
    double tagYaw = nearTag.getRotation().getZ();
    double standoff = ClimbAlignConstants.kStandoffDistance.in(Meters);
    double targetX = refX + standoff * Math.cos(tagYaw);
    double targetY = refY + standoff * Math.sin(tagYaw);

    Pose2d result = new Pose2d(targetX, targetY, new Rotation2d(tagYaw));
    Logger.recordOutput("ClimbAlign/TargetPose", result);
    return result;
  }

  private boolean isAtSetpoint() {
    return xController.atSetpoint()
        && yController.atSetpoint()
        && headingController.atSetpoint();
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}