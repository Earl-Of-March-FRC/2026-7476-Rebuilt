
package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignTowerCmd extends Command {

  private final DrivetrainSubsystem driveSub;

  private final PIDController translationController = new PIDController(
      AutoConstants.kPTranslationController,
      AutoConstants.kITranslationController,
      AutoConstants.kDTranslationController);

  private final PIDController rotationController = new PIDController(
      AutoConstants.kPThetaController,
      AutoConstants.kIThetaController,
      AutoConstants.kDThetaController);

  // private final int towerSide;

  public AlignTowerCmd(DrivetrainSubsystem driveSub, boolean leftSide, boolean rightSide) {
    this.driveSub = driveSub;
    addRequirements(driveSub);

    // Continuous input in radians — handles the -π/+π wraparound correctly
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(AutoConstants.kAlignRotationTolerance.in(Radians));
    translationController.setTolerance(AutoConstants.kAlignTranslationTolerance.in(Meters));

    // towerSide = 2 + (leftSide ? -1 : 0) + (rightSide ? 1 : 0); // leftSide = 1,
    // middle = 2, rightSide = 3
  }

  @Override
  public void initialize() {
    Logger.recordOutput("AlignTower/Status", "Initialized");
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    Pose2d targetPose = driveSub.getHubTargetPose(0); // Replace to a

    // Calculate raw PID outputs
    double xOutput = translationController.calculate(currentPose.getX(), targetPose.getX());
    double yOutput = translationController.calculate(currentPose.getY(), targetPose.getY());
    double rotOutput = rotationController.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    // Clamp values to max constants
    double maxLinear = AutoConstants.kMaxSpeedMetersPerSecond;
    double maxAngular = AutoConstants.kMaxAngularSpeed.in(RadiansPerSecond);

    LinearVelocity xVel = MetersPerSecond.of(MathUtil.clamp(xOutput, -maxLinear, maxLinear));
    LinearVelocity yVel = MetersPerSecond.of(MathUtil.clamp(yOutput, -maxLinear, maxLinear));
    AngularVelocity omega = RadiansPerSecond.of(MathUtil.clamp(rotOutput, -maxAngular, maxAngular));

    // Convert Field-Relative PID results to Robot-Relative for the motors
    driveSub.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, omega, currentPose.getRotation()));

    Logger.recordOutput("AlignTower/CurrentPose", currentPose);
    Logger.recordOutput("AlignTower/TargetPose", targetPose);
    Logger.recordOutput("AlignTower/Output/XVelMetersPerSec", xVel.in(MetersPerSecond));
    Logger.recordOutput("AlignTower/Output/YVelMetersPerSec", yVel.in(MetersPerSecond));
    Logger.recordOutput("AlignTower/Output/OmegaRadPerSec", omega.in(RadiansPerSecond));
    Logger.recordOutput("AlignTower/AtTranslationGoal", translationController.atSetpoint());
    Logger.recordOutput("AlignTower/AtRotationGoal", rotationController.atSetpoint());

    /*
     *
     *
     *
     * targetX = targetPose.getX();
     *
     * // Calculate target rotation based on side of field that robot is currently
     * on
     * targetRadians = targetPose.getRotation().getRadians();
     *
     * Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
     * currentPose);
     * Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
     * targetPose);
     *
     * double xVel =
     * MathUtil.clamp(translationController.calculate(currentPose.getX(),
     * targetX),
     * -AutoConstants.kMaxSpeedMetersPerSecond,
     * AutoConstants.kMaxSpeedMetersPerSecond);
     *
     * // Invert the direction if robot is on red alliance
     * if (DriverStation.getAlliance().isPresent()) {
     * Alliance alliance = DriverStation.getAlliance().get();
     * if (alliance == Alliance.Red) {
     * xVel *= -1;
     * }
     * }
     *
     * double currentRotation = currentPose.getRotation().getRadians();
     *
     * double rotVel = MathUtil.clamp(
     * rotationController.calculate(currentRotation, targetRadians *
     * Math.signum(currentRotation)),
     * -AutoConstants.kMaxAngularSpeedRadiansPerSecond,
     * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
     *
     * // Set drivetrain to run at calculated velocity
     * ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, 0, rotVel);
     * driveSub.runVelocityFieldRelative(chassisSpeeds);
     *
     * Logger.recordOutput(
     * "Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityX", xVel);
     * Logger.recordOutput(
     * "Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityRotation",
     * rotVel);
     * Logger.recordOutput(
     * "Odometry/MoveToNearestBargeLaunchingZone/PID/OutputChassisSpeeds",
     * chassisSpeeds);
     */
  }

  @Override
  public void end(boolean interrupted) {
    driveSub.runVelocity(new ChassisSpeeds(0, 0, 0));
    Logger.recordOutput("AlignTower/Status", interrupted ? "Interrupted" : "Completed");
  }

  // Ends when both translation and rotation are within tolerance
  @Override
  public boolean isFinished() {
    return translationController.atSetpoint() && rotationController.atSetpoint();
  }
}
