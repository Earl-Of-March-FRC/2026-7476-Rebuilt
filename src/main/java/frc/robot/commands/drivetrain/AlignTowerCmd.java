package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignTowerCmd extends Command {

  private DrivetrainSubsystem driveSub;

  private final PIDController translationController = new PIDController(AutoConstants.kPTranslationController, AutoConstants.kITranslationController, AutoConstants.kDTranslationController);
  private final PIDController rotationController = new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController);

  public AlignTowerCmd(DrivetrainSubsystem driveSub, int xTarget, int yTarget) {
    this.driveSub = driveSub;

    // Declare subsystem dependencies (takes control over driveSub)
    addRequirements(driveSub);

    rotationController.enableContinuousInput(-180, 180);
    rotationController.setTolerance(2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = driveSub.getPose();
    Pose2d targetPose = driveSub.getBargeTargetPose(0);

    double currentAngle = driveSub.getPose().getRotation().getDegrees();
    double targetAngle = 180.0;

    AngularVelocity omega = DriveConstants.kMaxAngularSpeed.times(angleSupplier.getAsDouble());
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));

    /*
    Pose2d currentPose = driveSub.getPose();

    

    targetX = targetPose.getX();

    // Calculate target rotation based on side of field that robot is currently on
    targetRadians = targetPose.getRotation().getRadians();

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        targetPose);

    double xVel = MathUtil.clamp(translationController.calculate(currentPose.getX(), targetX),
        -AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxSpeedMetersPerSecond);

    // Invert the direction if robot is on red alliance
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        xVel *= -1;
      }
    }

    double currentRotation = currentPose.getRotation().getRadians();

    double rotVel = MathUtil.clamp(
        rotationController.calculate(currentRotation, targetRadians * Math.signum(currentRotation)),
        -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    // Set drivetrain to run at calculated velocity
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, 0, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputChassisSpeeds", chassisSpeeds);
    */
  }

  @Override
  public void initialize() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
