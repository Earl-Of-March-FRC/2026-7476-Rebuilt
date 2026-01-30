package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

public class AlignTowerCmd extends Command {

  private DrivetrainSubsystem driveSub;
  private DoubleSupplier xTransSupplier;
  private DoubleSupplier yTransSupplier;
  private DoubleSupplier angleSupplier;

  private final PIDController turnPID = new PIDController(0.05, 0, 0);

  public AlignTowerCmd(DrivetrainSubsystem driveSub, DoubleSupplier xTransSupplier, DoubleSupplier yTransSupplier,
      PIDController turnPID) {
    this.driveSub = driveSub;
    this.xTransSupplier = xTransSupplier;
    this.yTransSupplier = yTransSupplier;

    // Declare subsystem dependencies (takes control over driveSub)
    addRequirements(driveSub);

    turnPID.enableContinuousInput(-180, 180);
    turnPID.setTolerance(2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity xVel = DriveConstants.kMaxSpeed.times(xTransSupplier.getAsDouble());
    LinearVelocity yVel = DriveConstants.kMaxSpeed.times(yTransSupplier.getAsDouble());

    double currentAngle = driveSub.getPose().getRotation().getDegrees();
    double targetAngle = 180.0;

    AngularVelocity omega = DriveConstants.kMaxAngularSpeed.times(angleSupplier.getAsDouble());
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));
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
