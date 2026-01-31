// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.climber.RaiseClimberCmd;
// import frc.robot.commands.drivetrain.AlignTowerCmd;
// import frc.robot.subsystems.Climber.ClimberSubsystem;
// import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.DriveConstants;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;

// public class ClimbSequenceCmd extends SequentialCommandGroup {

// private final ClimberSubsystem climber;

// private DrivetrainSubsystem driveSub;

// public ClimbSequenceCmd(ClimberSubsystem climber, DrivetrainSubsystem
// driveSub) {

// this.climber = climber;
// this.driveSub = driveSub;

// // addCommands(new RaiseClimberCmd(climber,
// // ClimberConstants.kClimberMotorSpeed), new AlignTowerCmd(driveSub, ));

// }
// }
