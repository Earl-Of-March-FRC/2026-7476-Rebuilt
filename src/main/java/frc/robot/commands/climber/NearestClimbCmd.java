// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.ClimbSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NearestClimbCmd extends SequentialCommandGroup {
  private final DrivetrainSubsystem drivetrain;
  private final ClimberSubsystem climber;

  /** Creates a new NearestClimbCmd. */
  public NearestClimbCmd(DrivetrainSubsystem drivetrain, ClimberSubsystem climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.drivetrain = drivetrain;
    this.climber = climber;

    addRequirements(drivetrain, climber);

    SmartDashboard.putNumber("Commands/NearestClimbCmd/RobotPoseY", drivetrain.getPose().getY());
    SmartDashboard.putNumber("Commands/NearestClimbCmd/FieldHalfWidth",
        FieldConstants.kFieldWidthY.div(2).in(Meters));

    addCommands(drivetrain.createDriveToZoneCommand(),
        Commands.defer(this::createAlignAndClimbCommand, getRequirements()));
  }

  /**
   * Creates a command that aligns the robot to the tower, and moves it to the
   * nearest side
   * 
   * @return SequentialCommandGroup with drivetrain and climber commands.
   * @apiNote Should be wrapped in {@code Commands.defer()} for dynamic updating
   */
  private SequentialCommandGroup createAlignAndClimbCommand() {
    PathPlannerPath climberPath;
    ClimbSide climbSide;
    ParallelCommandGroup alignAndRaiseClimber = new ParallelCommandGroup();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;
    if (drivetrain.getPose().getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      climbSide = ClimbSide.Right;
      climberPath = isBlueAlliance ? AutoConstants.outpostClimbPath : AutoConstants.depotClimbPath;

    } else {
      climbSide = ClimbSide.Left;
      climberPath = isBlueAlliance ? AutoConstants.depotClimbPath : AutoConstants.outpostClimbPath;
    }

    alignAndRaiseClimber.addCommands(new RaiseClimberCmd(climber, ClimberConstants.kRaisePosition),
        AutoBuilder.pathfindThenFollowPath(climberPath, AutoConstants.L1ClimbConstraints));

    PullClimberCmd pullClimber = new PullClimberCmd(climber, () -> ClimberConstants.kMotorHookSpeed, climbSide);

    return new SequentialCommandGroup(alignAndRaiseClimber, pullClimber);
  }
}
