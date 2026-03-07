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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drivetrain.ClimbAlignCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NearestClimbCmd extends SequentialCommandGroup {

  /** Creates a new NearestClimbCmd. */
  public NearestClimbCmd(DrivetrainSubsystem drivetrain, ClimberSubsystem climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    ParallelCommandGroup alignAndRaiseClimber = new ParallelCommandGroup();
    PullClimberCmd pullClimber;

    if (drivetrain.getCurrentBotZone() == FieldZones.Neutral) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      boolean isBlueAlliance = !alliance.isPresent() || alliance.get() == Alliance.Blue;

      int neutralZoneWaypointId;
      if (isBlueAlliance) {
        neutralZoneWaypointId = 4;
      } else {
        neutralZoneWaypointId = 6;
      }
      addCommands(PathGenerator.crossTrenchAuto(
          Arrays.copyOfRange(FieldConstants.kTrenchPathWaypoints, neutralZoneWaypointId, neutralZoneWaypointId + 2)));
    }

    if (drivetrain.getPose().getY() <= FieldConstants.kFieldWidthY.div(2).in(Meters)) {
      SmartDashboard.putNumber("pose", drivetrain.getPose().getY());
      SmartDashboard.putNumber("field width", FieldConstants.kFieldWidthY.div(2).in(Meters));
      alignAndRaiseClimber.addCommands(new RaiseClimberCmd(climber, false),
          AutoBuilder.pathfindThenFollowPath(AutoConstants.outpostClimbPath, AutoConstants.L1ClimbConstraints));
      pullClimber = new PullClimberCmd(climber, false);
    } else {
      alignAndRaiseClimber.addCommands(new RaiseClimberCmd(climber, true),
          AutoBuilder.pathfindThenFollowPath(AutoConstants.depotClimbPath, AutoConstants.L1ClimbConstraints));
      pullClimber = new PullClimberCmd(climber, true);
    }

    addCommands(alignAndRaiseClimber);
    addCommands(pullClimber);

    // addRequirements(drivetrain, climber);
  }
}
