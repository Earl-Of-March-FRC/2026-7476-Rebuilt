// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.ClimbToHeightCmd;
import frc.robot.commands.drivetrain.DriveStopCmd;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem.TowerSide;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.PathGenerator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Drives to the corresponding tower side and climbs
 */
public class DriveAndClimbCmd extends SequentialCommandGroup {
  /**
   * Create a command that drives to the corresponding tower side, and climbs.
   * This command ensures that the climber is at the bottom before it fully
   * commits to climbing
   * 
   * @param drivetrain Drivetrain subsystem
   * @param climber    Climber subsystem
   * @param towerSide  Tower side to climb
   */
  public DriveAndClimbCmd(DrivetrainSubsystem drivetrain, ClimberSubsystem climber, TowerSide towerSide) {
    final Supplier<FieldZones> currentBotZone = () -> {
      Logger.recordOutput("Commands/DriveAndClimbCmd/MeasuredBotZone", drivetrain.getCurrentBotZone());
      return drivetrain.getCurrentBotZone();
    };

    final BooleanSupplier isOutsideAllianceZone = () -> currentBotZone.get() != FieldZones.Launch
        && currentBotZone.get() != FieldZones.Alliance;

    // Move into the alliance zone before trying to raise climbers
    final Command moveIntoZoneAllianceZoneCmd = PathGenerator.driveToTowerFrontAuto(towerSide)
        .onlyWhile(isOutsideAllianceZone);

    // Already in alliance zone, can start moving up climbers
    final Command moveToTowerFrontCmd = new ParallelDeadlineGroup(
        new ReturnClimbersToBottomCmd(climber)
            .andThen(new ClimbToHeightCmd(climber, ClimberConstants.kLatchPosition)),
        PathGenerator.driveToTowerFrontAuto(towerSide));

    addCommands(
        moveIntoZoneAllianceZoneCmd,
        moveToTowerFrontCmd,
        new DriveToTowerSideCmd(drivetrain, towerSide),
        new DriveStopCmd(drivetrain),
        new ClimbToHeightCmd(climber, ClimberConstants.kRaisePosition));
  }
}
