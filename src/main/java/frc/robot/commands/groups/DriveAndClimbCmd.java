// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.RaiseClimberCmd;
import frc.robot.commands.drivetrain.AlignTowerCmd;
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
 * 
 * @deprecated This command is currently put on hold.
 */
public class DriveAndClimbCmd extends SequentialCommandGroup {
  /** Creates a new DriveAndClimbCmd. */
  public DriveAndClimbCmd(DrivetrainSubsystem drivetrain, ClimberSubsystem climber, TowerSide towerSide) {

    final BooleanSupplier climberAtSetpoint = () -> climber.isAtPosition(towerSide.getCorrespondingClimberSide(false));

    final Supplier<FieldZones> currentBotZone = () -> {
      Logger.recordOutput("Commands/DriveAndClimbCmd/MeasuredBotZone", drivetrain.getCurrentBotZone());
      return drivetrain.getCurrentBotZone();
    };

    final BooleanSupplier canRaiseClimber = () -> currentBotZone.get() == FieldZones.Launch
        || currentBotZone.get() == FieldZones.Alliance;

    // Move into the alliance zone before trying to raise climbers
    final Command moveIntoZoneAllianceZoneCmd = PathGenerator.driveToTowerFrontAuto(towerSide)
        .onlyWhile(() -> !canRaiseClimber.getAsBoolean());

    // Already in alliance zone, can start moving up climbers
    final Command moveToTowerFrontCmd = new ParallelDeadlineGroup(
        // new RaiseClimberCmd(climber, ClimberConstants.kRaisePosition)
        // .until(climberAtSetpoint),
        PathGenerator.driveToTowerFrontAuto(towerSide));

    final Command moveToTowerSideCmd = new ParallelDeadlineGroup(
        PathGenerator.driveToTowerSideAuto(towerSide)
    // new RaiseClimberCmd(climber, ClimberConstants.kRaisePosition)
    );

    addCommands(
        moveIntoZoneAllianceZoneCmd,
        moveToTowerFrontCmd,
        moveToTowerSideCmd,
        // new RaiseClimberCmd(climber, ClimberConstants.kRaisePosition)
        // .until(climberAtSetpoint),
        new DriveStopCmd(drivetrain)
    // new RaiseClimberCmd(climber, 0)
    );
  }
}
