// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  private final RobotContainer m_robotContainer;

  private double matchTime, phaseTime;
  private boolean isHubActiveFirst = true;
  private boolean currentHubState = true;
  private boolean nextHubState = true;
  private final int[] hubSwitchTimestamps = { 130, 105, 80, 55 };
  private int currentPeriod;

  private char allianceColour;

  /*
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // Start logger
    Logger.recordMetadata("ProjectName", "2026-7576-Rebuilt");
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      Logger.registerURCL(URCL.startExternal());
    }
    Logger.start();

    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("Auto?", isAutonomous());

    SmartDashboard.putNumber("Match Time", matchTime);

    SmartDashboard.putNumber("Phase Time", phaseTime);

    SmartDashboard.putData("Commands coming soon: ", CommandScheduler.getInstance());

    SmartDashboard.putBoolean("Starting Hub State", isHubActiveFirst);

    SmartDashboard.putBoolean("Hub State", currentHubState);

    SmartDashboard.putBoolean("Next Hub State", nextHubState);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    currentHubState = true;
    SmartDashboard.putString("Current Phase", "Auto");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    matchTime = DriverStation.getMatchTime() + 140;

    phaseTime = DriverStation.getMatchTime();
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Current Phase", "Transition");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    matchTime = DriverStation.getMatchTime();

    if (matchTime >= 135) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
          allianceColour = 'R';
        } else {
          allianceColour = 'B';
        }
      }

      String gameData = DriverStation.getGameSpecificMessage();

      if (gameData.length() > 0) {
        if (gameData.charAt(0) == allianceColour) {
          isHubActiveFirst = false;
        } else {
          isHubActiveFirst = true;
        }

        nextHubState = isHubActiveFirst;
      }
    }

    for (int i = 0; i < hubSwitchTimestamps.length; i++) {
      if ((int) matchTime == hubSwitchTimestamps[i]) {
        currentPeriod = i + 1;
        SmartDashboard.putString("Current Phase", "Phase " + currentPeriod + "/4");
        if (i % 2 == 0) {
          currentHubState = isHubActiveFirst;
        } else {
          currentHubState = !isHubActiveFirst;
        }

        nextHubState = switch (i) {
          case 0, 1, 2 -> !currentHubState;
          default -> true;
        };

        break;
      }
    }

    if ((int) matchTime == 30) {
      currentHubState = true;
      SmartDashboard.putString("Current Phase", "End Game");

    }

    if (DriverStation.getMatchTime() >= 130) {
      phaseTime = DriverStation.getMatchTime() - 130;
    } else if (DriverStation.getMatchTime() >= 105) {
      phaseTime = DriverStation.getMatchTime() - 105;
    } else if (DriverStation.getMatchTime() >= 80) {
      phaseTime = DriverStation.getMatchTime() - 80;
    } else if (DriverStation.getMatchTime() >= 55) {
      phaseTime = DriverStation.getMatchTime() - 55;
    } else if (DriverStation.getMatchTime() >= 30) {
      phaseTime = DriverStation.getMatchTime() - 30;
    } else {
      phaseTime = DriverStation.getMatchTime();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
