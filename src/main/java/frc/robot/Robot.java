// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.fasterxml.jackson.databind.ser.std.MapProperty;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LauncherAndIntakeConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.ProfileSelector;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private boolean gyroCalibrated = false;
  private final RobotContainer m_robotContainer;
  private final GameModel gameModel = new GameModel();

  private Command autonomousCommand;
  private boolean simulateFuel = false;

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
    // Only write log files if real & USB stick is plugged in
    // if (isReal() && new File("/u/").exists()) {
    Logger.addDataReceiver(new WPILOGWriter("/u/logs"));
    // } else {
    // System.err
    // .println("No USB flashdrive was found in the RoboRIO's directory.
    // WPILOGWriter and URCL not initiated.");
    // }
    Logger.start();
    Logger.registerURCL(URCL.startExternal());

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
    ProfileSelector.updatePreferences();
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("Auto?", isAutonomous());
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
    SmartDashboard.putBoolean("is neutral zone", m_robotContainer.driveSub.getCurrentBotZone() == FieldZones.Neutral);

    SmartDashboard.putString("Auto Pose",
        m_robotContainer.driveSub.getPose().getX() + " " + m_robotContainer.driveSub.getPose().getY());
    Logger.recordOutput("Launcher/LaunchAngle/", LauncherAndIntakeConstants.kBallReleaseAngle().in(Degrees));
  }

  @Override
  public void robotInit() {
    // Use AdvantageKit compatible pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    // calibrate while disabled and stationary -- this call blocks (~5s)
    m_robotContainer.getGyro().calibrate();
    ProfileSelector.init();
    LauncherAndIntakeConstants.initReleaseAngleChooser();
    gyroCalibrated = true;

    // Port forward Photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);
    // Photonvision cameras
    PortForwarder.add(1181, "photonvision.local", 1181);
    PortForwarder.add(1182, "photonvision.local", 1182);
    PortForwarder.add(1183, "photonvision.local", 1183);
    PortForwarder.add(1184, "photonvision.local", 1184);
    PortForwarder.add(1185, "photonvision.local", 1185);
    PortForwarder.add(1186, "photonvision.local", 1186);

    SmartDashboard.putBoolean("is Neutral Zone", false);

    SmartDashboard.putNumber("Delayed Crossing Time (Auto)", AutoConstants.kDefaultAutoDelay.in(Seconds));
    SmartDashboard.putNumber("8 Fuel Launch Time (Auto)", AutoConstants.kAutoLaunch8Time.in(Seconds));
    SmartDashboard.putNumber("32 Fuel Launch Time (Auto)", AutoConstants.kAutoLaunch32Time.in(Seconds));

    PathPlannerLogging.clearLoggingCallbacks();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (!gyroCalibrated) {
      m_robotContainer.getGyro().calibrate();
      gyroCalibrated = true;
    }
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
    gameModel.initAuto();
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    // Logger.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    gameModel.updateAuto();
  }

  @Override
  public void teleopInit() {
    gameModel.initTeleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    gameModel.updateTeleop();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.configureTestMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SmartDashboard.putNumber("ClimberModelTest", SmartDashboard.getNumber("ClimberModelTest", 0));
    simulateFuel = SmartDashboard.getBoolean("FieldSimulation/SimulateFuel", false);
    if (simulateFuel) {
      SimulatedArena.getInstance().resetFieldForAuto();
    } else {
      SmartDashboard.putBoolean("FieldSimulation/SimulateFuel", false);
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    if (simulateFuel) {
      Logger.recordOutput("FieldSimulation/Fuel",
          SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    // Test the height of the climbers
    Logger.recordOutput("FieldSimulation/Extra/ClimberModelTest",
        new Pose3d(0, 0, SmartDashboard.getNumber("ClimberModelTest", 0), Rotation3d.kZero));
  }

  /**
   * Get the GameModel instance for use by commands.
   */
  public GameModel getGameModel() {
    return gameModel;
  }
}
