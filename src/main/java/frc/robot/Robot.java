// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.net.URI;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;
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
import frc.robot.util.LocalADStarAK;
import frc.robot.util.swerve.FieldZones;
import frc.robot.util.swerve.ProfileSelector;
import frc.robot.util.swerve.SwerveConfig;
import frc.robot.util.vision.CameraProfile;

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

  }

  @Override
  public void robotInit() {
    // Use AdvantageKit compatible pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    // calibrate while disabled and stationary -- this call blocks (~5s)
    m_robotContainer.getGyro().calibrate();
    ProfileSelector.init();
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

    new Thread(() -> {
      try {
        // Poll until PhotonVision is up (not blindly waiting)
        // Tries every 100ms for a max of 10 seconds
        for (int i = 0; i < 100; i++) {
          try {
            // Attempt a test connection to see if PhotonVision is ready
            URI uri = new URI("ws://photonvision.local:5800/websocket_data");
            WebSocketClient test = new WebSocketClient(uri) {
              @Override
              public void onOpen(ServerHandshake h) {
                close();
              }

              @Override
              public void onMessage(String m) {
              }

              @Override
              public void onMessage(ByteBuffer b) {
              }

              @Override
              public void onClose(int c, String r, boolean remote) {
              }

              @Override
              public void onError(Exception e) {
              }
            };

            // If this succeeds without an exception, PhotonVision is up
            test.connectBlocking(100, TimeUnit.MILLISECONDS);
            test.close();
            break; // Exit the polling loop
          } catch (Exception e) {
            // PhotonVision not ready yet, wait 100ms and retry
            Thread.sleep(100);
          }
        }

        ObjectMapper msgpackMapper = new ObjectMapper(new MessagePackFactory());

        // Iterate over each camera and toggle auto exposure ON -> OFF -> ON
        // This forces the camera driver to reinitialize exposure correctly on boot
        for (CameraProfile cameraProfile : SwerveConfig.kCameraProfiles) {
          String camera = cameraProfile.name();

          // Build the ON payload
          Map<String, Object> inner = new HashMap<>();
          inner.put("cameraUniqueName", camera);
          inner.put("cameraAutoExposure", true);

          Map<String, Object> payload = new HashMap<>();
          payload.put("changePipelineSetting", inner);

          byte[] onBytes = msgpackMapper.writeValueAsBytes(payload);

          // Build the OFF payload (reuses same inner map)
          inner.put("cameraAutoExposure", false);
          byte[] offBytes = msgpackMapper.writeValueAsBytes(payload);

          // Send ON -> OFF -> ON to force exposure reinitialization
          sendMsgPackWS("photonvision.local", 5800, onBytes);
          Thread.sleep(300);
          sendMsgPackWS("photonvision.local", 5800, offBytes);
          Thread.sleep(300);
          sendMsgPackWS("photonvision.local", 5800, onBytes);
          Thread.sleep(300);
        }
      } catch (Exception e) {
        System.err.println("PhotonVision exposure fix failed: " + e.getMessage());
      }
    }).start();

    SmartDashboard.putBoolean("is Neutral Zone", false);

    PathPlannerLogging.clearLoggingCallbacks();
  }

  /**
   * Sends a MessagePack-encoded binary message to the PhotonVision WebSocket
   * server.
   * Opens a connection, sends the data, waits for confirmation, then closes.
   *
   * @param host the hostname of the PhotonVision server (e.g.
   *             "photonvision.local")
   * @param port the port of the PhotonVision server (e.g. 5800)
   * @param data the MessagePack-encoded binary payload to send
   * @throws Exception if the URI is malformed or the connection times out
   */
  private void sendMsgPackWS(String host, int port, byte[] data) throws Exception {
    URI uri = new URI("ws://" + host + ":" + port + "/websocket_data");
    CountDownLatch latch = new CountDownLatch(1);

    WebSocketClient client = new WebSocketClient(uri) {
      @Override
      public void onOpen(ServerHandshake handshake) {
        send(data);
        latch.countDown();
      }

      @Override
      public void onMessage(String message) {
      }

      @Override
      public void onMessage(ByteBuffer bytes) {
      }

      @Override
      public void onClose(int code, String reason, boolean remote) {
      }

      @Override
      public void onError(Exception ex) {
        System.err.println("WS error: " + ex.getMessage());
        latch.countDown();
      }
    };

    client.connect();
    latch.await(3, TimeUnit.SECONDS);
    Thread.sleep(200);
    client.close();
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

    Logger.start();
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
