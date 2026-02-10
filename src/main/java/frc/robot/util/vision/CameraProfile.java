package frc.robot.util.vision;

import java.io.File;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.SimulationConstants;

/**
 * Record class representing a camera configuration profile.
 * Contains all the necessary parameters to configure a camera.
 *
 * @param name                       The camera name/identifier
 * @param roll                       Rotation around x-axis in radians
 * @param pitch                      Rotation around y-axis in radians
 * @param yaw                        Rotation around z-axis in radians
 * @param x                          Translation in x-direction (meters, +x is
 *                                   forward)
 * @param y                          Translation in y-direction (meters, +y is
 *                                   left)
 * @param z                          Translation in z-direction (meters, +z is
 *                                   up)
 * @param standardDeviation          Standard deviations for vision measurements
 *                                   [x, y,
 *                                   theta]
 * @param configFile                 [OPTIONAL] PhotonVision config file for
 *                                   simulation purposes (config.json)
 * @param fps                        [OPTIONAL] Frames per second for simulation
 *                                   purposes
 * @param avgLatency                 [OPTIONAL] Average latency for simulation
 *                                   purposes
 * @param avgLatencyDeviation        [OPTIONAL] Max deviation from average
 *                                   latency for
 *                                   simulation
 *                                   purposes
 * @param fieldOfView                [OPTIONAL] Diagonal field of view for
 *                                   simulation
 *                                   purposes
 * @param resolution                 [OPTIONAL] Video resolution for simulation
 *                                   purposes
 * @param camIntrinsics              [OPTIONAL] Camera instrinsics for
 *                                   simulation
 * @param distCoeffs                 [OPTIONAL] Distortion coeffients for
 *                                   simulation
 * @param simulationWireframeEnabled [OPTIONAL] Simulate wireframe, usually
 *                                   false due
 *                                   to being extremely resource-heavy
 */
public record CameraProfile(
    String name,
    Angle roll,
    Angle pitch,
    Angle yaw,
    Distance x,
    Distance y,
    Distance z,
    Vector<N3> standardDeviation,
    File configFile,
    double fps,
    Time avgLatency,
    Time avgLatencyDeviation,
    Angle fieldOfView,
    int[] resolution,
    Matrix<N3, N3> camIntrinsics,
    Matrix<N8, N1> distCoeffs,
    boolean simulationWireframeEnabled) {

  /**
   * {@inheritDoc}
   */
  public CameraProfile(String name,
      Angle roll,
      Angle pitch,
      Angle yaw,
      Distance x,
      Distance y,
      Distance z,
      Vector<N3> standardDeviation) {
    this(name, roll, pitch, yaw, x, y, z, standardDeviation, null);
  }

  /**
   * {@inheritDoc}
   */
  public CameraProfile(String name,
      Angle roll,
      Angle pitch,
      Angle yaw,
      Distance x,
      Distance y,
      Distance z,
      Vector<N3> standardDeviation, boolean simulationWireframeEnabled) {
    this(name, roll, pitch, yaw, x, y, z, standardDeviation, null, simulationWireframeEnabled);
  }

  /**
   * {@inheritDoc}
   */
  public CameraProfile(String name,
      Angle roll,
      Angle pitch,
      Angle yaw,
      Distance x,
      Distance y,
      Distance z,
      Vector<N3> standardDeviation,
      File configFile) {
    this(name, roll, pitch, yaw, x, y, z, standardDeviation, configFile, false);
  }

  /**
   * {@inheritDoc}
   */
  public CameraProfile(String name,
      Angle roll,
      Angle pitch,
      Angle yaw,
      Distance x,
      Distance y,
      Distance z,
      Vector<N3> standardDeviation,
      File configFile, boolean simulationWireframeEnabled) {
    this(name, roll, pitch, yaw, x, y, z, standardDeviation, configFile, SimulationConstants.kSimVisionFPS,
        SimulationConstants.kSimVisionLatency, SimulationConstants.kSimVisionLatencyDeviation,
        SimulationConstants.kSimVisionFOV, SimulationConstants.kSimVisionResolution,
        SimulationConstants.kSimVisionIntrinsics, SimulationConstants.kSimVisionDistCoeffs, simulationWireframeEnabled);
  }

  /**
   * Creates a Transform3d representing the robot-to-camera transformation.
   *
   * @return Transform3d from robot to camera
   */
  public Transform3d getRobotToCameraTransform() {
    return new Transform3d(
        new Translation3d(x(), y(), z()),
        new Rotation3d(roll(), pitch(), yaw()));
  }
}