package frc.robot.util.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Record class representing a camera configuration profile.
 * Contains all the necessary parameters to configure a camera.
 *
 * @param name              The camera name/identifier
 * @param roll              Rotation around x-axis in radians
 * @param pitch             Rotation around y-axis in radians
 * @param yaw               Rotation around z-axis in radians
 * @param x                 Translation in x-direction (meters, +x is forward)
 * @param y                 Translation in y-direction (meters, +y is left)
 * @param z                 Translation in z-direction (meters, +z is up)
 * @param standardDeviation Standard deviations for vision measurements [x, y,
 *                          theta]
 */
public record CameraProfile(
    String name,
    Angle roll,
    Angle pitch,
    Angle yaw,
    Distance x,
    Distance y,
    Distance z,
    Vector<N3> standardDeviation) {

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