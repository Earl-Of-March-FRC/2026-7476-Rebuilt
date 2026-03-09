package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitHelpers {
  /**
   * Clamps a measurement between two values
   * 
   * @param <U>            Unit
   * @param a              Measurement to clamp
   * @param minMeasurement Minimum
   * @param maxMeasurement Maximum
   * @return Clamped measurement
   * @apiNote Output may need to be casted to the desired unit
   */
  public static <U extends Unit> Measure<U> clamp(Measure<U> a, Measure<U> minMeasurement, Measure<U> maxMeasurement) {
    return Measure.max(minMeasurement, Measure.min(maxMeasurement, a));
  }
}
