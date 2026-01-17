// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Telemetry {

  private final NetworkTable matchTable;
  private final NetworkTableEntry matchTimerEntry;

  public Telemetry() {
    matchTable = NetworkTableInstance.getDefault().getTable("Match");
    matchTimerEntry = matchTable.getEntry("Match Timer");
  }

  public void updateMatchTime(double matchTime) {
    matchTimerEntry.setDouble(DriverStation.getMatchTime());
  }
}
