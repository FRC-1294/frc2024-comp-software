// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;


public class Limelight extends SubsystemBase {
  NetworkTable mTable;
  double tv;
  double tx;
  double ta;
  public Limelight() {
    mTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);
  }
  public Limelight(String limeLightName) {
    mTable = NetworkTableInstance.getDefault().getTable(limeLightName);
  }

  @Override
  public void periodic() {
    tv = mTable.getEntry("tv").getDouble(0);
    ta = mTable.getEntry("ta").getDouble(0);
    tx = mTable.getEntry("tx").getDouble(0);
  }

  public boolean isDetectionValid() {
    return tv == 1.0;
  }

  public double getTX() {
    return tx;
  }

  public double getTA() {
    return ta;
  }

  public double getTV() {
    return tv;
  }
}