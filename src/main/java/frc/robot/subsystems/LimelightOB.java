// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightOB extends SubsystemBase {
  NetworkTable mTable;
  String mLimeLightName;
  double tv;
  double tx;
  double ta;
  public LimelightOB() {
    mLimeLightName = "limelight";
    mTable = NetworkTableInstance.getDefault().getTable(mLimeLightName);
  }
  public LimelightOB(String LimeLightName) {
    mLimeLightName = LimeLightName;
    mTable = NetworkTableInstance.getDefault().getTable(mLimeLightName);
  }

  @Override
  public void periodic() {
    tv = mTable.getEntry("tv").getDouble(0);
    ta = mTable.getEntry("ta").getDouble(0);
    tx = mTable.getEntry("tx").getDouble(0);
  }

  public boolean isDetectionValid() {
    return (tv == 1.0 && ta >= 2.0);
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
