// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
=======
>>>>>>> d0ca9dfc2bd16df823429358b08b99853b491f93
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem; 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

  

  public Command getNoteAlignmentCommand(SwerveSubsystem swerve) {
    return new FunctionalCommand(() -> startNoteAlignment(swerve), null, interrupted -> swerve.setChassisSpeed(0.0, 0.0, 0.0), this::isRotationFinished, this, swerve);  

  }

  public void startNoteAlignment(SwerveSubsystem swerve) {
    if (isDetectionValid()) {
      if (getTX() > 2) {
        swerve.setChassisSpeed(0.0, 0.0, 0.5);
      }
      else if (getTX() < 2) {
        swerve.setChassisSpeed(0.0, 0.0, -0.5);
      }
    }
  }

  private boolean isRotationFinished() {
    
    return Math.abs(getTX())<=2.0 && isDetectionValid();
  }
}
