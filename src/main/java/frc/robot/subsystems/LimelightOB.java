// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  public Command getAutomousIntakeCommand() {
    return new FunctionalCommand(() -> m_intake.intakeAtSpeed(IntakeConstants.INTAKE_SPEED), null, interrupted -> m_intake.stopMotor(), this::functionalCommandIsFinished, this);    
  }

  public 

  private boolean functionalCommandIsFinished() {
    return m_intake.pieceInIntake() || Input.getLeftBumper();
  }
}
