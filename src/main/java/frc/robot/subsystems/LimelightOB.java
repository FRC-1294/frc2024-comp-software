// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem; 
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LimelightOB extends SubsystemBase {
  NetworkTable mTable;
  IntakeSubsystem m_intake; 
  public LimelightOB() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAutomousIntakeCommand() {
    return new FunctionalCommand(() -> m_intake.intakeAtSpeed(IntakeConstants.INTAKE_SPEED), null, interrupted -> m_intake.stopMotor(), this::functionalCommandIsFinished, this);    
  }

  public 

  private boolean functionalCommandIsFinished() {
    return m_intake.pieceInIntake() || Input.getLeftBumper();
  }
}
