// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightOB;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem; 
import frc.robot.subsystems.Input;

public class DefaultNoteDetectionCommand extends Command {
  /** Creates a new NoteDetection. */
  LimelightOB m_limelight; 
  //IntakeSubsystem m_intake; 
  SwerveSubsystem m_swerve; 
  boolean intakeDone;
  boolean rotDone;
  double xMovement;
  double yMovement;
  double rot = 0.25; 
  public DefaultNoteDetectionCommand(LimelightOB limelight, SwerveSubsystem swerve /* , IntakeSubsystem intake */) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight; 
    m_swerve = swerve; 
    addRequirements(limelight);
    addRequirements(swerve); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.getNoteAlignment()){
      if (m_limelight.isDetectionValid()) {
        xMovement = Math.sin(Math.toRadians(m_limelight.getTX())); 
        yMovement = Math.cos(Math.toRadians(m_limelight.getTX()));
        if (m_limelight.getTX() > 2) {
          m_swerve.setChassisSpeed(xMovement, yMovement, rot); 
        }
        else if (m_limelight.getTX() < 2) {
          m_swerve.setChassisSpeed(xMovement, yMovement, rot);
        }
        rotDone = false;
        //intakeDone = false; 
      }
      //m_intake.intakeAtSpeed(IntakeConstants.INTAKE_SPEED);
    }
      /* if (m_intake.pieceInIntake()) {
      m_intake.stopMotor();
      m_swerve.setChassisSpeed(0, 0, 0);
      intakeDone = true;
    }
    */
     if (m_limelight.isDetectionValid() && Math.abs(m_limelight.getTX()) >= 2) {
      rotDone = true;
      m_swerve.setChassisSpeed(xMovement, yMovement, rot);
    }
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true whten the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}

// if tx < Math.abs(1) then do execution