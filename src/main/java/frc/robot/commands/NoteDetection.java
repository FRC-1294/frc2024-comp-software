// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.LimeLightLime;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem; 
import frc.robot.subsystems.SwerveSubsystem; 
public class NoteDetection extends Command {
  /** Creates a new NoteDetection. */
  LimeLightLime m_limelight; 
  IntakeSubsystem m_intake; 
  SwerveSubsystem m_swerve; 
  public NoteDetection(LimeLightLime limelight, IntakeSubsystem intake, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight; 
    m_intake = intake; 
    m_swerve = swerve; 
    addRequirements(limelight);
    addRequirements(intake);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override 
  public Command getAutomousIntakeCommand() {
    return new FunctionalCommand(() -> intakeAtSpeed(IntakeConstants.INTAKE_SPEED), null, interrupted -> m_intake.stopMotor(), this::functionalCommandIsFinished, this);    
  }
}

// if tv = 1 then do execution