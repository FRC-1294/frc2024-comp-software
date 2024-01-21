// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntakeCommand extends Command {

  private IntakeSubsystem _intake;
  /** Creates a new DefaultIntakeCommand. */
  public AutonomousIntakeCommand(IntakeSubsystem intake) {
    _intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!_intake.pieceInIntake()) {
      _intake.intakeAtSpeed(100);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_intake.pieceInIntake()) {
      _intake.stopMotor();
      return true;
    }
    return false;
  }
}
