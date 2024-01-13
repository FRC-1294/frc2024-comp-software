// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends Command {

  Intake _intake;
  boolean alreadyRunning = false;
  /** Creates a new DefaultIntakeCommand. */
  public DefaultIntakeCommand(Intake intake) {
    _intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _intake.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alreadyRunning && _intake.gamePieceInIntake()) {
      _intake.stopMotor();
      alreadyRunning = false;
    }
    if (!alreadyRunning && Input.getleftBumperXbox() && !_intake.gamePieceInIntake()) {
      _intake.runMotorRaw(-1.0);
      alreadyRunning = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
