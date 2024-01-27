// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.CompConstants;

public class DefaultIntakeCommand extends Command {

  private IntakeSubsystem mIntake;
  /** Creates a new DefaultIntakeCommand. */
  public DefaultIntakeCommand(IntakeSubsystem intake) {
    mIntake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.getOnePiece()) {
      CompConstants.ONE_PIECE = !CompConstants.ONE_PIECE;
    }
    if (mIntake.pieceInIntake()) {
      mIntake.stopMotor();
    }
    if (Input.getIntake() && (!mIntake.pieceInIntake() || !CompConstants.ONE_PIECE)) {
      mIntake.intakeAtSpeed(IntakeConstants.INTAKE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Not needed because it is a default command
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
