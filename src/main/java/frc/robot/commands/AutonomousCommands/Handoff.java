// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Handoff extends Command {
  /** Creates a new Handofff. */
  private final IntakeSubsystem mIntake;
  private final LauncherSubsystem mLauncher;
  public Handoff(IntakeSubsystem intake, LauncherSubsystem launcher) {
    mIntake = intake;
    mLauncher = launcher;
    addRequirements(mIntake,mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_ACTIVE);
    mLauncher.runIndexer(LauncherConstants.INDEXER_VELOCITY_INDEXING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLauncher.stopIndexer();
    mIntake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mLauncher.pieceInIndexer() && !IntakeSubsystem.pieceInIntake();
  }
}
