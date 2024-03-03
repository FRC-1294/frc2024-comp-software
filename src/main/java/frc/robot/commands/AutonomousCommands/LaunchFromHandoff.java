// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AimState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchFromHandoff extends Command {
  /** Creates a new LaunchFromHandoff. */
  private final AimingSubsystem mWrist;
  private final LauncherSubsystem mLauncher;
  private final AimState mDesiredState; 
  private final Command mCommand;

  public LaunchFromHandoff(AimingSubsystem wrist, LauncherSubsystem launcher, AimState desiredState) {
    mWrist = wrist;
    mLauncher = launcher;
    mDesiredState = desiredState;
    mCommand = new SequentialCommandGroup(
      mWrist.waitUntilSetpoint(mDesiredState),
      mLauncher.waitUntilFlywheelSetpointCommand(mDesiredState),
      mLauncher.indexUntilNoteLaunchedCommand());
    addRequirements(mWrist,mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    mCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("LaunchFromHandoffFinished", mCommand.isFinished());
    return mCommand.isFinished();
  }
}
