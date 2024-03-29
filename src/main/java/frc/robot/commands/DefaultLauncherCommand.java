// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.LauncherSubsystem;

public class DefaultLauncherCommand extends Command {

  private final LauncherSubsystem mLauncher;

  public DefaultLauncherCommand(LauncherSubsystem launcher) {
    mLauncher = launcher;
    addRequirements(mLauncher);
  }

  @Override
  public void initialize() {
    mLauncher.stopLauncher(); 
  }

  @Override
  public void execute() {
    if (Input.getDPad()==Input.DPADRIGHT) {
      mLauncher.stopLauncher();
    }
    //speaker
    if (Input.getY()) {
      mLauncher.setLauncherMode(LauncherMode.SPEAKER);
    } 
    //amp
    else if (Input.getA()) {
      mLauncher.setLauncherMode(LauncherMode.AMP);
    }
    else if (Input.getB()) {
      mLauncher.setLauncherMode(LauncherMode.TRAP);
    }

    if (Input.getLeftTrigger()>0.1 && !mLauncher.pieceInIndexer()) {
      mLauncher.runIndexer(LauncherConstants.INDEXER_VELOCITY_INDEXING);
    } else if (Input.getRightBumper() && mLauncher.isLauncherReady()) {
      mLauncher.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCHING);
    } else {
      mLauncher.runIndexer(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
