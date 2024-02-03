// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.Input;
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
    if (Input.getY()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      mLauncher.stopLauncher();
    }
    //speaker
    else if (Input.getA()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      mLauncher.setLauncherMode(LauncherMode.SPEAKER);
    } 
    //amp
    else if (Input.getB()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      mLauncher.setLauncherMode(LauncherMode.AMP);
    }
    else if (Input.getX()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      mLauncher.setLauncherMode(LauncherMode.TRAP);
    }
    else if (Input.getLeftBumper()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      mLauncher.turnIndexerOn(!mLauncher.isIndexerOn());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
