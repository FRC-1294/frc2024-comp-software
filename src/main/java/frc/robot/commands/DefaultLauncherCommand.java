// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Launcher;

public class DefaultLauncherCommand extends Command {

  private final Launcher mLauncher;

  public DefaultLauncherCommand(Launcher launcher) {
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
      SequentialCommandGroup command = new SequentialCommandGroup(

      new InstantCommand(
          mLauncher::stopLauncher, mLauncher
      ));
      command.schedule();
    }
    //speaker
    else if (Input.getA()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      SequentialCommandGroup command = new SequentialCommandGroup(

      new InstantCommand(
          () -> mLauncher.setLauncherMode(LauncherMode.SPEAKER), mLauncher)
      );
      command.schedule();
    } 
    //amp
    else if (Input.getB()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      SequentialCommandGroup command = new SequentialCommandGroup(

      new InstantCommand(
          () -> mLauncher.setLauncherMode(LauncherMode.AMP), mLauncher)
      );
      command.schedule();
    }
    else if (Input.getX()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      SequentialCommandGroup command = new SequentialCommandGroup(

      new InstantCommand(
          () -> mLauncher.setLauncherMode(LauncherMode.TRAP), mLauncher)
      );
      command.schedule();
    }
    else if (Input.getLeftBumper()) { //TBD https://github.com/FRC-1294/frc2024/issues/241
      SequentialCommandGroup command = new SequentialCommandGroup(

      new InstantCommand(
          () -> mLauncher.turnIndexerOn(!mLauncher.isIndexerOn()), mLauncher) //reverse current state
      );
      command.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
