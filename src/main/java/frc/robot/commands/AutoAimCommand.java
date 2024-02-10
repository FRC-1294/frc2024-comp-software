// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.AimingConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;

public class DefaultAimCommand extends Command {
  private final AimingSubsystem mAimingSubsystem;

  // Use the Arm from last year as inspiration
  public autoAim(AimingSubsystem aimingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mAimingSubsystem = aimingSubsystem;
    addRequirements(aimingSubsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.DPADRIGHT()) {
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mAimingSubsystem.atSetpoints();
  }
}
