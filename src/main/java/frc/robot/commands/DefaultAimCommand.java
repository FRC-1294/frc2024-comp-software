// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.Input;
import frc.robot.constants.AimingConstants;
import frc.robot.subsystems.AimingSubsystem;

public class DefaultAimCommand extends Command {
  private final AimingSubsystem mAimingSubsystem;

  // Use the Arm from last year as inspiration
  public DefaultAimCommand(AimingSubsystem aimingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mAimingSubsystem = aimingSubsystem;
    addRequirements(aimingSubsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.getA()) {
      mAimingSubsystem.setDesiredSetpoint(AimState.AMP);
    }
    if (Input.getY()) {
      mAimingSubsystem.setDesiredSetpoint(AimState.SPEAKER);
    }
    else if (Input.getX()) {
      mAimingSubsystem.setDesiredSetpoint(AimState.HANDOFF);
    }
    else if (Input.getDPad() == 0.0) {
      mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB);
    }


    if (Math.abs(Input.getLeftStickY()) > 0) {
      //convert between input to increment
      double increment = Input.getLeftStickY() * AimingConstants.MAX_WRIST_TELEOP_INCREMENT;
      mAimingSubsystem.changeDesiredWristRotation(increment);
    }
    
    if (Math.abs(Input.getRightStickY()) > 0) {
      //convert between input to increment
      double increment = Input.getRightStickY() * AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT;
      mAimingSubsystem.changeDesiredElevatorPosition(increment);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mAimingSubsystem.atSetpoints();
  }
}
