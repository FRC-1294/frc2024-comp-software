// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;

public class DefaultAimCommand extends Command {

  private final AimingSubsystem mAimingSubsystem;

  //TODO: Implement Input Using an Xbox Controller(You can choose the button mapping)
  // Use the Arm from last year as inspiration
  public DefaultAimCommand(AimingSubsystem aimingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mAimingSubsystem = aimingSubsystem;
    addRequirements(aimingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AimState state = AimState.STOW;
    if (Input.getA()) {
      state = AimState.AMP;
    }
    else if (Input.getB()) {
      state = AimState.SPEAKER;
    }
    else if (Input.getX()) {
      state = AimState.CLIMB;
    }
    mAimingSubsystem.setDesiredSetpoint(state);

    if (Math.abs(Input.getLeftStickY()) > 0) {
      //convert between input to motor speed
      double speed = Input.getLeftStickY() / JoystickConstants.MAX_XBOX_JOYSTICK_L_MAGNITUDE * AimingConstants.MAX_WRIST_ROTATION_VELOCITY;
      mAimingSubsystem.changeDesiredWristRotation(speed);
    }
    
    if (Math.abs(Input.getRightStickY()) > 0) {
      //convert between input to motor speed
      double speed = Input.getRightStickY() / JoystickConstants.MAX_XBOX_JOYSTICK_R_MAGNITUDE * AimingConstants.MAX_ELEVATOR_EXTENSION_VELOCITY;
      mAimingSubsystem.changeDesiredElevatorPosition((speed));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mAimingSubsystem.atSetpoints();
  }
}
