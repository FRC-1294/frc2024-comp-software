// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants.AimState;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.Input;

public class DefaultElevatorCommand extends Command {
  private final Elevator mElevator;

  // Use the Arm from last year as inspiration
  public DefaultElevatorCommand(Elevator elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Input.getA()) {
      mElevator.setDesiredElevatorDistance(0.5);
    }
    else if (Input.getX()) {
      mElevator.setDesiredElevatorDistance(0.1);;
    }


    // if (Math.abs(Input.getJoystickX()) > 0) {
    //   //convert between input to increment
    //   double increment = Input.getJoystickX() * ElevatorConstants.MAX_ELEVATOR_TELEOP_INCREMENT;
    //   // mElevator.changeDesiredWristRotation(increment);
    // }
    
    // if (Math.abs(Input.getJoystickY()) > 0) {
    //   //convert between input to increment
    //   double increment = Input.getJoystickY() * ElevatorConstants.MAX_WRIST_TELEOP_INCREMENT;
    //   mElevator.changeDesiredElevatorPosition(increment);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mElevator.atSetpoints();
  }
}