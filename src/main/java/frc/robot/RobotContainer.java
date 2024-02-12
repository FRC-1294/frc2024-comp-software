// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem(new CompetitionBotSwerveConfig());
  // private final PoseEstimation mEstimation = new PoseEstimation();
  private final Elevator mElevator = new Elevator();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // mIntake.setDefaultCommand(new DefaultIntakeCommand(mIntake));
    // mSwerveSubsystem.setDefaultCommand(new DefaultDriveCommand(mSwerveSubsystem));
    mElevator.setDefaultCommand(new DefaultElevatorCommand(mElevator));
  }
  
  // public SwerveSubsystem getSwerveSubsystem() {
  //     return mSwerveSubsystem;
  // }

}
