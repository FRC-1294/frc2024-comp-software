// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
<<<<<<< HEAD
import frc.robot.robots.CompetitionBotSwerveConfig;
import frc.robot.robots.PracticeBotSwerveConfig;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
=======
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
>>>>>>> main

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
<<<<<<< HEAD
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem(new CompetitionBotSwerveConfig());
  private final PoseEstimation mEstimation = new PoseEstimation();
=======
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem();

>>>>>>> main
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.setDefaultCommand(new DefaultDriveCommand(mSwerveSubsystem));
<<<<<<< HEAD
  }
  
  public SwerveSubsystem getSwerveSubsystem() {
      return mSwerveSubsystem;
  }

=======

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
>>>>>>> main
}
