// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.robots.PracticeBotSwerveConfig;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem(new PracticeBotSwerveConfig());
  // private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  // private final AimingSubsystem mAimingSubsystem = new AimingSubsystem();
  // private final LauncherSubsystem mLauncherSubsystem = new LauncherSubsystem();
  // private final DefaultMechCommand mDefaultMechCommand = new DefaultMechCommand(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);
  private final Limelight mLightLight = new Limelight();
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.setDefaultCommand(new DefaultDriveCommand(mSwerveSubsystem, mLightLight));
    // mLauncherSubsystem.setDefaultCommand(mDefaultMechCommand);
    // mAimingSubsystem.setDefaultCommand(mDefaultMechCommand);
    // mIntakeSubsystem.setDefaultCommand(mDefaultMechCommand);
  }
  
  public SwerveSubsystem getSwerveSubsystem() {
      return mSwerveSubsystem;
  }
  
  // public IntakeSubsystem getIntakeSubsystem(){
  //   return mIntakeSubsystem;
  // }
  
  // public AimingSubsystem getAimingSubsystem(){
  //   return mAimingSubsystem;
  // }

  // public LauncherSubsystem getLauncher(){
  //   return mLauncherSubsystem;
  // }
}
