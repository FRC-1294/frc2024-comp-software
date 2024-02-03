// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

public class InitializePathPlanner{
  /** Creates a new InitializePathPlanner. */
  private final SwerveSubsystem mSwerve;
  public InitializePathPlanner(SwerveSubsystem swerve) {
    mSwerve = swerve;
  }
  public void initializeNamedCOmmands(){
    NamedCommands.registerCommand("IntakeUntilNote", new SequentialCommandGroup(new PrintCommand("Intaking until note enters")));
    NamedCommands.registerCommand("Handoff", new SequentialCommandGroup(new PrintCommand("Handoff"), new WaitCommand(1)));
    NamedCommands.registerCommand("ShootDynamic", new SequentialCommandGroup(new PrintCommand("Shoot Note"), new WaitCommand(0.5)));
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    AutoBuilder.configureHolonomic(
    PoseEstimation::getRobotPose, 
    (Pose2d pose)->PoseEstimation.resetPose(pose),
    SwerveSubsystem::getChassisSpeeds, 
    (ChassisSpeeds speeds)->mSwerve.setChassisSpeed(speeds),
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                                      // your Constants class
          new PIDConstants(5, 0.0, 0), // Translation PID constants
          new PIDConstants(5, 0.0, 0), // Rotation PID constants
          3, // Max module speed, in m/s
          0.4669, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig(false, true) // Default path replanning config. See the API for the options
                              // here
                              
    ), ()->false, mSwerve);
    initializeNamedCOmmands();
  }
}
