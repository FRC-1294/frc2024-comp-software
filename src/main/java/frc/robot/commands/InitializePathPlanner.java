// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class InitializePathPlanner extends Command {
  /** Creates a new InitializePathPlanner. */
  private final SwerveSubsystem mSwerve;
  private final Command mFinalCmd;
  public InitializePathPlanner(SwerveSubsystem swerve) {
    mSwerve = swerve;
    mFinalCmd = new InstantCommand(()->AutoBuilder.configureHolonomic(
    mSwerve::getRobotPose, 
    (Pose2d pose)->mSwerve.resetRobotPose(pose),
    mSwerve::getChassisSpeeds, 
    (ChassisSpeeds speeds)->mSwerve.setChassisSpeed(speeds),
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                                      // your Constants class
          new PIDConstants(5, 0.0, 0), // Translation PID constants
          new PIDConstants(5, 0.0, 0), // Rotation PID constants
          3, // Max module speed, in m/s
          0.4669, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig(false, true) // Default path replanning config. See the API for the options
                              // here
                              
    ), ()->false, mSwerve),mSwerve);
    addRequirements(mSwerve);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFinalCmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mFinalCmd.isFinished();
  }
}
