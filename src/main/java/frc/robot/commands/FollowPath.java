// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class FollowPath extends Command {
  /** Creates a new FollowPath. */
  private final SwerveSubsystem mSwerve;
  private final Command mFinalCmd;
  private final String pathPlannerAutoPath;
  PathPlannerPath p;
  public FollowPath(SwerveSubsystem swerveSubsystem,String s) {
    mSwerve = swerveSubsystem;
    pathPlannerAutoPath = s;
    p = PathPlannerPath.fromPathFile(s);

    mFinalCmd = AutoBuilder.buildAuto(s);
    addRequirements(mSwerve);
  }

  // Called when the command is iniqtially scheduled.
  @Override
  public void initialize() {
    mSwerve.resetRobotPose(PathPlannerAuto.getPathGroupFromAutoFile(pathPlannerAutoPath).get(0).getPreviewStartingHolonomicPose());
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

