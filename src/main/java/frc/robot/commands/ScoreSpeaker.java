// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SpeakerState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreSpeaker extends Command {
  /** Creates a new ScoreSpeaker. */
  private final SwerveSubsystem mSwerve;
  private final LauncherSubsystem mLauncher;
  private final AimingSubsystem mWrist;
  private final SpeakerState mPrioSpeakerState;

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist, SpeakerState prioSpeakerState) {
    mSwerve = swerveSubsystem;
    mLauncher = launcher;
    mWrist = wrist;
    mPrioSpeakerState = prioSpeakerState;
    addRequirements(mSwerve,mLauncher,mWrist);
  }

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist) {
    mSwerve = swerveSubsystem;
    mLauncher = launcher;
    mWrist = wrist;
    mPrioSpeakerState = getBestSpeakerState();
    addRequirements(mSwerve,mLauncher,mWrist);
  }

  public SpeakerState getBestSpeakerState(){
    //TBD
    return SpeakerState.SUBWOOFER;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TBD Move robot to the prio speaker state's radius and angle
    mWrist.setDesiredWristRotation(mPrioSpeakerState.mWristAngleDeg);
    mLauncher.setLauncherMode(LauncherMode.SPEAKER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mWrist.atWristSetpoint() && mLauncher.isLauncherReady();
    //TBD We also need to check if swerve is at the right position and angle tolerance
  }
}
