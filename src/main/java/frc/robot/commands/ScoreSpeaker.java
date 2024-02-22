// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SpeakerState;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreSpeaker extends Command {
  /** Creates a new ScoreSpeaker. */
  private final SwerveSubsystem mSwerve;
  private final LauncherSubsystem mLauncher;
  private final AimingSubsystem mWrist;
  private final SpeakerState mPrioSpeakerState;
  private boolean readyToAim;
  private final Command mCommand;

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist, SpeakerState prioSpeakerState) {
    mSwerve = swerveSubsystem;
    mLauncher = launcher;
    mWrist = wrist;
    mPrioSpeakerState = prioSpeakerState;
    mCommand = new SequentialCommandGroup(mWrist.waitUntilWristSetpoint(mPrioSpeakerState.mWristAngleDeg),
    mLauncher.waitUntilFlywheelSetpointCommand(),mLauncher.waitUntilNoteExitIntakeCommand());
    addRequirements(mSwerve,mLauncher,mWrist);
  }

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist) {
    mSwerve = swerveSubsystem;
    mLauncher = launcher;
    mWrist = wrist;
    mPrioSpeakerState = getBestSpeakerState();
    mCommand = new SequentialCommandGroup(mWrist.waitUntilWristSetpoint(mPrioSpeakerState.mWristAngleDeg),
    mLauncher.waitUntilFlywheelSetpointCommand(),mLauncher.waitUntilNoteExitIntakeCommand());
    addRequirements(mSwerve,mLauncher,mWrist);
  }

  public SpeakerState getBestSpeakerState(){
    //TBD
    return SpeakerState.SUBWOOFER;
  }

  @Override
  public void initialize(){
    readyToAim = DefaultMechCommand.determineState().getClass() == ReadyForAim.class;
    if (readyToAim){
      mCommand.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !readyToAim || mCommand.isFinished();
    //TBD We also need to check if swerve is at the right position and angle tolerance
  }
}
