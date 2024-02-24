// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.CompConstants;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.states.mech_states.Intaken;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.states.mech_states.ReadyForHandoff;
import frc.robot.states.mech_states.ReadyForIntake;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreSpeaker extends Command {
  /** Creates a new ScoreSpeaker. */
  private final LauncherSubsystem mLauncher;
  private final AimingSubsystem mWrist;
  private final IntakeSubsystem mIntake;
  private final AimState mPrioSpeakerState;
  private MechState curState;
  private Command mCommand;

  public ScoreSpeaker(LauncherSubsystem launcher, AimingSubsystem wrist, IntakeSubsystem intake, AimState prioSpeakerState) {
    mLauncher = launcher;
    mWrist = wrist;
    mIntake = intake;
    mPrioSpeakerState = prioSpeakerState;
    mCommand = new SequentialCommandGroup(
      mWrist.waitUntilSetpoint(prioSpeakerState),
      mLauncher.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER),
      mLauncher.waitUntilNoteLaunchedCommand());
    addRequirements(mLauncher,mWrist);
  }

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist, IntakeSubsystem intake) {
    mLauncher = launcher;
    mWrist = wrist;
    mIntake = intake;
    mPrioSpeakerState = getBestSpeakerState();
    mCommand = new SequentialCommandGroup(mWrist.waitUntilSetpoint(mPrioSpeakerState),
    mLauncher.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER),mLauncher.waitUntilNoteLaunchedCommand());
    addRequirements(mLauncher,mWrist);
  }

  public AimState getBestSpeakerState(){
    //TBD
    return AimState.SUBWOOFER;
  }

  @Override
  public void initialize(){
    curState = DefaultMechCommand.determineState();
    if (curState.getClass() == ReadyForIntake.class){
      mCommand = new PrintCommand("No Note L");
    } else if (curState.getClass() == Intaken.class){
      mCommand = mWrist.waitUntilSetpoint(AimState.HANDOFF)
      .andThen(new Handoff(mIntake, mLauncher))
      .andThen(new LaunchFromHandoff(mWrist, mLauncher, mPrioSpeakerState)).withTimeout(CompConstants.AUTO_LAUNCH_TIMEOUT_SEC);
    } else if (curState.getClass() == ReadyForHandoff.class){
      mCommand =new Handoff(mIntake, mLauncher)
      .andThen(new LaunchFromHandoff(mWrist, mLauncher, mPrioSpeakerState)).withTimeout(CompConstants.AUTO_LAUNCH_TIMEOUT_SEC);
    } else if (curState.getClass() == ReadyForAim.class){
      mCommand = new LaunchFromHandoff(mWrist, mLauncher, mPrioSpeakerState).withTimeout(CompConstants.AUTO_LAUNCH_TIMEOUT_SEC);
    } else{
      mCommand = new PrintCommand("How tf u get here??");
    }
    mCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mCommand.isFinished();
    //TBD We also need to check if swerve is at the right position and angle tolerance
  }
}
