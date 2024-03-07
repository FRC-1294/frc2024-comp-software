// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.states.mech_states.Intaken;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.states.mech_states.ReadyForHandoff;
import frc.robot.states.mech_states.ReadyForIntake;
import frc.robot.states.mech_states.ReadyForLaunch;
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
  public static Command mCommand = new PrintCommand("Uhh");

  public ScoreSpeaker(LauncherSubsystem launcher, AimingSubsystem wrist, IntakeSubsystem intake, AimState prioSpeakerState) {
    mLauncher = launcher;
    mWrist = wrist;
    mIntake = intake;
    mPrioSpeakerState = prioSpeakerState;
    //addRequirements(mLauncher,mWrist);
  }

  public ScoreSpeaker(SwerveSubsystem swerveSubsystem,LauncherSubsystem launcher, AimingSubsystem wrist, IntakeSubsystem intake) {
    mLauncher = launcher;
    mWrist = wrist;
    mIntake = intake;
    mPrioSpeakerState = getBestSpeakerState();
    //addRequirements(mLauncher,mWrist);
  }

  public AimState getBestSpeakerState(){
    //TBD
    return AimState.SUBWOOFER;
  }

  @Override
  public void initialize(){
    
  }

  @Override
  public void execute() {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // SmartDashboard.putBoolean("ScoreSpeakerIsFinished", mCommand.isFinished());
    return mCommand.isFinished();
    //TBD We also need to check if swerve is at the right position and angle tolerance
  }
}
