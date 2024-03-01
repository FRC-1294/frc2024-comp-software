package frc.robot.states.mech_states;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }
    
    @Override
    public void launch() {
        mLauncherSubsystem.indexUntilNoteLaunchedCommand().schedule();
    }

    @Override
    public void speakerPosition(){
        mSpeakerPositionCommand.schedule();
    }

    @Override
    public void ampPosition(){
        mAmpPositionCommand.schedule();
    }
    
    @Override
    public void trapPosition(){
        mTrapPositionCommand.schedule();
    }

    @Override
    public void handoffPosition(){
        mHandoffPositionCommand.schedule();
    }

    @Override
    public void aimStatePosition(AimState aim){
        mAimStatePositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(aim),
         mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER));
        mAimStatePositionCommand.schedule();
    }

}
