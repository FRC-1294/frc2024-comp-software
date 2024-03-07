package frc.robot.states.mech_states;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }
    
    @Override
    public Command launch() {
       return mLauncherSubsystem.indexUntilNoteLaunchedCommand();
    }

    @Override
    public Command speakerPosition(){
       return mSpeakerPositionCommand;
    }

    @Override
    public Command ampPosition(){
       return mAmpPositionCommand;
    }
    
    @Override
    public Command trapPosition(){
       return mTrapPositionCommand;
    }

    @Override
    public Command handoffPosition(){
       return mHandoffPositionCommand;
    }

    @Override
    public Command podiumPosition() {
       return mPodiumPositionCommand;
    }
    @Override
    public Command index(double vel){
       return new InstantCommand(() -> mLauncherSubsystem.runIndexer(vel),mLauncherSubsystem);
    }

    @Override
    public Command aimStatePosition(AimState aim){
        mAimStatePositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(aim),
        mLauncherSubsystem.waitUntilFlywheelSetpointCommand(aim));
        return mAimStatePositionCommand;
    }
    @Override
    public Command emergencyOuttake(){
        mLaunchCommand.cancel();
       return new SequentialCommandGroup(new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.OUTTAKE),
            mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.OUTTAKE)),
            new InstantCommand(()->mLauncherSubsystem.runIndexer(-0.6))); 
    }

    @Override
    public Command staticAutoAim(){
        return mStaticAutoAimCommand;
    }
}
