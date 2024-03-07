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

public class ReadyForAim extends MechState {

    public ReadyForAim (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
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
    public Command podiumPosition(){
       return mPodiumPositionCommand;
    }

    @Override
    public Command emergencyOuttake(){
        mLaunchCommand.cancel();
       return new SequentialCommandGroup(new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.OUTTAKE),
            mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.OUTTAKE)),
            new InstantCommand(()->mLauncherSubsystem.runIndexer(-0.6))); 
    }
}
