package frc.states.mechstates;

import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem) {
        super(launcherSubsystem);
    }

    @Override
    public void setSpeakerSP() {
        //nothing
    }

    @Override
    public void setAmpSP() {
        //nothing
    }

    @Override
    public void setTrapSP() {
        //nothing
    }

    @Override
    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
    }
}
