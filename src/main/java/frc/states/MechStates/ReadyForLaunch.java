package frc.states.MechStates;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem) {
        super(launcherSubsystem);
    }

    @Override
    public void setSpeakerSP() {
        
    }

    @Override
    public void setAmpSP() {
        
    }

    @Override
    public void setTrapSP() {
        
    }

    @Override
    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
    }
}
