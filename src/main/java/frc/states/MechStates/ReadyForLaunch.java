package frc.states.MechStates;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForLaunch extends MechState {

    private final LauncherSubsystem mLauncherSubsystem;

    public ReadyForLaunch (LauncherSubsystem launcherSubsystem) {
        mLauncherSubsystem = launcherSubsystem;
    }

    @Override
    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
    }
}
