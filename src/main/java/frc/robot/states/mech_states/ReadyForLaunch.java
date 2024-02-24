package frc.robot.states.mech_states;

import frc.robot.constants.LauncherConstants;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem, IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem, aimingSubsystem, intakeSubsystem);
    }

    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCH);
    }

}
