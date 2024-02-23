package frc.robot.states.mech_states;

import frc.robot.commands.AutonomousCommands.Handoff;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForHandoff extends MechState {

    public ReadyForHandoff (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public void preformHandoff(){
        new Handoff(mIntakeSubsystem, mLauncherSubsystem).schedule();
    }
}
