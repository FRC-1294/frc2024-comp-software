package frc.states.mechstates;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForHandoff extends MechState {

    private final IntakeSubsystem mIntakeSubsystem;

    public ReadyForHandoff (IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        super(launcherSubsystem);
        mIntakeSubsystem = intakeSubsystem;
    }
    
    @Override
    public void intake() {
        mIntakeSubsystem.getAutomousIntakeCommand();
    }
}
