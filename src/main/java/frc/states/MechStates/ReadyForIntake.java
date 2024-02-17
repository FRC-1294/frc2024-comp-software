package frc.states.MechStates;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForIntake extends MechState {

    private final IntakeSubsystem mIntakeSubsystem;

    public ReadyForIntake (IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        super(launcherSubsystem);
        mIntakeSubsystem = intakeSubsystem;
    }

    @Override
    public void intake() {
        mIntakeSubsystem.getAutomousIntakeCommand();
    }
}
