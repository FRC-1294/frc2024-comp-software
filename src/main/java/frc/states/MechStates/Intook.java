package frc.states.MechStates;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class Intook extends MechState {
    private final LauncherSubsystem mLauncherSubsystem;


    public Intook (DefaultMechCommand mechCommand, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mLauncherSubsystem = launcherSubsystem;
    }

    @Override
    public void setFlywheelOff() {
        mLauncherSubsystem.stopLauncher();
    }

    @Override
    public void setSpeakerSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);
    }

    @Override
    public void setAmpSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);
    }

    @Override
    public void setTrapSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);
    }
}
