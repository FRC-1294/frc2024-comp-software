package frc.states.MechStates;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForHandoff extends MechState {

    private final IntakeSubsystem mIntakeSubsystem;
    private final LauncherSubsystem mLauncherSubsystem;

    public ReadyForHandoff (IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
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
    
    @Override
    public void intake() {
        mIntakeSubsystem.getAutomousIntakeCommand();
    }
}
