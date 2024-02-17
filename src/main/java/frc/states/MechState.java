package frc.states;

import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.LauncherSubsystem;

public abstract class MechState {
    protected final LauncherSubsystem mLauncherSubsystem;

    protected MechState (LauncherSubsystem launcherSubsystem) {
        mLauncherSubsystem = launcherSubsystem;
    }

    public void setFlywheelOff() {
        mLauncherSubsystem.stopLauncher();
    }

    public void setSpeakerSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);
    }

    public void setAmpSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);
    }

    public void setTrapSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);
    }

    public void intake() {

    }

    public void launch() {

    }

    public void controlWrist() {

    }

    public void controlElevator() {

    }

    public void setElevatorSPtoStage() {

    }

    public void setElevatorSPtoBase() {

    }

    public void resetEncoders() {
        
    }
}
