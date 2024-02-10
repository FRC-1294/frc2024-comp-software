package frc.states.MechStates;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForIntake implements MechState {

    DefaultMechCommand mMechCommand;

    IntakeSubsystem mIntakeSubsystem;
    LauncherSubsystem mLauncherSubsystem;
    //AimingSubsystem mAimingSubsystem;

    public ReadyForIntake (DefaultMechCommand mechCommand, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) { //AimingSubsystem aimingSubsystem
        mMechCommand = mechCommand;
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        //mAimingSubsystem = aimingSubsystem;
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

    @Override
    public void launch() {

    }

    @Override
    public void controlWrist() {

    }

    @Override
    public void controlElevator() {

    }

    @Override
    public void setElevatorSPtoStage() {

    }

    @Override
    public void setElevatorSPtoBase() {

    }

    @Override
    public void resetEncoders() {

    }
}
