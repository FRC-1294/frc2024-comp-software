package frc.states.MechStates;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForLaunch implements MechState {

    DefaultMechCommand mMechCommand;

    IntakeSubsystem mIntakeSubsystem;
    LauncherSubsystem mLauncherSubsystem;
    //AimingSubsystem mAimingSubsystem;

    public ReadyForLaunch (DefaultMechCommand mechCommand, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) { //AimingSubsystem aimingSubsystem
        mMechCommand = mechCommand;
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        //mAimingSubsystem = aimingSubsystem;
    }

    @Override
    public void setFlywheelOff() {

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
    public void intake() {

    }

    @Override
    public void launch() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
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
