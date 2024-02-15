package frc.states.MechStates;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForAim implements MechState {

    DefaultMechCommand mMechCommand;
    
    IntakeSubsystem mIntakeSubsystem;
    LauncherSubsystem mLauncherSubsystem;
    AimingSubsystem mAimingSubsystem;

    public ReadyForAim (DefaultMechCommand mechCommand, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mMechCommand = mechCommand;
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;
    }

    @Override
    public void setFlywheelOff() {
        mLauncherSubsystem.stopLauncher();
    }

    @Override
    public void setSpeakerSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);
        mAimingSubsystem.setDesiredSetpoint(AimState.SPEAKER);
    }

    @Override
    public void setAmpSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);
        mAimingSubsystem.setDesiredSetpoint(AimState.AMP);
    }

    @Override
    public void setTrapSP() {
        mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);
        mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB);
    }
    
    @Override
    public void intake() {

    }

    @Override
    public void launch() {

    }

    @Override
    public void controlWrist() {
        double increment = Input.getLeftStickY() * AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT;
        mAimingSubsystem.changeDesiredWristRotation(increment);
    }

    @Override
    public void controlElevator() {
        double increment = Input.getRightStickY() * AimingConstants.MAX_WRIST_TELEOP_INCREMENT;
        mAimingSubsystem.changeDesiredElevatorPosition(increment);
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
