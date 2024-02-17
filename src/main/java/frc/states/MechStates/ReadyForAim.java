package frc.states.MechStates;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;

public class ReadyForAim extends MechState {

    private final AimingSubsystem mAimingSubsystem;

    public ReadyForAim (LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        super(launcherSubsystem);
        mAimingSubsystem = aimingSubsystem;
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
    public void controlWrist() {
        double increment = Input.getLeftStickY() * AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT;
        mAimingSubsystem.changeDesiredWristRotation(increment);
    }

    @Override
    public void controlElevator() {
        double increment = Input.getRightStickY() * AimingConstants.MAX_WRIST_TELEOP_INCREMENT;
        mAimingSubsystem.changeDesiredElevatorPosition(increment);
    }
}
