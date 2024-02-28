package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForHandoff extends MechState {

    public ReadyForHandoff (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);

        mSpeakerPositionCommand = mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER);
        mAmpPositionCommand = mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.AMP);
        mTrapPositionCommand = mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.TRAP);
    }

    @Override
    public void preformHandoff(){
        mPreformHandoffCommand.schedule();
    }

    @Override
    public void brakeIntake() {
        new InstantCommand(()->this.mIntakeSubsystem.outerMotorAtSpeed(0)).schedule();
    }

    @Override
    public void speakerPosition() {
        mSpeakerPositionCommand.schedule();
    }

    @Override
    public void ampPosition() {
        mAmpPositionCommand.schedule();
    }

    public void trapPosition() {
        mTrapPositionCommand.schedule();
    }
}
