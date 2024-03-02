package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForHandoff extends MechState {

    public ReadyForHandoff (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
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
        mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER).schedule();
    }
    @Override
    public void ampPosition() {
        mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.AMP).schedule();
    }

    public void trapPosition() {
        mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.TRAP).schedule();
    }
}
