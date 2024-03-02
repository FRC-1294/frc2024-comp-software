package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Intaken extends MechState {

    public Intaken(LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public void brakeIntake() {
        new InstantCommand(()->mIntakeSubsystem.outerMotorAtSpeed(0));
    }

    @Override
    public void handoffPosition(){
        mHandoffPositionCommand.schedule();
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
