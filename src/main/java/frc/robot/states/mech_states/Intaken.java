package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.Command;
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
    public Command brakeIntake() {
        return new InstantCommand(()->mIntakeSubsystem.outerMotorAtSpeed(0));
    }

    @Override
    public Command handoffPosition(){
        return mHandoffPositionCommand;
    }
    @Override
    public Command speakerPosition() {
        return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER);
    }
    @Override
    public Command ampPosition() {
        return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.AMP);
    }

    public Command trapPosition() {
        return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.TRAP);
    }
}
