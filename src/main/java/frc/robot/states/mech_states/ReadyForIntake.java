package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimState;
import frc.robot.constants.IntakeConstants;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForIntake extends MechState {

    public ReadyForIntake(LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public Command runIntakeMotors() {
        return new InstantCommand(() -> {
        mIntakeSubsystem.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_AQUIRE);
        mIntakeSubsystem.outerMotorAtSpeed(IntakeConstants.OUTER_INTAKE_SPEED_ACTIVE);},mIntakeSubsystem);
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

    @Override
    public Command podiumPosition() {
        return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.PODIUM);
    }

    public Command trapPosition() {
       return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.TRAP);
    }
}
