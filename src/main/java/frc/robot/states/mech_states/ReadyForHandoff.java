package frc.robot.states.mech_states;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForHandoff extends MechState {

    public ReadyForHandoff (LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem, IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem, aimingSubsystem, intakeSubsystem);
    }

    @Override
    public void intakeInnerMotor() {
        mIntakeSubsystem.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_ACTIVE);
    }
    
    @Override
    public void index() {
        mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_HANDOFF);
    }
}
