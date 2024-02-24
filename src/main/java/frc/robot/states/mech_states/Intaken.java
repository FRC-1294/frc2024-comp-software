package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Intaken extends MechState {

    public Intaken(LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public void handoffPosition(){
        new InstantCommand(()->{
            mAimingSubsystem.setDesiredSetpoint(AimState.HANDOFF);
            mLauncherSubsystem.setLauncherMode(LauncherMode.OFF);},mLauncherSubsystem,mAimingSubsystem
        ).schedule();
    }

}
