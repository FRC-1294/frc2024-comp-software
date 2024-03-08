package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForAim extends MechState {

    public ReadyForAim (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }
    
    @Override
    public Command index(double vel){
       return new InstantCommand(() -> mLauncherSubsystem.runIndexer(vel),mLauncherSubsystem);
    }

    @Override
    public Command speakerPosition(){
      DefaultMechCommand.mDesiredState = AimState.SUBWOOFER;
      return MechState.mSpeakerPositionCommand;
    }

    @Override
    public Command ampPosition(){
      DefaultMechCommand.mDesiredState = AimState.AMP;
       return MechState.mAmpPositionCommand;
    }
    
    @Override
    public Command trapPosition(){
      DefaultMechCommand.mDesiredState = AimState.TRAP;
       return MechState.mTrapPositionCommand;
    }

    @Override
    public Command handoffPosition(){
      DefaultMechCommand.mDesiredState = AimState.HANDOFF;
       return MechState.mHandoffPositionCommand;
    }

    @Override
    public Command podiumPosition() {
      DefaultMechCommand.mDesiredState = AimState.PODIUM;
       return MechState.mPodiumPositionCommand;
    }

    @Override
    public Command emergencyOuttake(){
        mLaunchCommand.cancel();
       return new SequentialCommandGroup(new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.OUTTAKE),
            mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.OUTTAKE)),
            new InstantCommand(()->mLauncherSubsystem.runIndexer(-0.6))); 
    }

    @Override
    public Command staticAutoAim(){
        return MechState.mStaticAutoAimCommand;
    }
}
