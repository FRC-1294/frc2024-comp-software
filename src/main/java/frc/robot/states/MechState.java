package frc.robot.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutonomousCommands.Handoff;
import frc.robot.constants.AimState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public abstract class MechState {
    protected final LauncherSubsystem mLauncherSubsystem;
    protected final IntakeSubsystem mIntakeSubsystem;
    protected final AimingSubsystem mAimingSubsystem;
    public final Command mHandoffPositionCommand;
    public final Command mSpeakerPositionCommand;
    public final Command mAmpPositionCommand;
    public final Command mTrapPositionCommand;
    public final Command mPreformHandoffCommand;
    public final Command mLaunchCommand;
    public Command mAimStatePositionCommand;
    

    protected MechState (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        mLauncherSubsystem = launcherSubsystem;
        mIntakeSubsystem = intakeSubsystem;
        mAimingSubsystem = aimingSubsystem;

        mHandoffPositionCommand = mAimingSubsystem.waitUntilSetpoint(AimState.HANDOFF);
        mSpeakerPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.SUBWOOFER), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER));
        mAmpPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.AMP),mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.AMP));
        mTrapPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.TRAP), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.TRAP));
        mPreformHandoffCommand = new Handoff(mIntakeSubsystem, mLauncherSubsystem);
        mLaunchCommand = mLauncherSubsystem.indexUntilNoteLaunchedCommand();
    }

    public void setLauncherSpeed(AimState state) {}

    public void brakeLauncher() {
        new InstantCommand(()->mLauncherSubsystem.stopLauncher(),mLauncherSubsystem).schedule();
    }
    public void brakeIndexer() {
        new InstantCommand(()->mLauncherSubsystem.stopIndexer(),mLauncherSubsystem).schedule();
    }
    public void brakeIntake() {
       new InstantCommand(()->mIntakeSubsystem.stopMotors(),mIntakeSubsystem).schedule();
    }
    public void runIntakeMotors(){}

    public void preformHandoff(){}

    public void index(double vel){}

    public void launch() {}

    public void controlWrist(double increment) {}

    public void controlElevator(double increment) {}

    public void ClimbExtendedState() {}
    
    public void ClimbRetractedState() {}

    public void setWristSP(AimState state) {}
    
    public void handoffPosition(){}

    public void speakerPosition(){}

    public void ampPosition(){}
    
    public void trapPosition(){}

    public void aimStatePosition(AimState state){}

    public void resetEncoders() {}
}
