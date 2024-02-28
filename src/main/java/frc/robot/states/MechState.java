package frc.robot.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutonomousCommands.Handoff;
import frc.robot.constants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public abstract class MechState {
    protected final LauncherSubsystem mLauncherSubsystem;
    protected final IntakeSubsystem mIntakeSubsystem;
    protected final AimingSubsystem mAimingSubsystem;
    public final Command mHandoffPositionCommand;
    public  Command mSpeakerPositionCommand;
    public  Command mAmpPositionCommand;
    public  Command mTrapPositionCommand;
    public final Command mPreformHandoffCommand;
    public final Command mLaunchCommand;
    

    protected MechState (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        mLauncherSubsystem = launcherSubsystem;
        mIntakeSubsystem = intakeSubsystem;
        mAimingSubsystem = aimingSubsystem;

        mHandoffPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.HANDOFF), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.OFF));
        mSpeakerPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.SUBWOOFER), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.SPEAKER));
        mAmpPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.AMP), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.AMP));
        mTrapPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.TRAP), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(LauncherMode.TRAP));
        mPreformHandoffCommand = new Handoff(mIntakeSubsystem, mLauncherSubsystem);
        mLaunchCommand = mLauncherSubsystem.indexUntilNoteLaunchedCommand();
    }

    public void setLauncherSpeed(LauncherMode mode) {}

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

    public void index(){}

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

    public void resetEncoders() {}
}
