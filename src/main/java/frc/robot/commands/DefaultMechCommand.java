// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.states.mech_states.Intaken;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.states.mech_states.ReadyForHandoff;
import frc.robot.states.mech_states.ReadyForIntake;
import frc.robot.states.mech_states.ReadyForLaunch;
import frc.robot.states.mech_states.UltraInstinct;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Input;
import frc.robot.subsystems.LauncherSubsystem;

public class DefaultMechCommand extends Command {
    private static IntakeSubsystem mIntakeSubsystem;
    private static LauncherSubsystem mLauncherSubsystem;
    private static AimingSubsystem mAimingSubsystem;

    private static MechState mReadyForIntake;
    private static MechState mIntaken;
    private static MechState mReadyForHandoff;
    private static MechState mReadyForAim;
    private static MechState mReadyForLaunch;
    private static MechState mUltraInstinct;
    private static boolean noteCurrentlyLaunching = false;
    private static boolean returnFromLaunch = true;

    private static boolean mUseUltraInstinct = false;
    private static MechState mMechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        //addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        mReadyForIntake = new ReadyForIntake(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mReadyForHandoff = new ReadyForHandoff(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mReadyForAim = new ReadyForAim(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mReadyForLaunch = new ReadyForLaunch(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mUltraInstinct = new UltraInstinct(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mMechState = determineState();
    }

    public static MechState determineState() {
        if (mUseUltraInstinct) {
            return mUltraInstinct;
        }
        if (!getIntakeBeamBreak() && !getIndexerBeamBreak()) {
            return mReadyForIntake;
        }
        else if (getIntakeBeamBreak() && !getIndexerBeamBreak() && mAimingSubsystem.getCurrentState() == AimState.HANDOFF) {
            return mReadyForHandoff;
        }
        else if (getIntakeBeamBreak() && !getIndexerBeamBreak()) {
            return mIntaken;
        }
        else if (getIndexerBeamBreak() && isFlywheelAtSP() && isAimAtSP() && isVisionAligned()) {
            returnFromLaunch = true;
            return mReadyForLaunch;
        }
        else if (getIndexerBeamBreak()) {
            return mReadyForAim;
        }
        return mUltraInstinct;
    }

    @Override
    public void execute() {
        if (mMechState.mLaunchCommand.isScheduled()){
            noteCurrentlyLaunching = !mMechState.mLaunchCommand.isFinished();
        }

        if (Input.getX()) {
            mMechState.brakeLauncher();
        }
        else if (Input.getY()) {
            mMechState.speakerPosition();
        } 
        else if (Input.getA()) {
            mMechState.ampPosition();
        }
        else if (Input.getB()) {
            mMechState.trapPosition();
        }
        if (Input.getLeftBumper()) {
            mMechState.runIntakeMotors();
        } else{
            mMechState.brakeIntake();
        }
        if (Input.getRightBumper()) {
            mMechState.launch();
        }
        if (Math.abs(Input.getLeftStickY()) > 0.1) {
            mMechState.controlWrist(Input.getLeftStickY()*AimingConstants.MAX_WRIST_TELEOP_INCREMENT);
        }
        if (Math.abs(Input.getRightStickY()) > 0.1) {
            mMechState.controlElevator(Input.getRightStickY()*AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT);
        }
        if (Math.abs(Input.getLeftTrigger()) > LauncherConstants.INDEX_TRIGGER_DEADZONE) {
            mMechState.index((Input.getLeftTrigger()-LauncherConstants.INDEX_TRIGGER_DEADZONE)*(Input.getReverseButton() ? 1 : -1));
        } 
        if (Math.abs(Input.getRightTrigger()) > LauncherConstants.INDEX_TRIGGER_DEADZONE) {
            mMechState.index((Input.getRightTrigger()-LauncherConstants.INDEX_TRIGGER_DEADZONE)*(Input.getReverseButton() ? 1 : -1));
        } 
        if (Input.getDPad() == Input.DPADUP) {
            mMechState.ClimbExtendedState();
        } else if (Input.getDPad() == Input.DPADDOWN) {
            mMechState.ClimbRetractedState();
        }
        if (Input.getDPad() == Input.DPADRIGHT){
            mUseUltraInstinct = true;
        }
        if (Input.getDPad() == Input.DPADLEFT){
            mUseUltraInstinct = false;
        }

        runAction();
        mMechState = determineState();

        SmartDashboard.putString("CurrentState", mMechState.getClass().getSimpleName());
    }

    //automatic actions
    public void runAction() {
        if (mMechState == mReadyForIntake) {
            if (!noteCurrentlyLaunching){
                mMechState.brakeIndexer();
                if (returnFromLaunch){
                    mMechState.brakeLauncher();
                    returnFromLaunch = false;
                }
            }
            if (!mMechState.mHandoffPositionCommand.isScheduled()){
                mMechState.handoffPosition();
            }
        }
        else if (mMechState == mIntaken) {
            mMechState.brakeIntake();
            if (!noteCurrentlyLaunching){
                mMechState.brakeIndexer();
                if (returnFromLaunch){
                    mMechState.brakeLauncher();
                    returnFromLaunch = false;
                }
            }
            if (!mMechState.mHandoffPositionCommand.isScheduled()){
                mMechState.handoffPosition();
            }
        }
        else if (mMechState == mReadyForHandoff) {
            mMechState.brakeIntake();
            if (!mMechState.mPreformHandoffCommand.isScheduled()){
                mMechState.preformHandoff();
            }
        }
        else if (mMechState == mReadyForAim) {
            mMechState.brakeIntake();
            //No Automation Yet
        }
        else if (mMechState == mReadyForLaunch) {
            mMechState.brakeIntake();
            //Need Operator Confirmation
        }
    }

    public static boolean getIntakeBeamBreak() {
        return IntakeSubsystem.pieceInIntake();
    }

    public static boolean getIndexerBeamBreak() {
        return mLauncherSubsystem.pieceInIndexer();
    }

    public static boolean isVisionAligned() {
        return true; //V2: return LimelightOB.getNoteAlignmentCommand(swerve)
    }

    public static boolean isFlywheelAtSP() {
        return mLauncherSubsystem.isLauncherReady();
    }

    public static boolean isAimAtSP() {
        return mAimingSubsystem.atSetpoints();
    }

    public static MechState getReadyForIntake() {
        return mReadyForIntake;
    }

    public static MechState getIntaken() {
        return mIntaken;
    }

    public static MechState getReadyForHandoff() {
        return mReadyForHandoff;
    }

    public static MechState getReadyForAim() {
        return mReadyForAim;
    }

    public static MechState getReadyForLaunch() {
        return mReadyForLaunch;
    }
}
