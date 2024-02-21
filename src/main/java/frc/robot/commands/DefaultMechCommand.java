// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.states.MechState;
import frc.robot.states.mech_states.Intaken;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.states.mech_states.ReadyForHandoff;
import frc.robot.states.mech_states.ReadyForIntake;
import frc.robot.states.mech_states.ReadyForLaunch;
import frc.robot.states.mech_states.UltraInstinct;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class DefaultMechCommand extends Command {
    private final IntakeSubsystem mIntakeSubsystem;
    private final LauncherSubsystem mLauncherSubsystem;
    private final AimingSubsystem mAimingSubsystem;

    private final MechState mReadyForIntake;
    private final MechState mIntaken;
    private final MechState mReadyForHandoff;
    private final MechState mReadyForAim;
    private final MechState mReadyForLaunch;
    private final MechState mUltraInstinct;

    private boolean mUseUltraInstinct = false;

    private MechState mMechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        mReadyForIntake = new ReadyForIntake(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mIntaken = new Intaken(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mReadyForHandoff = new ReadyForHandoff(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mReadyForAim = new ReadyForAim(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mReadyForLaunch = new ReadyForLaunch(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mUltraInstinct = new UltraInstinct(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        mMechState = determineState();
    }

    public MechState determineState() {
        if (mUseUltraInstinct) {
            return mUltraInstinct;
        }
        if (getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return mReadyForIntake;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return mIntaken;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak() && mAimingSubsystem.getCurrentState() == AimState.HANDOFF) {
            return mReadyForHandoff;
        }
        else if (!getIndexerBeamBreak()) {
            return mReadyForAim;
        }
        else if (!getIndexerBeamBreak() && isFlywheelAtSP() && isAimAtSP() && isVisionAligned() && Input.getRightBumper()) {
            return mReadyForLaunch;
        }
        return mUltraInstinct;
    }

    @Override
    public void execute() {
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
            mMechState.intake();
        }
        if (Input.getRightBumper()) {
            mMechState.launch();
        }
        if (Math.abs(Input.getLeftStickY()) > 0) {
            mMechState.controlWrist(Input.getLeftStickY()*AimingConstants.MAX_WRIST_TELEOP_INCREMENT);
        }
        if (Math.abs(Input.getRightStickY()) > 0) {
            mMechState.controlElevator(Input.getLeftStickY()*AimingConstants.MAX_WRIST_TELEOP_INCREMENT);
        }
        if (Input.getDPad() == Input.DPADUP) {
            mMechState.setElevatorSP(AimState.CLIMB);
        } else if (Input.getDPad() == Input.DPADDOWN) {
            mMechState.setElevatorSP(AimState.STOW);
        }
        if (Input.getDPad() == Input.DPADRIGHT){
            mUseUltraInstinct = !mUseUltraInstinct;
        }

        runAction();
        mMechState = determineState();
    }

    //automatic actions
    public void runAction() {
        if (mMechState == mReadyForIntake) {
            mMechState.handoffPosition();
        }
        else if (mMechState == mIntaken) {
            mMechState.handoffPosition();
        }
        else if (mMechState == mReadyForHandoff) {
            mMechState.intake();
            mMechState.index();
        }
        else if (mMechState == mReadyForAim) {
            //No Automation Yet
        }
        else if (mMechState == mReadyForLaunch) {
            //Need Operator Confirmation
        }
    }

    public boolean getIntakeBeamBreak() {
        return mIntakeSubsystem.pieceInIntake();
    }

    public boolean getIndexerBeamBreak() {
        return mLauncherSubsystem.pieceInIndexer();
    }

    public boolean isVisionAligned() {
        return true; //V2: return LimelightOB.getNoteAlignmentCommand(swerve)
    }

    public boolean isFlywheelAtSP() {
        return mLauncherSubsystem.isLauncherReady();
    }

    public boolean isAimAtSP() {
        return mAimingSubsystem.atSetpoints();
    }

    public MechState getReadyForIntake() {
        return mReadyForIntake;
    }

    public MechState getIntaken() {
        return mIntaken;
    }

    public MechState getReadyForHandoff() {
        return mReadyForHandoff;
    }

    public MechState getReadyForAim() {
        return mReadyForAim;
    }

    public MechState getReadyForLaunch() {
        return mReadyForLaunch;
    }
}
