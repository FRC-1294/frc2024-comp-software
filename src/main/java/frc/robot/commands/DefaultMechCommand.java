// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;
import frc.states.mechstates.Intaken;
import frc.states.mechstates.ReadyForAim;
import frc.states.mechstates.ReadyForHandoff;
import frc.states.mechstates.ReadyForIntake;
import frc.states.mechstates.ReadyForLaunch;

public class DefaultMechCommand extends Command {
    private final IntakeSubsystem mIntakeSubsystem;
    private final LauncherSubsystem mLauncherSubsystem;
    private final AimingSubsystem mAimingSubsystem;

    private LauncherMode mLauncherMode = LauncherMode.OFF;
    private AimState mAimState = AimState.STOW;

    private final MechState mReadyForIntake;
    private final MechState mIntaken;
    private final MechState mReadyForHandoff;
    private final MechState mReadyForAim;
    private final MechState mReadyForLaunch;

    private MechState mMechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        mReadyForIntake = new ReadyForIntake(intakeSubsystem, launcherSubsystem);
        mIntaken = new Intaken(launcherSubsystem);
        mReadyForHandoff = new ReadyForHandoff(intakeSubsystem, launcherSubsystem);
        mReadyForAim = new ReadyForAim(launcherSubsystem, aimingSubsystem);
        mReadyForLaunch = new ReadyForLaunch(launcherSubsystem);

        mMechState = determineState();
    }

    public MechState determineState() {
        if (getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return mReadyForIntake;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return mIntaken;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak() && mAimState == AimState.STOW) {
            return mReadyForHandoff;
        }
        else if (!getIndexerBeamBreak()) {
            return mReadyForAim;
        }
        else if (getIndexerBeamBreak() && isFlywheelAtSP() && isAimAtSP() && isVisionAligned() && Input.getRightBumper()) {
            return mReadyForLaunch;
        }
        return null;
    }

    @Override
    public void execute() {
        if (Input.getX()) {
            mMechState.setFlywheelOff();
        }
        else if (Input.getY()) {
            mMechState.setSpeakerSP();
            if (mMechState != mReadyForLaunch) {
                mLauncherMode = LauncherMode.SPEAKER;
            }
            if (mMechState == mReadyForAim) {
                mAimState = AimState.SPEAKER;
            }
        } 
        else if (Input.getA()) {
            mMechState.setAmpSP();
            if (mMechState != mReadyForLaunch) {
                mLauncherMode = LauncherMode.AMP;
            }
            if (mMechState == mReadyForAim) {
                mAimState = AimState.AMP;
            }
        }
        else if (Input.getB()) {
            mMechState.setTrapSP();
            if (mMechState != mReadyForLaunch) {
                mLauncherMode = LauncherMode.TRAP;
            }
            if (mMechState == mReadyForAim) {
                mAimState = AimState.CLIMB;
            }
        }
        if (Input.getLeftBumper()) {
            mMechState.intake();
        }
        if (Input.getRightBumper()) {
            mMechState.launch();
        }
        if (Math.abs(Input.getLeftStickY()) > 0) {
            mMechState.controlWrist();
        }
        if (Math.abs(Input.getRightStickY()) > 0) {
            mMechState.controlElevator();
        }
        if (Input.getDPad() == Input.DPADUP) {
            mMechState.setElevatorSPtoStage();
        }
        else if (Input.getDPad() == Input.DPADDOWN) {
            mMechState.setElevatorSPtoBase();
        }
        if (Input.getDPad() == Input.DPADLEFT) {
            mMechState.resetEncoders();
        }

        runAction();

        mMechState = determineState();
    }

    //automatic actions
    public void runAction() {
        if (mMechState == mReadyForIntake) {
            mAimState = AimState.STOW;
            mAimingSubsystem.setDesiredSetpoint(mAimState);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mMechState == mIntaken) {
            //prepare momentum for handoff
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
            mAimState = AimState.STOW;
            mAimingSubsystem.setDesiredSetpoint(mAimState);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mMechState == mReadyForHandoff) {
            //handoff
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mMechState == mReadyForAim) {
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
            mAimingSubsystem.setDesiredSetpoint(mAimState);
        }
        else if (mMechState == mReadyForLaunch) {
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
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

    public void setMechState(MechState mechState) {
        this.mMechState = mechState;
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
