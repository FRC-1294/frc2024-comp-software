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
import frc.states.MechStates.Intook;
import frc.states.MechStates.ReadyForAim;
import frc.states.MechStates.ReadyForHandoff;
import frc.states.MechStates.ReadyForIntake;
import frc.states.MechStates.ReadyForLaunch;

public class DefaultMechCommand extends Command {
    private final IntakeSubsystem mIntakeSubsystem;
    private final LauncherSubsystem mLauncherSubsystem;
    private final AimingSubsystem mAimingSubsystem;

    private LauncherMode mLauncherMode = LauncherMode.OFF;
    private AimState mAimState = AimState.STOW;

    private final MechState readyForIntake;
    private final MechState intook;
    private final MechState readyForHandoff;
    private final MechState readyForAim;
    private final MechState readyForLaunch;

    private MechState mechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        readyForIntake = new ReadyForIntake(intakeSubsystem, launcherSubsystem);
        intook = new Intook(launcherSubsystem);
        readyForHandoff = new ReadyForHandoff(intakeSubsystem, launcherSubsystem);
        readyForAim = new ReadyForAim(launcherSubsystem, aimingSubsystem);
        readyForLaunch = new ReadyForLaunch(launcherSubsystem);

        mechState = determineState();
    }

    public MechState determineState() {
        if (getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return readyForIntake;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak()) {
            return intook;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak() && mAimState == AimState.STOW) {
            return readyForHandoff;
        }
        else if (!getIndexerBeamBreak()) {
            return readyForAim;
        }
        else if (getIndexerBeamBreak() && isFlywheelAtSP() && isAimAtSP() && isVisionAligned() && Input.getRightBumper()) {
            return readyForLaunch;
        }
        return null;
    }

    @Override
    public void execute() {
        if (Input.getX()) {
            mechState.setFlywheelOff();
        }
        else if (Input.getY()) {
            mechState.setSpeakerSP();
            if (mechState != readyForLaunch) {
                mLauncherMode = LauncherMode.SPEAKER;
            }
            if (mechState == readyForAim) {
                mAimState = AimState.SPEAKER;
            }
        } 
        else if (Input.getA()) {
            mechState.setAmpSP();
            if (mechState != readyForLaunch) {
                mLauncherMode = LauncherMode.AMP;
            }
            if (mechState == readyForAim) {
                mAimState = AimState.AMP;
            }
        }
        else if (Input.getB()) {
            mechState.setTrapSP();
            if (mechState != readyForLaunch) {
                mLauncherMode = LauncherMode.TRAP;
            }
            if (mechState == readyForAim) {
                mAimState = AimState.CLIMB;
            }
        }
        if (Input.getLeftBumper()) {
            mechState.intake();
        }
        if (Input.getRightBumper()) {
            mechState.launch();
        }
        if (Math.abs(Input.getLeftStickY()) > 0) {
            mechState.controlWrist();
        }
        if (Math.abs(Input.getRightStickY()) > 0) {
            mechState.controlElevator();
        }
        if (Input.getDPad() == Input.DPADUP) {
            mechState.setElevatorSPtoStage();
        }
        else if (Input.getDPad() == Input.DPADDOWN) {
            mechState.setElevatorSPtoBase();
        }
        if (Input.getDPad() == Input.DPADLEFT) {
            mechState.resetEncoders();
        }

        runAction();

        mechState = determineState();
    }

    //automatic actions
    public void runAction() {
        if (mechState == readyForIntake) {
            mAimState = AimState.STOW;
            mAimingSubsystem.setDesiredSetpoint(mAimState);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mechState == intook) {
            //prepare momentum for handoff
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
            mAimState = AimState.STOW;
            mAimingSubsystem.setDesiredSetpoint(mAimState);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mechState == readyForHandoff) {
            //handoff
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
        }
        else if (mechState == readyForAim) {
            mLauncherSubsystem.setLauncherMode(mLauncherMode);
            mAimingSubsystem.setDesiredSetpoint(mAimState);
        }
        else if (mechState == readyForLaunch) {
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
        this.mechState = mechState;
    }

    public MechState getReadyForIntake() {
        return readyForIntake;
    }

    public MechState getIntook() {
        return intook;
    }

    public MechState getReadyForHandoff() {
        return readyForHandoff;
    }

    public MechState getReadyForAim() {
        return readyForAim;
    }

    public MechState getReadyForLaunch() {
        return readyForLaunch;
    }
}
