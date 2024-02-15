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
    private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private LauncherSubsystem mLauncherSubsystem = new LauncherSubsystem();
    private AimingSubsystem mAimingSubsystem = new AimingSubsystem();

    private LauncherMode mLauncherMode = LauncherMode.OFF;
    private AimState mAimState = AimState.STOW;

    private MechState readyForIntake;
    private MechState intook;
    private MechState readyForHandoff;
    private MechState readyForAim;
    private MechState readyForLaunch;

    MechState mechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        readyForIntake = new ReadyForIntake(this, intakeSubsystem, launcherSubsystem, aimingSubsystem);
        intook = new Intook(this, intakeSubsystem, launcherSubsystem, aimingSubsystem);
        readyForHandoff = new ReadyForHandoff(this, intakeSubsystem, launcherSubsystem, aimingSubsystem);
        readyForAim = new ReadyForAim(this, intakeSubsystem, launcherSubsystem, aimingSubsystem);
        readyForLaunch = new ReadyForLaunch(this, intakeSubsystem, launcherSubsystem, aimingSubsystem);

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
        mLauncherMode = null;
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

        MechState newMechState = determineState();

        if (newMechState != mechState) {
            mechState = newMechState;
        }
    }

    public void run_action() {
        if (mechState == readyForIntake) {
            mIntakeSubsystem.getAutomousIntakeCommand();
            mAimState = AimState.STOW;
            if (mLauncherMode != LauncherMode.OFF) {
                mLauncherSubsystem.setLauncherMode(mLauncherMode);
            }
        }
        else if (mechState == intook) {
            mAimState = AimState.STOW;
            if (mLauncherMode != LauncherMode.OFF) {
                mLauncherSubsystem.setLauncherMode(mLauncherMode);
            }
        }
        else if (mechState == readyForHandoff) {
            //run intake to handoff
            mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_DEFAULT);
            if (mLauncherMode != LauncherMode.OFF) {
                mLauncherSubsystem.setLauncherMode(mLauncherMode);
            }
        }
        else if (mechState == readyForAim) {
            if (mLauncherMode != LauncherMode.OFF) {
                mLauncherSubsystem.setLauncherMode(mLauncherMode);
            }
            if (mAimState != AimState.STOW) {
                mAimingSubsystem.setDesiredSetpoint(mAimState);
            }
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
