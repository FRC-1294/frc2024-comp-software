// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.states.MechState;
import frc.states.MechStates.ReadyForAim;
import frc.states.MechStates.ReadyForHandoff;
import frc.states.MechStates.ReadyForIntake;
import frc.states.MechStates.ReadyForLaunch;


public class DefaultMechCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private LauncherSubsystem mLauncherSubsystem = new LauncherSubsystem();
    //private AimingSubsystem mAimingSubsystem = new AimingSubsystem();

    private MechState readyForIntake;
    private MechState readyForHandoff;
    private MechState readyForAim;
    private MechState readyForLaunch;

    MechState mechState;

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) { //AimingSubsystem aimingSubsystem
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        //mAimingSubsystem = aimingSubsystem;

        addRequirements(mIntakeSubsystem, mLauncherSubsystem); //mAimingSubsystem

        readyForIntake = new ReadyForIntake(this, intakeSubsystem, launcherSubsystem);
        readyForHandoff = new ReadyForHandoff(this, intakeSubsystem, launcherSubsystem);
        readyForAim = new ReadyForAim(this, intakeSubsystem, launcherSubsystem);
        readyForLaunch = new ReadyForLaunch(this, intakeSubsystem, launcherSubsystem);

        mechState = determineState();
    }

    public MechState determineState() {
        if (getIntakeBeamBreak() && getIndexerBeamBreak()) { //intake is empty, note had been launched
            return readyForIntake;
        }
        else if (!getIntakeBeamBreak() && getIndexerBeamBreak()) { //note in intake AND ARM STATE IS STOW
            return readyForHandoff;
        }
        else if (!getIndexerBeamBreak()) { //note in indexer
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
        } 
        else if (Input.getA()) {
            mechState.setAmpSP();
        }
        else if (Input.getB()) {
            mechState.setTrapSP();
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

    public void setMechState(MechState mechState) {
        this.mechState = mechState;
    }

    public MechState getReadyForIntake() {
        return readyForIntake;
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

    public boolean getIntakeBeamBreak() {
        return mIntakeSubsystem.pieceInIntake();
    }

    public boolean getIndexerBeamBreak() {
        return mLauncherSubsystem.pieceInIndexer();
    }

    public boolean isVisionAligned() {
        return true; //tbd
    }

    public boolean isFlywheelAtSP() {
        return true; //tbd
    }

    public boolean isAimAtSP() {
        return true; //tbd
    }
}
